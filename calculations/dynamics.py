"""Module for all dynamics calculations.
"""
import numpy as np
import math
import numba

import config
from environment import spider
from calculations import kinematics as kin
from calculations import mathtools as mathTools
from calculations import transformations as tf

A_TORQUE_POLYNOM = 0.0
B_TORQUE_POLYNOM = 2.9326
C_TORQUE_POLYNOM = -0.1779

#region public methods
@numba.jit(nopython = True, cache = True)
def getTorquesAndForcesOnLegsTips(jointsValues, currentsInMotors, spiderGravityVector, numberOfLegs = spider.NUMBER_OF_LEGS):
    """Calculate forces, applied to tips of all legs, from currents in motors.
    Args:
        jointsValues (list): 5x3 array of angles in joints.
        currentsInMotors (list): 5x3 array of currents in motors.
        spiderGravityVector(list): 1x3 gravity vector in spider's origin.

    Returns:
        tuple: 5x3 array of forces, applied to leg tips in x, y, z direction of spider's origin and 5x3x3 array of damped pseudo inverses of jacobian matrices.
    """
    torques = getTorquesInLegs(jointsValues, currentsInMotors, spiderGravityVector)
    forces = np.zeros((numberOfLegs, 3), dtype = np.float32)

    for legId, jointValues in enumerate(jointsValues):
        J = kin.spiderBaseToLegTipJacobi(legId, jointValues)
        Jhash = mathTools.dampedPseudoInverse(J)
        forces[legId] = np.dot(np.transpose(Jhash), torques[legId])
    
    return torques, forces
    
def getForceEllipsoidLengthInGivenDirection(legId, jointsValues, direction):
    """Calculate size of vector from center to the surface of force manipulability ellipsoid, in given direction.'

    Args:
        legId (int): Leg's id.
        jointsValues (list): 1x3 array of leg's joints values in radians.
        direction (list): 1x3 vector of direction in spider's origin.

    Returns:
        float: Length of vector from center to the surface of force ellipsoid in given direction.
    """
    J = kin.spiderBaseToLegTipJacobi(legId, jointsValues)
    A = np.linalg.inv(np.dot(J, np.transpose(J)))
    eigVals, eigVects = np.linalg.eig(A)

    # Unit vector in given direction in ellipsoid origin.
    eGrav = np.dot(eigVects, direction)

    ellipsoidSizeInGlobalNegY = math.sqrt(np.prod(eigVals) / (math.pow(eGrav[0], 2) * eigVals[1] * eigVals[2] + math.pow(eGrav[1], 2) * eigVals[0] * eigVals[2] + math.pow(eGrav[2], 2) * eigVals[0] * eigVals[1]))

    ed = np.array([0, -ellipsoidSizeInGlobalNegY, 0])
    ed = ed / np.linalg.norm(ed)
    ed = np.dot(np.linalg.inv(eigVects), ed)
    t = math.sqrt(np.prod(eigVals) / (math.pow(ed[0], 2) * eigVals[1] * eigVals[2] + math.pow(ed[1], 2) * eigVals[0] * eigVals[2] + math.pow(ed[2], 2) * eigVals[0] * eigVals[1]))

    return t 

def calculateDistributedForces(measuredTorques, jointsValues, legsIds, offloadLegId):
    """Calculated distributed forces from minimized torques.

    Args:
        measuredTorques (list): 5x3 array of measured torques in joints.
        jointsValues (list): 5x3 array of angles in joints, in radians.
        legsIds (list): List of used legs' ids.

    Returns:
        numpy.ndarray: 5x3 array of desired forces, to be applied on leg-tips.
    """
    W, Jx, JhashTransDiag = _getSpiderExternalForces(measuredTorques, jointsValues)

    # weights = np.eye(15)
    # weights[0, 0] = 0.1
    # weights[3, 3] = 0.1
    # weights[6, 6] = 0.1
    # weights[9, 9] = 0.1
    # weights[12, 12] = 0.1

    
    if len(offloadLegId):
        Jx = np.delete(Jx, range(offloadLegId[0] * 3, (offloadLegId[0] * 3) + 3), axis = 1)
        # weights = np.delete(weights, range(offloadLegId[0] * 3, (offloadLegId[0] * 3) + 3), axis = 0)
        # weights = np.delete(weights, range(offloadLegId[0] * 3, (offloadLegId[0] * 3) + 3), axis = 1)
        JhashTransDiag = np.delete(JhashTransDiag, range(offloadLegId[0] * 3, (offloadLegId[0] * 3) + 3), axis = 0)
        JhashTransDiag = np.delete(JhashTransDiag, range(offloadLegId[0] * 3, (offloadLegId[0] * 3) + 3), axis = 1)

    # Jxw = mathTools.weightedPseudoInverse(Jx, weights)
    distTorquesArray = np.dot(np.linalg.pinv(Jx), W)
    # distTorquesArray = np.dot(Jxw, W)
    distForcesArray = np.dot(JhashTransDiag, distTorquesArray)

    return np.reshape(distForcesArray, (len(legsIds), 3))

@numba.jit(nopython = True, cache = True)
def getGravityRotationMatrices(jointsValues, qb):
    q1, q2, q3 = jointsValues

    rotMatrices = np.zeros((3, 3, 3), dtype = np.float32)
    rotMatrices[0] = tf.R_B1(qb, q1)
    rotMatrices[1] = tf.R_B2(qb, q1, q2)
    rotMatrices[2] = tf.R_B3(qb, q1, q2, q3)

    return rotMatrices

@numba.jit(nopython = True, cache = True)
def getForceRotationMatrices(jointsValues):
    forceMatrices = np.zeros((2, 3, 3), dtype = np.float32)
    forceMatrices[0] = tf.R_23(jointsValues[2])
    forceMatrices[1] = tf.R_12(jointsValues[1])

    return forceMatrices

@numba.jit(nopython = True, cache = True)
def getTorquesInLegs(jointsValues, currentsInMotors, spiderGravityVector):
    """Calculate torques in leg-joints from measured currents. 

    Args:
        jointsValues (list): 5x3 array of angles in joints.
        currentsInMotors (list): 5x3 array of currents in motors.
        spiderGravityVector (list): 3x1 gravity vector in spider's origin.

    Returns:
        numpy.ndarray: 5x3 array of torques in motors.
    """
    currentsInMotors[:, 1] *= -1

    gravityTorques = getGravityCompensationTorques(jointsValues, spiderGravityVector)
    torques = (A_TORQUE_POLYNOM + B_TORQUE_POLYNOM * currentsInMotors + C_TORQUE_POLYNOM * currentsInMotors**2) - gravityTorques
    torquesArray = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
    for i in range(len(torques)):
        torquesArray[i] = torques[i]

    return torquesArray

@numba.jit(nopython = True, cache = True)
def getGravityCompensationTorques(jointsValues, spiderGravityVector, numberOfLegs = spider.NUMBER_OF_LEGS, numberOfMotorsInLeg = spider.NUMBER_OF_MOTORS_IN_LEG, angleBetweenLegs = spider.ANGLE_BETWEEN_LEGS):
    """Calculate torques in joints (for all legs), required to compensate movement, caused only by gravity.

    Args:
        jointsValues (list): 5x3 array of angles in joints, in radians.
        spiderGravityVector (list): 1x3 gravity vector in spider's origin.

    Returns:
        numpy.ndarray: 5x3 array of required torques in joints.
    """
    torques = np.zeros((numberOfLegs, numberOfMotorsInLeg), dtype = np.float32)
    for legId in range(len(jointsValues)):
        qb = legId * angleBetweenLegs + math.pi / 2.0
        gravityRotationMatrices = getGravityRotationMatrices(jointsValues[legId], qb)
        forceRotationMatrices = getForceRotationMatrices(jointsValues[legId])

        localGravityVectors = calculateGravityVectors(gravityRotationMatrices, spiderGravityVector)
        legTorques = calculateTorques(legId, forceRotationMatrices, localGravityVectors)
        torques[legId] = legTorques

    return torques 

@numba.jit(nopython = True, cache = True)
def calculateGravityVectors(gravityRotationMatrices, spiderGravityVector):
    """Calculate gravity vectors in segments' origins.

    Args:
        gravityRotationMatrices (list): 5x3x3 array of five 3x3 rotation matrices.
        spiderGravityVector (list): 1x3 gravity vector in spider's origin.

    Returns:
        numpy.ndarray: 3x3 array of three local gravity vectors, given in segments' origins.
    """
    localGravityVectors = np.zeros((3, 3), dtype = np.float32)
    for i in range(3):
        localGravityVectors[i] = np.dot(np.transpose(gravityRotationMatrices[i]), spiderGravityVector)
    return localGravityVectors

@numba.jit(nopython = True, cache = True)
def calculateTorques(legId, forceRotationMatrices, localGravityVectors, cogVectors = spider.VECTORS_TO_COG_SEGMENT, legsDimensions = spider.LEGS_DIMENSIONS, segmentsMasses = spider.SEGMENTS_MASSES):
    """Calculate torques in the motors, using Newton-Euler method.

    Args:
        forceRotationMatrices (list): 3x3x3 array of three rotation matrices.
        localGravityVectors (list): 3x3 array of three gravity vectors in segments' origins.

    Returns:
        numpy.ndarray: 1x3 array of torques in joints.
    """
    torquesVectorsInLeg = np.zeros((3, 3), dtype = np.float32)
    torquesValuesInLeg = np.zeros(3, dtype = np.float32)
    forces = np.zeros((3, 3), dtype = np.float32)

    for i in range(3):
        lc = np.array([1, 0, 0], dtype = np.float32) * cogVectors[legId][2 - i]
        l = np.array([1, 0, 0], dtype = np.float32) * legsDimensions[2 - i]

        if i != 0:
            fgSegment = segmentsMasses[legId][2 - i] * localGravityVectors[2 - i]
            forces[i] = np.dot(forceRotationMatrices[i - 1], forces[i - 1]) - fgSegment
            torquesVectorsInLeg[i] = np.dot(forceRotationMatrices[i - 1], torquesVectorsInLeg[i - 1]) + np.cross(fgSegment, lc) - np.cross(np.dot(forceRotationMatrices[i - 1], forces[i - 1]), l)
            torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
            continue

        forces[i] = -segmentsMasses[legId][2 - i] * localGravityVectors[2 - i]
        torquesVectorsInLeg[i] = (-1) * np.cross(forces[i], lc)
        torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
    
    
    return np.flip(torquesValuesInLeg)

@numba.jit(nopython = True, cache = True)
def createDiagTransposeJHash(jointsValues):
    """Create diagonal matrix from transposed damped pseudo-inverses of jacobian matrices.

    Args:
        jointsValues (list): nx3 array of angles in joints in radians, where n is number of used legs.
 
    Returns:
        numpy.ndarray: (3*n)x(3*n) diagonal matrix of transposed damped-pseudo-inverses of jacobian matrices.
    """
    diagJhashTrans = np.zeros((3 * len(jointsValues), 3 * len(jointsValues)))
    for leg in range(len(jointsValues)):
        JhashTrans = np.transpose(mathTools.dampedPseudoInverse(kin.spiderBaseToLegTipJacobi(leg, jointsValues[leg])))
        diagJhashTrans[3 * leg : 3 * leg + len(JhashTrans), 3 * leg : 3 * leg + len(JhashTrans)] = JhashTrans
    
    return diagJhashTrans
#endregion

#region private methods
def _getSpiderExternalForces(measuredTorques, jointsValues):
    """Calculate external forces and torques from all internal torques and joints values.

    Args:
        measuredTorques (list): 5x3 array of measured torques in motors.
        jointsValues (list): 5x3 array of angles in joints in radians.

    Returns:
        numpy.ndarray: 6x1 array of external forces and torques.
    """
    measuredTorquesArray = measuredTorques.flatten()
    Jf = _createJfMatrix()
    xA = kin.allLegsPositions(jointsValues, config.SPIDER_ORIGIN)
    Jm = _createJmMatrix(xA)
    Jfm = np.r_[Jf, Jm]
    JhashTransDiag = createDiagTransposeJHash(jointsValues)
    Jx = np.dot(Jfm, JhashTransDiag)
    W = np.dot(Jx, measuredTorquesArray)

    return W, Jx, JhashTransDiag

def _createJmMatrix(xA):
    """Create Jm matrix from antisimetric matrices of position vectors

    Args:
        xA (list): nx3 array of legs' positions.

    Returns:
        numpy.ndarray: 3x(3xn) Jm matrix, where n is number of used legs.
    """
    xA = np.array(xA, dtype = np.float32)
    for i in range(len(xA)):
        x, y, z = xA[i]
        antisimMatrix = np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ], dtype = np.float32)
        if i == 0:
            Jm = antisimMatrix
            continue
        Jm = np.c_[Jm, antisimMatrix]

    return Jm

def _createJfMatrix():
    for i in range(spider.NUMBER_OF_LEGS):
        if i == 0:
            Jf = np.eye(3, dtype = np.float32)
            continue
        Jf = np.c_[Jf, np.eye(3, dtype = np.float32)]
    return Jf
#endregion