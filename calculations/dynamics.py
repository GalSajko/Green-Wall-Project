"""Module for all dynamics calculations.
"""
import numpy as np
import math

import environment.spider as spider
import mathtools as mathTools
import transformations as tf
import kinematics as kin

A_TORQUE_POLYNOM = 0.0
B_TORQUE_POLYNOM = 2.9326
C_TORQUE_POLYNOM = -0.1779

#region public methods
def getTorquesAndForcesOnLegsTips(jointsValues, currentsInMotors, spiderGravityVector):
    """Calculate forces, applied to tips of all legs, from currents in motors.
    Args:
        jointsValues (list): 5x3 array of angles in joints.
        currentsInMotors (list): 5x3 array of currents in motors.
        spiderGravityVector(list): 1x3 gravity vector in spider's origin.

    Returns:
        tuple: 5x3 array of forces, applied to leg tips in x, y, z direction of spider's origin and 5x3x3 array of damped pseudo inverses of jacobian matrices.
    """
    torques = _getTorquesInLegs(jointsValues, currentsInMotors, spiderGravityVector)

    forces = np.zeros((spider.NUMBER_OF_LEGS, 3))
    dampedPseudoInverses = np.zeros((spider.NUMBER_OF_LEGS, 3, 3))
    for legId, jointsInLeg in enumerate(jointsValues):
        J = kin.spiderBaseToLegTipJacobi(legId, jointsInLeg)
        Jhash = mathTools.dampedPseudoInverse(J)
        dampedPseudoInverses[legId] = Jhash
        forces[legId] = np.dot(np.transpose(Jhash), torques[legId])
    
    return forces, dampedPseudoInverses
    
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
    eGrav = np.dot(eigVects, direction[0])
    ez = np.dot(eigVects, direction[1])

    ellipsoidSizeInGlobalNegY = math.sqrt(np.prod(eigVals) / (math.pow(eGrav[0], 2) * eigVals[1] * eigVals[2] + math.pow(eGrav[1], 2) * eigVals[0] * eigVals[2] + math.pow(eGrav[2], 2) * eigVals[0] * eigVals[1]))
    ellipsoidSizeInGlobalZ = math.sqrt(np.prod(eigVals) / (math.pow(ez[0], 2) * eigVals[1] * eigVals[2] + math.pow(ez[1], 2) * eigVals[0] * eigVals[2] + math.pow(ez[2], 2) * eigVals[0] * eigVals[1]))

    ed = np.array([0, -ellipsoidSizeInGlobalNegY, ellipsoidSizeInGlobalZ])
    ed = ed / np.linalg.norm(ed)
    ed = np.dot(np.linalg.inv(eigVects), ed)
    t = math.sqrt(np.prod(eigVals) / (math.pow(ed[0], 2) * eigVals[1] * eigVals[2] + math.pow(ed[1], 2) * eigVals[0] * eigVals[2] + math.pow(ed[2], 2) * eigVals[0] * eigVals[1]))

    return t

def distributeForces(fA, legToOffload = None, xA = None):
    if legToOffload is not None and (not (0 <= legToOffload <= 4)):
        raise ValueError ("Wrong leg ID.")
    if (legToOffload is not None) and (xA is None):
        raise ValueError("To offload leg, other legs' positions should be given.")

    # Sum of forces in gravity and z directions.
    fgSum = np.sum(fA[:,1])
    fDist = np.copy(fA)

    # Distribute forces between all legs.
    if legToOffload is None:
        # TODO: use force-ellipsoids and spider's gravity vector here.
        fgDist = fgSum / spider.NUMBER_OF_LEGS
        fDist[:,1] = fgDist

        return fDist
    
    # Offload selected leg and distribute forces between remaining legs.
    # TODO: use force-ellipsoids 
    # usedLegs = np.delete(spider.LEGS_IDS, legToOffload).astype(np.int8)
    # usedLegsPositions = np.array(xA)[usedLegs]
    fgDist = fgSum / (spider.NUMBER_OF_LEGS - 1)
    # fzSum = np.sum(np.abs(fA[:,2]))
    # fxSum = np.sum(np.abs(fA[:,0]))
    # fzDist = self.__distributeZForces(fzSum, usedLegsPositions, usedLegs)
    # fxDist = self.__distributeXForces(fxSum, usedLegsPositions, usedLegs)

    # fDist[:,0] = fxDist
    fDist[:,1] = fgDist
    # fDist[:,2] = fzDist
    fDist[legToOffload] = np.zeros(3)

    return fDist   

def distributeForces(self, fA, xA, legToOffload = None):
    genForceVector = fA.flatten()
    Jf = np.array([np.eye(3)] * len(fA))
    Jm = _createJmMatrix(np.array(xA, dtype = np.float32))
    Jfm = np.r_[Jf, Jm]
#endregion

#region private methods
def _getTorquesInLegs(jointsValues, currentsInMotors, spiderGravityVector):
    """Calculate torques in leg-joints from measured currents. 

    Args:
        jointsValues (list): 5x3 array of angles in joints.
        currentsInMotors (list): 5x3 array of currents in motors.
        spiderGravityVector (list): 3x1 gravity vector in spider's origin.

    Returns:
        numpy.ndarray: 5x3 array of torques in motors.
    """
    currentsInMotors = np.array(currentsInMotors)
    currentsInMotors[:,1] *= -1

    gravityTorques = _getGravityCompensationTorques(jointsValues, spiderGravityVector)
    torques = (A_TORQUE_POLYNOM + B_TORQUE_POLYNOM * currentsInMotors + C_TORQUE_POLYNOM * currentsInMotors**2) - gravityTorques

    return np.array(torques, dtype = np.float32)

def _getGravityCompensationTorques(jointsValues, spiderGravityVector):
    """Calculate torques in joints (for all legs), required to compensate movement, caused only by gravity.

    Args:
        jointsValues (list): 5x3 array of angles in joints, in radians.
        spiderGravityVector (list): 1x3 gravity vector in spider's origin.

    Returns:
        numpy.ndarray: 5x3 array of required torques in joints.
    """
    torques = np.zeros([spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG])
    for legId, jointsInLeg in enumerate(jointsValues): 
        qb = legId * spider.ANGLE_BETWEEN_LEGS + math.pi / 2.0
        gravityRotationMatrices = _getGravityRotationMatrices(jointsInLeg, qb)
        forceRotationMatrices = _getForceRotationMatrices(jointsInLeg)

        # Calculate gravity vectors in segments' origins (from first to last segment).
        localGravityVectors = _calculateGravityVectors(gravityRotationMatrices, spiderGravityVector)
        legTorques = _calculateForcesAndTorques(spider.VECTORS_TO_COG_SEGMENT[legId], spider.LEGS_DIMENSIONS[legId], spider.SEGMENTS_MASSES[legId], forceRotationMatrices, localGravityVectors)
        torques[legId] = np.flip(legTorques) 

    return torques 

def _calculateGravityVectors(gravityRotationMatrices, spiderGravityVector):
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

def _calculateForcesAndTorques(vectorsToCog, legsDimensions, segmentMasses, forceRotationMatrices, localGravityVectors):
    """Calculate forces on leg-tip and torques in the motors, using Newton-Euler method.

    Args:
        vectorsToCog (list): 1x3 array of three local vectors to centers of gravity of segments.
        legsDimensions (list): 1x3 array of leg dimensions in segments' x direction.
        segmentMasses (list): 1x3 array of masses of three segments.
        forceRotationMatrices (list): 3x3x3 array of three rotation matrices.
        localGravityVectors (list): 3x3 array of three gravity vectors in segments' origins.

    Returns:
        numpy.ndarray: 1x3 array of torques in joints.
    """
    torquesVectorsInLeg = np.zeros((3, 3), dtype = np.float32)
    torquesValuesInLeg = np.zeros(3)
    forces = np.zeros((3, 3), dtype = np.float32)
    
    for i in range(3):
        lc = np.array([1, 0, 0], dtype = np.float32) * vectorsToCog[2 - i]
        l = np.array([1, 0, 0], dtype = np.float32) * legsDimensions[2 - i]

        if i != 0:
            fgSegment = segmentMasses[2 - i] * localGravityVectors[2 - i]
            forces[i] = np.dot(forceRotationMatrices[i - 1], forces[i - 1]) - fgSegment
            torquesVectorsInLeg[i] = np.dot(forceRotationMatrices[i - 1], torquesVectorsInLeg[i - 1]) + np.cross(fgSegment, lc) - np.cross(np.dot(forceRotationMatrices[i - 1], forces[i - 1]), l)
            torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
            continue

        forces[i] = -segmentMasses[2 - i] * localGravityVectors[2 - i]
        torquesVectorsInLeg[i] = (-1) * np.cross(forces[i], lc)
        torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
    
    return torquesValuesInLeg

def _createJmMatrix(xA):
    """Create Jm matrix from antisimetric matrices of position vectors

    Args:
        xA (list): nx3 array of legs' positions.

    Returns:
        numpy.ndarray: 3x(3xn) Jm matrix, where n is number of used legs.
    """
    xA = np.array(xA, dtype = np.float32)
    for i in range(len(xA)):
        posVector = xA[i]
        antisimMatrix = np.array([
            [0, -posVector[2], posVector[1]],
            [posVector[2], 0, -posVector[0]],
            [-posVector[1], posVector[0], 0]
        ], dtype = np.float32)
        if i == 0:
            Jm = antisimMatrix
            continue
        Jm = np.c_[Jm, antisimMatrix]

    return Jm

def _getGravityRotationMatrices(jointsValues, qb):
    q1, q2, q3 = jointsValues
    return np.array([
        tf.R_B1(qb, q1),
        tf.R_B2(qb, q1, q2),
        tf.R_B3(qb, q1, q2, q3)], dtype = np.float32)

def _getForceRotationMatrices(jointsValues):
    return np.array([tf.R_23(jointsValues[2]), tf.R_12(jointsValues[1])], dtype = np.float32)
#endregion