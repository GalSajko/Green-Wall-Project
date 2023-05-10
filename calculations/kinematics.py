"""Module for all kinematics calculations.
"""
import math
import numpy as np
import itertools as itt
import numba

from config import SPIDER_ORIGIN, LEG_ORIGIN
from environment import spider
from calculations import transformations as tf
from calculations import mathtools as mathTools

#region forward kinematics
@numba.jit(nopython = True, cache = False)
def all_legs_positions(joints_values, fkType, legs = spider.LEGS_IDS):
    """Get positions of all legs in given origin.

    Args:
        joints_values (list): 5x3 array of all angles in joints in radians.
        fkType (char): Origin in which positions will be calculated - either spider's or leg's origin.

    Raises:
        ValueError: If given origin is not valid.

    Returns:
        numpy.ndarray: 5x3 array of (x, y, z) positions of all legs, given in selected origin.
    """
    if fkType not in (SPIDER_ORIGIN, LEG_ORIGIN):
        raise ValueError("Invalid value of FK type parameter.")

    x_a = np.zeros((len(legs), 3), dtype = np.float32)
    for leg in legs:
        if fkType == LEG_ORIGIN:
            x_a[leg] = leg_forward_kinematics(joints_values[leg])[:,3][:3]
            continue
        x_a[leg] = spider_base_to_leg_tip_forward_kinematics(leg, joints_values[leg])[:,3][:3]
    
    return x_a

@numba.jit(nopython = True, cache = False)
def leg_forward_kinematics(joints_values, legs_dimensions = spider.LEGS_DIMENSIONS):
    """Calculate forward kinematics for spiders leg, using transformation matrices.  

    Args:
        joints_values (list): Joints values in radians.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from base to end effector.
    """
    q1, q2, q3 = joints_values
    L1 = legs_dimensions[0]
    L2 = legs_dimensions[1]
    L3 = legs_dimensions[2]

    return np.array([
        [math.cos(q1) * math.cos(q2 + q3) * 1.0, -math.cos(q1) * math.sin(q2 + q3) * 1.0, math.sin(q1) * 1.0, math.cos(q1) * (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * 1.0],
        [math.cos(q2 + q3) * math.sin(q1) * 1.0, -math.sin(q1) * math.sin(q2 + q3) * 1.0, -math.cos(q1) * 1.0, (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1) * 1.0],
        [math.sin(q2 + q3) * 1.0, math.cos(q2 + q3) * 1.0, 0.0, L2 * math.sin(q2) + L3 * math.sin(q2 + q3) * 1.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32)

def legBaseToThirdJointForwardKinematics(jointValues):
    """Calculate forward kinematics from leg base to third joint.

    Args:
        leg_id (int): Leg id.
        jointValues (list): Joints values in radians.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from leg-base to third joint.
    """
    q1, q2, _ = jointValues
    L1 = spider.LEGS_DIMENSIONS[0]
    L2 = spider.LEGS_DIMENSIONS[1]

    return np.array([
        [math.cos(q1) * math.cos(q2), -math.cos(q1) * math.sin(q2), math.sin(q1), math.cos(q1) * (L1 + L2 * math.cos(q2))],
        [math.cos(q2) * math.sin(q1), -math.sin(q1) * math.sin(q2), -math.cos(q1), (L1 + L2 * math.cos(q2)) * math.sin(q1)],
        [math.sin(q2), math.cos(q2), 0.0, L2 * math.sin(q2)],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = False)
def spider_base_to_leg_tip_forward_kinematics(leg_id, joints_values, angle_between_legs = spider.ANGLE_BETWEEN_LEGS, radius = spider.BODY_RADIUS, legs_dimensions = spider.LEGS_DIMENSIONS):
    """Calculate forward kinematics from spider base to leg-tip.

    Args:
        leg_id (int): Leg id.
        joints_values (list): Joints values in radians.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from spider base to leg-tip.
    """
    q_b = leg_id * angle_between_legs + math.pi / 2
    q1, q2, q3 = joints_values
    r = radius
    L1 = legs_dimensions[0]
    L2 = legs_dimensions[1]
    L3 = legs_dimensions[2]

    return np.array([
        [math.cos(q2 + q3) * math.cos(q1 + q_b), -math.cos(q1 + q_b) * math.sin(q2 + q3), math.sin(q1 + q_b), r * math.cos(q_b) + (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.cos(q1 + q_b)],
        [math.cos(q2 + q3) * math.sin(q1 + q_b), -math.sin(q2 + q3) * math.sin(q1 + q_b), -math.cos(q1 + q_b), r * math.sin(q_b) + (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1 + q_b)],
        [math.sin(q2 + q3), math.cos(q2 + q3), 0.0, L2 * math.sin(q2) + L3 * math.sin(q2 + q3)],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32)

def platformForwardKinematics(legsIds, legsGlobalPositions, legsSpiderPoses):
    """Calculate forward kinematics of platform. Use only those legs, that are in contact with pins.

    Args:
        legsIds (list): Ids of used legs.
        legsGlobalPositions (list): Global positions of used legs.
        legsSpiderPoses (list):4x4 transformation matrices, representing a legs poses in spider's origin.

    Returns:
        list: 1x6 list with global xyzrpy values.
    """

    if len(legsIds) != 3 or len(legsGlobalPositions) != 3 or len(legsSpiderPoses) != 3:
        print("Use exactly three legs for calculations of forward kinematics.")
        return False
    
    legsSpiderPoses = np.array(legsSpiderPoses)
    legsGlobalPositions = np.array(legsGlobalPositions)
    l1, l2, l3 = legsSpiderPoses
    p1, p2, _ = legsGlobalPositions

    # Compute coordinate system of a wall-plane (in spider's origin)
    l12 = l2[:,3][:3] - l1[:,3][:3]
    l13 = l3[:,3][:3] - l1[:,3][:3]
    l23 = l3[:,3][:3] - l2[:,3][:3]
    n = [
        np.cross(l12, l13) if np.cross(l12, l13)[2] >= 0.0 else np.cross(l13, l12),
        np.cross(l12, l23) if np.cross(l12, l23)[2] >= 0.0 else np.cross(l23, l12),
        np.cross(-l13, -l23) if np.cross(-l13, -l23)[2] >= 0.0 else np.cross(-l23, -l13)
    ]
    n = np.mean(n, axis = 0)
    ez = n / np.linalg.norm(n)
    ex = l12 / np.linalg.norm(l12)
    ey = np.cross(ez, ex)
    P = np.array([ex, ey, ez])

    p12 = p2 - p1
    phi = mathTools.calculateSignedAngleBetweenTwoVectors(p12[:2], np.array([1, 0]))

    # Rotate P for angle phi aroud z axis to align it with global origin.
    rot = np.array([
        [math.cos(phi), -math.sin(phi), 0],
        [math.sin(phi), math.cos(phi), 0],
        [0, 0, 1]
    ])
    Pglobal = np.dot(np.linalg.inv(rot), P)
    Pglobal = np.c_[Pglobal, np.zeros(3)]
    Pglobal = np.r_[Pglobal, [[0, 0, 0, 1]]]

    positions = np.zeros([len(legsSpiderPoses), 3])
    for idx, leg in enumerate(legsSpiderPoses):
        positions[idx] = legsGlobalPositions[idx] + np.dot(Pglobal[:3, :3], -leg[:, 3][:3])
    Pglobal[:,3][:3] = np.mean(positions, axis = 0)

    yaw = math.atan2(Pglobal[1, 0], Pglobal[0, 0])
    # Note that roll and pitch are swapped because of spider's axis definition.
    roll = math.atan2(-Pglobal[2, 0], math.sqrt(math.pow(Pglobal[2, 1], 2) + math.pow(Pglobal[2, 2], 2)))
    pitch = math.atan2(Pglobal[2, 1], Pglobal[2, 2])

    x, y, z = Pglobal[:,3][:3]
    xyzrpy = [x, y, z, roll, pitch, yaw]
    
    return xyzrpy

def getSpiderPose(legsIds, legsGlobalPositions, qA):
    """Calculate spider's pose in global origin. If more than three legs are given, it calculates spider's pose from each
    combination of these three legs. Finally pose is determined as mean value of all calculations.

    Args:
        legsIds (list): Ids of legs to use for calculating spider's pose. Should not use less than three legs.
        legsGlobalPositions (list): nx3 array of x, y, z positions of used legs in global origin, where n should be the same as length of legsIds.

    Returns:
        list: 1x6 array of xyzrpy pose.
    """
    legsGlobalPositions = np.array(legsGlobalPositions)
    poses = []
    legsIds = list(legsIds)
    for legsSubset in itt.combinations(legsIds, 3):
        legsSubset = np.array(legsSubset)
        subsetIdxs = [legsIds.index(leg) for leg in legsSubset]
        # Skip calculations if all three selected legs are on the same line.
        if not (np.diff(legsGlobalPositions[subsetIdxs][:, 0]).any() and np.diff(legsGlobalPositions[subsetIdxs][:, 1]).any()):
            continue
        joints_values = qA[legsSubset]
        legsPoses = np.zeros([3, 4, 4])
        for idx, leg in enumerate(legsSubset):
            legsPoses[idx] = spider_base_to_leg_tip_forward_kinematics(leg, joints_values[idx])
        poses.append(platformForwardKinematics(legsSubset, legsGlobalPositions[(subsetIdxs)], legsPoses))
    pose = np.mean(np.array(poses), axis = 0)

    return pose

def getGoalPinInLocal(leg_id, attachedLegs, legsGlobalPositions, qA, rpy):
    """Calculate goal pin position in leg-local origin.

    Args:
        leg_id (int): Leg id.
        attachedLegs (list): List of attached legs.
        legsGlobalPositions (list): 5x3 array of global legs positions.
        qA (list): 5x3 array of joints values.
        rpy (list): 1x3 array of spider's rpy rotation values.

    Returns:
        numpy.ndarray: 1x3 array of goal pin's x, y, z position in leg-local origin.
    """
    pose = getSpiderPose(attachedLegs, legsGlobalPositions[attachedLegs], qA)
    pose = tf.xyzRpyToMatrix(pose)
    rotation = tf.xyzRpyToMatrix(rpy, True)
    pose[:3, :3] = rotation
    goalPinInSpider = np.dot(np.linalg.inv(pose), np.append(legsGlobalPositions[leg_id], 1))
    goalPinInLocal = np.dot(np.linalg.inv(spider.T_ANCHORS[leg_id]), goalPinInSpider)[:3]

    return goalPinInLocal
#endregion

#region inverse kinematics
@numba.jit(nopython = True, cache = False)
def legInverseKinematics(endEffectorPosition, legs_dimensions = spider.LEGS_DIMENSIONS):
    """Calculate inverse kinematics for leg, using geometry.

    Args:
        endEffectorPosition (list): Desired position of end effector in leg-base origin.

    Returns:
        tuple: Angles in radians in first, second and third joint.
    """
    L1, L2, L3 = legs_dimensions

    # Angle in first joint.
    q1 = math.atan2(endEffectorPosition[1], endEffectorPosition[0])

    # Position of second joint in leg-base origin.
    secondJointPosition = np.array([
        L1 * math.cos(q1),
        L1 * math.sin(q1),
        0], dtype = np.float32)

    # Vector from second joint to end effector.
    secondJointToEndVector = endEffectorPosition - secondJointPosition

    # Distance between second joint and end effector.
    r = np.linalg.norm(secondJointToEndVector)
    # Angle in third joint, note Pi - acos(x) = acos(-x).
    q3 =  math.acos(np.round((r**2 - L2**2 - L3**2) / (2 * L2 * L3), 4))

    # Angle in second joint.
    alpha = abs(math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3)))
    xy = np.linalg.norm(secondJointToEndVector[0:2])
    gamma = math.atan2(secondJointToEndVector[2], xy)
    q2 = alpha + gamma
    
    return q1, q2, q3
#endregion

#region jacobians bv  
@numba.jit(nopython = True, cache = False)
def legJacobi(joints_values, legs_dimensions = spider.LEGS_DIMENSIONS):
    """Calculate Jacobian matrix for given leg.

    Args:
        leg_id (int): Leg id.
        joints_values (list): Joints values in radians.

    Returns:
        numpy.ndarray: 3x3 Jacobian matrix.
    """
    q1, q2, q3 = joints_values
    L1 = legs_dimensions[0]
    L2 = legs_dimensions[1]
    L3 = legs_dimensions[2]
    
    return np.array([
        [-(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1) * 1.0, -math.cos(q1) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)) * 1.0, -L3 * math.cos(q1) * math.sin(q2 + q3) * 1.0],
        [math.cos(q1) * (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * 1.0, -math.sin(q1) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)) * 1.0, -L3 * math.sin(q1) * math.sin(q2 + q3) * 1.0],
        [0.0, L2 * math.cos(q2) + L3 * math.cos(q2 + q3) * 1.0, L3 * math.cos(q2 + q3) * 1.0]
        ], dtype = np.float32)

@numba.jit(nopython = True, cache = False)
def spider_base_to_leg_tip_jacobi(leg_id, jointValues, angle = spider.ANGLE_BETWEEN_LEGS, legs_dimensions = spider.LEGS_DIMENSIONS):
    """Calculate Jacobian matrix for spider's origin - leg-tip relation.

    Args:
        leg_id (int): Leg id.
        jointValues (list): Joints values in radians.

    Returns:
        numpy.ndarray: 3x3 Jacobian matrix.
    """
    q_b = leg_id * angle + math.pi / 2
    q1, q2, q3 = jointValues
    L1 = legs_dimensions[0]
    L2 = legs_dimensions[1]
    L3 = legs_dimensions[2]

    return np.array([
        [-(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1 + q_b), -math.cos(q1 + q_b) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.cos(q1 + q_b) * math.sin(q2 + q3)],
        [(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.cos(q1 + q_b), -(L2 * math.sin(q2) + L3 * math.sin(q2 + q3)) * math.sin(q1 + q_b), -L3 * math.sin(q2 + q3) * math.sin(q1 + q_b)],
        [0, L2 * math.cos(q2) + L3 * math.cos(q2 + q3), L3 * math.cos(q2 + q3)]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = False)
def getJointsVelocities(currentAngles, xCds, number_of_legs = spider.NUMBER_OF_LEGS, number_of_motors_in_leg = spider.NUMBER_OF_MOTORS_IN_LEG):
    """Calculated needed joints velocities from given end-effectors' velocites, for all legs.

    Args:
        currentAngles (list): 5x3 array of angles in joints.
        xCds (list): 5x3 array of x, y, z velocities of all legs.

    Returns:
        numpy.ndarray: 5x3 array of joints velocities.
    """
    qCds = np.zeros((number_of_legs, number_of_motors_in_leg), dtype = np.float32)

    for leg, xCd in enumerate(xCds):
        J_inv = np.ascontiguousarray(mathTools.damped_pseudo_inverse(legJacobi(currentAngles[leg])))
        xCd = np.ascontiguousarray(xCd)
        qCds[leg] = np.dot(J_inv, xCd)
    
    return qCds

# @numba.jit(nopython = True, cache = False)
def getXdXddFromOffsets(forceModeLegs, offsetsInSpiderOrigin, velocitiesInSpiderOrigin, spiderToLegTransforms = spider.T_ANCHORS):
    """Rotate position offets and velocities, calculated from force controller into leg-local origins.

    Args:
        forceModeLegs (list): List of legs' ids, that are used in force controll.
        offsetsInSpiderOrigin (list): Calculated offsets from force-position P controller.
        velocitiesInSpiderOrigin (list): Calculated velocities from force-position P controller.

    Returns:
        Tuple: Two nx3 numpy.ndarrays with (x, y, z) offsets and velocities for each leg used in force mode, where n is number of used legs.
    """
    offsetsInLegsOrigins = np.zeros((len(forceModeLegs), 3), dtype = np.float32)
    velocitiesInLegsOrigins = np.zeros((len(forceModeLegs), 3), dtype = np.float32)

    for i, leg in enumerate(forceModeLegs):
        # Rotate offset and velocity vector in spider's origin into leg-local origin.
        spiderToLegRotation = np.linalg.inv(spiderToLegTransforms[leg][:3, :3])
        offsetsInLegsOrigins[i] = np.dot(spiderToLegRotation, offsetsInSpiderOrigin[leg])
        velocitiesInLegsOrigins[i] = np.dot(spiderToLegRotation, velocitiesInSpiderOrigin[leg])

    return offsetsInLegsOrigins, velocitiesInLegsOrigins
#endregion