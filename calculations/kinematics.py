"""Module for all kinematics calculations.
"""
import math
import numpy as np
import itertools as itt

from environment import spider
from calculations import transformations as tf
from calculations import mathtools as mathTools


def legForwardKinematics(jointsValues):
    """Calculate forward kinematics for spiders leg, using transformation matrices.  

    :param jointValues: Joint values in radians.
    :return: Transformation matrix from base to end effector.

    Args:
        legId (int):Leg id.
        jointsValues (list): Joints values in radians.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from base to end effector.
    """
    q1, q2, q3 = jointsValues
    L1 = spider.LEGS_DIMENSIONS[0]
    L2 = spider.LEGS_DIMENSIONS[1]
    L3 = spider.LEGS_DIMENSIONS[2]

    return np.array([
        [math.cos(q1) * math.cos(q2 + q3), -math.cos(q1) * math.sin(q2 + q3), math.sin(q1), math.cos(q1) * (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3))],
        [math.cos(q2 + q3) * math.sin(q1), -math.sin(q1) * math.sin(q2 + q3), -math.cos(q1), (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1)],
        [math.sin(q2 + q3), math.cos(q2 + q3), 0, L2 * math.sin(q2) + L3 * math.sin(q2 + q3)],
        [0, 0, 0, 1]
    ], dtype = np.float32)

def legBaseToThirdJointForwardKinematics(jointValues):
    """Calculate forward kinematics from leg base to third joint.

    Args:
        legId (int): Leg id.
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
        [math.sin(q2), math.cos(q2), 0, L2 * math.sin(q2)],
        [0, 0, 0, 1]
    ], dtype = np.float32)

def legInverseKinematics(endEffectorPosition):
    """Calculate inverse kinematics for leg, using geometry.

    Args:
        legId (int): Leg id.
        endEffectorPosition (list): Desired position of end effector in leg-base origin.

    Returns:
        tuple: Angles in radians in first, second and third joint.
    """
    endEffectorPosition = np.array(endEffectorPosition)

    L1 = spider.LEGS_DIMENSIONS[0]
    L2 = spider.LEGS_DIMENSIONS[1]
    L3 = spider.LEGS_DIMENSIONS[2]

    # Angle in first joint.
    q1 = math.atan2(endEffectorPosition[1], endEffectorPosition[0])
    # Allow passing from upper workspace of the leg (Xe > 0), to the lower (Xe < 0).
    if endEffectorPosition[0] < 0.0:
        q1 = math.atan2(-endEffectorPosition[1], -endEffectorPosition[0])

    # Position of second joint in leg-base origin.
    secondJointPosition = np.array([
        L1 * math.cos(q1),
        L1 * math.sin(q1),
        0])

    # Vector from second joint to end effector.
    secondJointToEndVector = np.array(endEffectorPosition - secondJointPosition)

    # Distance between second joint and end effector.
    r = np.linalg.norm(secondJointToEndVector)
    # Angle in third joint, note Pi - acos(x) = acos(-x).
    q3 =  -math.acos(np.round((r**2 - L2**2 - L3**2) / (2 * L2 * L3), 4))
    # Angle in second joint.
    alpha = abs(math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3)))
    xy = np.linalg.norm(secondJointToEndVector[0:2]) if endEffectorPosition[0] >= secondJointPosition[0] else -np.linalg.norm(secondJointToEndVector[0:2])
    gamma = math.atan2(secondJointToEndVector[2], xy)
    q2 = alpha + gamma
    
    return q1, q2, q3

def platformInverseKinematics(legsIds, legsGlobalPositions, goalPose):
    """Calculate inverse kinematics for spider platform.

    Args:
        legsIds (list): Ids of used legs.
        legsGlobalPositions (list): Global positions of used legs.
        goalPose (list): Goal pose in global, given as 1x6 array of xyzrpy values.

    Raises:
        ValueError: If length of legsIds and legsGlobalPositions are not the same.

    Returns:
        numpy.ndarray: nx3 matrix with joints values for all used legs, where n is number of used legs.
    """
    if len(legsIds) != len(legsGlobalPositions):
        raise ValueError("Lengths of legsIds and legsGlobalPositions do not match.")

    # Get transformation matrix from spiders xyzrpy.
    globalTransformMatrix = tf.xyzRpyToMatrix(goalPose)

    # Array to store calculated joints values for all legs.
    joints = np.zeros([len(legsIds, spider.NUMBER_OF_MOTORS_IN_LEG)])
    for idx, leg in enumerate(legsIds):
        # Pose of leg anchor in global
        anchorInGlobal = np.dot(globalTransformMatrix, spider.T_ANCHORS[leg])
        # Position of leg anchor in global.
        anchorInGlobalPosition = anchorInGlobal[:,3][:3]

        # Vector from anchor to end of leg in global.
        anchorToPinGlobal = np.array(legsGlobalPositions[idx] - anchorInGlobalPosition)

        # Transform this vector in legs local origin - only rotate.
        rotationMatrix = anchorInGlobal[:3, :3]
        anchorToPinLocal = np.dot(np.linalg.inv(rotationMatrix), anchorToPinGlobal)

        # With inverse kinematics for single leg calculate joints values.
        q1, q2, q3 = legInverseKinematics(leg, anchorToPinLocal)
        joints[idx] = np.array([q1, q2, q3])

    return joints

def platformForwardKinematics(legsIds, legsGlobalPositions, legsLocalPoses):
    """Calculate forward kinematics of platform. Use only those legs, that are in contact with pins.

    Args:
        legsIds (list): Ids of used legs.
        legsGlobalPositions (list): Global positions of used legs.
        legsLocalPoses (list):4x4 transformation matrices, representing a legs poses in legs-local origins.

    Returns:
        list: 1x6 list with global xyzrpy values.
    """

    if len(legsIds) != 3 or len(legsGlobalPositions) != 3 or len(legsLocalPoses) != 3:
        print("Use exactly three legs for calculations of forward kinematics.")
        return False
    
    legsLocalPoses = np.array(legsLocalPoses)
    legsGlobalPositions = np.array(legsGlobalPositions)
    l1, l2, l3 = legsLocalPoses
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

    positions = np.zeros([len(legsLocalPoses, 3)])
    for idx, leg in enumerate(legsLocalPoses):
        positions[idx] = np.dot(Pglobal[:3,:3], leg[:,3][:3]) + np.dot(Pglobal[:3,:3], -legsGlobalPositions[idx])
    Pglobal[:,3][:3] = np.mean(positions, axis = 0)

    pose = np.linalg.inv(Pglobal)

    yaw = math.atan2(Pglobal[1, 0], Pglobal[0, 0])
    pitch = math.atan2(-Pglobal[2, 0], math.sqrt(math.pow(Pglobal[2, 1], 2) + math.pow(Pglobal[2, 2], 2)))
    roll = math.atan2(Pglobal[2, 1], Pglobal[2, 2])

    x, y, z = pose[:,3][:3]
    xyzrpy = [x, y, z, roll, pitch, yaw]
    
    return xyzrpy

def legJacobi(jointValues):
    """Calculate Jacobian matrix for given leg.

    Args:
        legId (int): Leg id.
        jointValues (list): Joints values in radians.

    Returns:
        numpy.ndarray: 3x3 Jacobian matrix.
    """
    q1, q2, q3 = jointValues
    L1 = spider.LEGS_DIMENSIONS[0]
    L2 = spider.LEGS_DIMENSIONS[1]
    L3 = spider.LEGS_DIMENSIONS[2]

    return np.array([
        [-(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1), -math.cos(q1) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.cos(q1) * math.sin(q2 + q3)],
        [math.cos(q1) * (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)), -math.sin(q1) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.sin(q1) * math.sin(q2 + q3)],
        [0, L2 * math.cos(q2) + L3 * math.cos(q2 + q3), L3 * math.cos(q2 + q3)]
        ], dtype = np.float32)

def spiderBaseToLegTipForwardKinematics(legId, jointsValues):
    """Calculate forward kinematics from spider base to leg-tip.

    Args:
        legId (int): Leg id.
        jointsValues (list): Joints values in radians.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from spider base to leg-tip.
    """
    qb = legId * spider.ANGLE_BETWEEN_LEGS + math.pi / 2
    q1, q2, q3 = jointsValues
    r = spider.BODY_RADIUS
    L1 = spider.LEGS_DIMENSIONS[0]
    L2 = spider.LEGS_DIMENSIONS[1]
    L3 = spider.LEGS_DIMENSIONS[2]

    return np.array([
        [math.cos(q2 + q3) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2 + q3), math.sin(q1 + qb), r * math.cos(qb) + (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.cos(q1 + qb)],
        [math.cos(q2 + q3) * math.sin(q1 + qb), -math.sin(q2 + q3) * math.sin(q1 + qb), -math.cos(q1 + qb), r * math.sin(qb) + (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1 + qb)],
        [math.sin(q2 + q3), math.cos(q2 + q3), 0, L2 * math.sin(q2) + L3 * math.sin(q2 + q3)],
        [0, 0, 0, 1]
    ], dtype = np.float32)

def spiderBaseToLegTipJacobi(legId, jointValues):
    """Calculate Jacobian matrix for spider's origin - leg-tip relation.

    Args:
        legId (int): Leg id.
        jointValues (list): Joints values in radians.

    Returns:
        numpy.ndarray: 3x3 Jacobian matrix.
    """
    qb = legId * spider.ANGLE_BETWEEN_LEGS + math.pi / 2
    q1, q2, q3 = jointValues
    L1 = spider.LEGS_DIMENSIONS[0]
    L2 = spider.LEGS_DIMENSIONS[1]
    L3 = spider.LEGS_DIMENSIONS[2]

    return np.array([
        [-(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1 + qb), -math.cos(q1 + qb) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.cos(q1 + qb) * math.sin(q2 + q3)],
        [(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.cos(q1 + qb), -(L2 * math.sin(q2) + L3 * math.sin(q2 + q3)) * math.sin(q1 + qb), -L3 * math.sin(q2 + q3) * math.sin(q1 + qb)],
        [0, L2 * math.cos(q2) + L3 * math.cos(q2 + q3), L3 * math.cos(q2 + q3)]
    ], dtype = np.float32)

def getSpiderToLegReferenceVelocities(legsIds, spiderVelocity):
    """Calculate needed legs velocities to match given spider velocity.

    Args:
        legsIds (list): Ids of used legs.
        spiderVelocity (list): 1x6 list with velocities in global xyzrpy directions.

    Returns:
        numpy.ndarray: nx3 array of reference legs velocities in leg-based origin, where n is number of used legs.
    """
    
    linearSpiderVelocity = spiderVelocity[:3]
    anguarSpiderVelocity = spiderVelocity[3:]

    # Rotate spiders reference velocity into anchors velocities which represent reference velocities for single leg.
    anchorsVelocities = [np.dot(np.linalg.inv(spider.T_ANCHORS[leg][:3,:3]), linearSpiderVelocity) for leg in legsIds]
    # Add angular velocities.
    for idx, leg in enumerate(legsIds):
        anchorPosition = spider.LEG_ANCHORS[leg]
        wx, wy, wz = anguarSpiderVelocity
        anchorsVelocities[idx] = np.array([
            anchorsVelocities[0],
            anchorsVelocities[1] + spider.BODY_RADIUS * wz,
            anchorsVelocities[2] - anchorPosition[0] * wy + anchorPosition[1] * wx
        ])
    refereneceLegVelocities = np.copy(anchorsVelocities) * (-1)

    return np.array(refereneceLegVelocities, dtype = np.float32)

def getLegsApproachPositionsInGlobal(legsIds, spiderPose, globalPinsPositions, offset = 0.03):
    """Calculate approach point in global origin, so that gripper would fit on pin.

    Args:
        legsIds (list): Ids of used legs.
        spiderPose (list): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.
        globalPinsPositions (list): nx3 array of used pins positions in global origin, where n should be same as number of used legs.
        offset (float, optional): Distance from pin to the approach point. Defaults to 0.03.

    Raises:
        ValueError: If lengths of legsIds and globalPinsPositions parameters are not the same.

    Returns:
        numpy.ndarray: nx3 array of approach positions in global origin, where n is number of used legs.
    """
    if len(legsIds) != len(globalPinsPositions):
        raise ValueError("Invalid values of legsIds or pinsPositions parameters.")
    
    approachPointsInGlobal = np.empty([len(legsIds), 3])
    for idx, leg in enumerate(legsIds):
        jointsValues = legInverseKinematics(leg, tf.getLegInLocal(leg, globalPinsPositions[idx], spiderPose))
        T_GA = np.dot(tf.xyzRpyToMatrix(spiderPose), spider.T_ANCHORS[leg])
        thirdJointLocalPosition = legBaseToThirdJointForwardKinematics(leg, jointsValues)[:,3][:3]
        thirdJointGlobalPosition = np.dot(T_GA, np.append(thirdJointLocalPosition, 1))[:3]

        pinToThirdJoint = thirdJointGlobalPosition - globalPinsPositions[idx]
        pinToThirdJoint = (pinToThirdJoint / np.linalg.norm(pinToThirdJoint)) * offset
        approachPointsInGlobal[idx] = globalPinsPositions[idx] + pinToThirdJoint

    return approachPointsInGlobal

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
    for legsSubset in itt.combinations(legsIds, 3):
        legsSubset = np.array(legsSubset)
        subsetIdxs = [legsIds.index(leg) for leg in legsSubset]
        jointsValues = qA[legsSubset]
        legsPoses = np.zeros([3, 4, 4])
        for idx, leg in enumerate(legsSubset):
            legsPoses[idx] = spiderBaseToLegTipForwardKinematics(leg, jointsValues[idx])
        poses.append(platformForwardKinematics(legsSubset, legsGlobalPositions[(subsetIdxs)], legsPoses))
    pose = np.mean(np.array(poses), axis = 0)

    return pose