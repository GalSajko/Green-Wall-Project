"""Module for calculating transformations between different origins.
"""
import numpy as np
import math

from environment import spider

def xyzRpyToMatrix(xyzrpy):
    """Calculate global transformation matrix for global origin - spider relation.

    Args:
        xyzrpy (list): Global spider's pose to be transformed into matrix. Could be given as 1x4 array, representing xyzy values or 
        1x6 array, representing xyzrpy values.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from global origin to spider.
    """
    if len(xyzrpy) == 4:
        xyzrpy = [xyzrpy[0], xyzrpy[1], xyzrpy[2], 0, 0, xyzrpy[3]]
    position = xyzrpy[0:3]
    rpy = xyzrpy[3:]

    roll = np.array([
        [math.cos(rpy[1]), 0, math.sin(rpy[1])],
        [0, 1, 0],
        [-math.sin(rpy[1]), 0, math.cos(rpy[1])]
    ], dtype = np.float32)
    pitch = np.array([
        [1, 0, 0],
        [0, math.cos(rpy[0]), -math.sin(rpy[0])],
        [0, math.sin(rpy[0]), math.cos(rpy[0])]
    ], dtype = np.float32)
    yaw = np.array([
        [math.cos(rpy[2]), -math.sin(rpy[2]), 0],
        [math.sin(rpy[2]), math.cos(rpy[2]), 0],
        [0, 0, 1]
    ], dtype = np.float32)

    rotationMatrix = np.dot(pitch, np.dot(roll, yaw))

    transformMatrix = np.c_[rotationMatrix, position]
    addRow = np.array([0, 0, 0, 1], dtype = np.float32)
    transformMatrix = np.r_[transformMatrix, [addRow]]
    
    return transformMatrix

def getLegInLocal(legId, globalLegPosition, spiderPose):
    """Calculate local leg's position from given global position.

    Args:
        legId (int): Leg id.
        globalLegPosition (list): Global position of leg.
        spiderPose (list): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.

    Returns:
        numpy.ndarray: 1x3 array of with x, y and z leg's positions in leg-local origin.
    """
    T_GS = xyzRpyToMatrix(spiderPose)
    T_GA = np.dot(T_GS, spider.T_ANCHORS[legId])
    globalLegPosition = np.append(globalLegPosition, 1)

    return np.dot(np.linalg.inv(T_GA), globalLegPosition)[:3]

def getGlobalDirectionInLocal(legId, spiderPose, globalDirection):
    T_GS = xyzRpyToMatrix(spiderPose)
    T_GA = np.dot(T_GS, spider.T_ANCHORS[legId])[:3,:3]
    localDirection = np.dot(np.linalg.inv(T_GA), globalDirection)

    return localDirection

def getLegsInGlobal(legsIds, localLegsPositions, spiderPose):
    """Calculate global positions of legs from given local positions.

    Args:
        legsIds (list): Legs ids.
        localLegsPositions (list): nx3 array of legs positions in their local origins, where n should be same as length of legsIds list.
        spiderPose (list): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.

    Returns:
        numpy.ndarray: nx3 array of legs positions in global origin, where n is number of given legs.
    """ 
    legsGlobalPositions = np.empty([len(legsIds), 3], dtype = np.float32)
    T_GS = xyzRpyToMatrix(spiderPose)
    for idx, leg in enumerate(legsIds):
        anchorInGlobal = np.dot(T_GS, spider.T_ANCHORS[leg])
        legInGlobal = np.dot(anchorInGlobal, np.append(localLegsPositions[idx], 1))
        legsGlobalPositions[idx] = legInGlobal[:3]

    return legsGlobalPositions

def R_B1(qb, q1):
    """Rotation matrix from spider's to 1st segment's origin.

    Args:
        qb (float): Angle from spider's origin to leg-base origin, in radians. 
        q1 (float): Angle in first joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q1 + qb), -math.sin(q1 + qb), 0],
        [math.sin(q1 + qb), math.cos(q1 + qb), 0],
        [0, 0, 1]
    ])

def R_12(q2):
    """Rotation matrix from 1st to 2nd leg-segment.

    Args:
        q2 (float): Angle in second joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q2), -math.sin(q2), 0],
        [0, 0, -1],
        [math.sin(q2), math.cos(q2), 0]
    ])

def R_23(q3):
    """Rotation matrix from 2nd to 3rd leg-segment.

    Args:
        q3 (float): Angle in third joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q3), -math.sin(q3), 0],
        [math.sin(q3), math.cos(q3), 0],
        [0, 0, 1]
    ])

def R_B2(qb, q1, q2):
    """Rotation matrix from spider's to 2nd segment's origin.

    Args:
        qb (float): Angle from spider's origin to leg-base origin, in radians.
        q1 (float): Angle in first joint, in radians.
        q2 (float): Angle in second joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q2) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2), math.sin(q1 + qb)],
        [math.cos(q2) * math.sin(q1 + qb), -math.sin(q2) * math.sin(q1 + qb), -math.cos(q1 + qb)],
        [math.sin(q2), math.cos(q2), 0]
    ])

def R_B3(qb, q1, q2, q3):
    return np.array([
        [math.cos(q2 + q3) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2 + q3), math.sin(q1 + qb)],
        [math.cos(q2 + q3) * math.sin(q1 + qb), -math.sin(q2 + q3) * math.sin(q1 + qb), -math.cos(q1 + qb)],
        [math.sin(q2 + q3), math.cos(q2 + q3), 0],
    ])