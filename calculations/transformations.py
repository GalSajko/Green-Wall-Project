"""Module for calculating transformations between different origins.
"""
import numpy as np
import math
import numba

import config
from environment import spider

def xyzRpyToMatrix(xyzrpy, rotationOnly = False):
    """Calculate global transformation matrix for global origin - spider relation.

    Args:
        xyzrpy (list): Global spider's pose to be transformed into matrix. Could be given as 1x4 array, representing xyzy values or 
        1x6 array, representing xyzrpy values.
        rotationOnly (bool): If True, calculate 3x3 rotation matrix from given rpy angles, otherwise calculate 4x4 full transformation matrix. Defaults to False.

    Returns:
        numpy.ndarray: 4x4 transformation matrix from global origin to spider or 3x3 rotation matrix from given rpy angles.
    """
    if rotationOnly:
        if len(xyzrpy) == 3:
            rpy = xyzrpy
        else:
            raise ValueError(f"Length of xyzrpy parameter should be 3, but it is {len(xyzrpy)}.")
    else:
        if len(xyzrpy) == 4:
            xyzrpy = [xyzrpy[0], xyzrpy[1], xyzrpy[2], 0, 0, xyzrpy[3]]
        position = xyzrpy[0:3]
        rpy = [xyzrpy[4], xyzrpy[3], xyzrpy[5]]

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
    rotationMatrix = np.dot(roll, np.dot(pitch, yaw))

    if not rotationOnly:
        transformMatrix = np.c_[rotationMatrix, position]
        transformMatrix = np.r_[transformMatrix, [np.array([0, 0, 0, 1], dtype = np.float32)]]
        
        return transformMatrix
    
    return rotationMatrix

def getPinToPinVectorInLocal(legId, rpy, currentPinPosition, goalPinPosition):
    """Calculate pin-to-pin vector in leg's local origin.

    Args:
        legId (int): Leg id.
        rpy (list): 1x3 array of roll, pitch and yaw values of spider's orientation.
        currentPinPosition (list): 1x3 array of current pin's position in global origin
        goalPinPosition (list): 1x3 array of goal pin's position in global origin.

    Returns:
        tuple: 1x3 pin-to-pin vector in leg's local origin and 3x3 orientation matrix of leg's anchor in global origin.
    """
    spiderRotationInGlobal = xyzRpyToMatrix(rpy, True)
    legOriginOrientationInGlobal = np.linalg.inv(np.dot(spiderRotationInGlobal, spider.T_ANCHORS[legId][:3, :3]))
    pinToPinGlobal = goalPinPosition - currentPinPosition
    pinToPinLocal = np.dot(legOriginOrientationInGlobal, pinToPinGlobal)

    return pinToPinLocal, legOriginOrientationInGlobal

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

def getLegsInGlobal(legsIds, localLegsPositions, spiderPose, origin):
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
        if origin == config.LEG_ORIGIN:
            anchorInGlobal = np.dot(T_GS, spider.T_ANCHORS[leg])
            legInGlobal = np.dot(anchorInGlobal, np.append(localLegsPositions[idx], 1))
        elif origin == config.SPIDER_ORIGIN:
            legInGlobal = np.dot(T_GS, np.append(localLegsPositions[idx], 1))
        legsGlobalPositions[idx] = legInGlobal[:3]

    return legsGlobalPositions

def convertIntoLocalGoalPosition(legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose):
    """Transform given leg's goal position into local origin.

    Args:
        legId (int): Leg id.
        legCurrentPosition (list): 1x3 array of current leg's position, given in local origin
        goalPositionOrOffset (list): 1x3 array of leg's goal position given as absolute position or offset. 
        origin (str): Origin that goal position or offset is given in.
        isOffset (bool): If True, goal position is given as an offset, otherwise as an absolute position.
        spiderPose (list): Spider's pose given in global origin.

    Returns:
        numpy.ndarray: 1x3 array of leg's goal position, given in local origin.
    """
    if origin == config.LEG_ORIGIN:
        localGoalPosition = np.copy(goalPositionOrOffset)
        if isOffset:
            localGoalPosition += legCurrentPosition
        return localGoalPosition
    if not isOffset:
        return getLegInLocal(legId, goalPositionOrOffset, spiderPose)
    return np.array(legCurrentPosition + getGlobalDirectionInLocal(legId, spiderPose, goalPositionOrOffset), dtype = np.float32)

def getWateringLegAndPose(plantPosition, spiderStartPose):
    """Calculate spider's pose for watering the plant and leg used for watering.

    Args:
        plantPosition (list): 1x3 array of plant's position in global origin.
        spiderStartPose (list): Spider's current pose in global origin.

    Returns:
        tuple: Leg id and spider's pose used for watering the plant.
    """
    if plantPosition[0] < spiderStartPose[0]:
        wateringLeg = spider.WATERING_LEGS_IDS[0]
        wateringPose = np.array([
            plantPosition[0] + spider.WATERING_XY_OFFSET_ABS[0],
            plantPosition[1] - spider.WATERING_XY_OFFSET_ABS[1],
            0.3
        ])
    else:
        wateringLeg = spider.WATERING_LEGS_IDS[1]
        wateringPose = np.array([
            plantPosition[0] - spider.WATERING_XY_OFFSET_ABS[0],
            plantPosition[1] - spider.WATERING_XY_OFFSET_ABS[1],
            0.3
        ])
    
    return wateringLeg, wateringPose


@numba.jit(nopython = True, cache = True)
def R_B1(qb, q1):
    """Rotation matrix from spider's to 1st segment's origin.

    Args:
        qb (float): Angle from spider's origin to leg-base origin, in radians. 
        q1 (float): Angle in first joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q1 + qb), -math.sin(q1 + qb), 0.0],
        [math.sin(q1 + qb), math.cos(q1 + qb), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = True)
def R_12(q2):
    """Rotation matrix from 1st to 2nd leg-segment.

    Args:
        q2 (float): Angle in second joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q2), -math.sin(q2), 0.0],
        [0.0, 0.0, -1.0],
        [math.sin(q2), math.cos(q2), 0.0]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = True)
def R_23(q3):
    """Rotation matrix from 2nd to 3rd leg-segment.

    Args:
        q3 (float): Angle in third joint, in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(q3), -math.sin(q3), 0.0],
        [math.sin(q3), math.cos(q3), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = True)
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
        [math.sin(q2), math.cos(q2), 0.0]
    ], dtype = np.float32)

@numba.jit(nopython = True, cache = True)
def R_B3(qb, q1, q2, q3):
    return np.array([
        [math.cos(q2 + q3) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2 + q3), math.sin(q1 + qb)],
        [math.cos(q2 + q3) * math.sin(q1 + qb), -math.sin(q2 + q3) * math.sin(q1 + qb), -math.cos(q1 + qb)],
        [math.sin(q2 + q3), math.cos(q2 + q3), 0.0],
    ], dtype = np.float32)