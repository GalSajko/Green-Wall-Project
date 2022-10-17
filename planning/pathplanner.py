"""Module for planning spider's path.
"""
import numpy as np
import math
import itertools

from ..environment import spider
from ..environment import wall
from ..calculations import mathtools as mt
from ..calculations import transformations as tf
from ..calculations import kinematics as kin
from ..calculations import dynamics as dyn


MAX_LIN_STEP = 0.05
MAX_ROT_STEP = 0.2
MAX_LIFT_STEP = 0.3

#region public methods
def calculateSpiderBodyPath(startPose, goalPose):
    """Calculate steps of spider's path, including rotation around z axis (orientation). Path's segments are as follow:
    - lift spider on the walking height (if necessary),
    - rotate spider towards the goal point,
    - walk towards the goal point,
    - when on goal point, rotate in goal orientation. 

    Args:
        startPose (list): 1x4 array of starting pose, given as x, y, z, rotZ values in global origin, where rotZ is toration around global z axis. 
        goalPose (_type_): 1x4 array of goal pose, given as x, y, z, rotZ values in global origin, where rotZ is toration around global z axis.

    Returns:
        numpy.ndarray: nx4 array of poses on each step of movenet, where n is number of steps.
    """
    path = [startPose]

    # If spider is lying on the pins first lift it up on the walking height.
    # if startPose[2] == spider.LYING_HEIGHT:
    #     startWalkingPose = np.copy(startPose)
    #     startWalkingPose[2] = spider.WALKING_HEIGHT
    #     path.append(startWalkingPose)

    # Rotate towards goal point.
    # refAngle = math.atan2(goalPose[0] - startPose[0], goalPose[1] - startPose[1]) * (-1)
    # angleError = geometryTools.wrapToPi(refAngle - startPose[3])
    # if angleError != 0.0:
    #     numberOfSteps = math.ceil(abs(angleError) / maxRotStep) + 1
    #     rotatedPose = np.copy(path[-1])
    #     rotatedPose[3] = refAngle 
    #     rotatePath = np.linspace(path[-1], rotatedPose, numberOfSteps)
    #     for pose in rotatePath:
    #         path.append(pose)

    # Move towards goal point.
    distToTravel = np.linalg.norm(np.array(goalPose[:2]) - np.array(startPose[:2]))
    if distToTravel != 0.0:
        numberOfSteps = math.ceil(distToTravel / MAX_LIN_STEP) + 1
        lastPose = path[-1]
        goalPoseWithStartOrientation = np.copy(goalPose)
        goalPoseWithStartOrientation[3] = lastPose[3]
        linPath = np.linspace(lastPose, goalPoseWithStartOrientation, numberOfSteps)
        for pose in linPath:
            path.append(pose)

    # Rotate towards goal orientation.
    # angleError = geometryTools.wrapToPi(path[-1][3] - goalPose[3])
    # if angleError != 0.0:
    #     numberOfSteps = math.ceil(abs(angleError) / maxRotStep) + 1
    #     rotatePath = np.linspace(path[-1], goalPose, numberOfSteps )
    #     for pose in rotatePath:
    #         path.append(pose)

    return np.array(path)

def calculateSpiderLegsPositionsXyzRpyFF(path):
    """Calculate legs positions in global origin (same as positions of pins in wall's origin), for each step of the spider's path. 
    Orientation around global z axis is included in path.

    Args:
        path (list): nx4 array of poses on each step of the path, where n is number of steps.

    Returns:
        list: nx5x3 array of global positions of each leg on each step of the path, where n is number of steps in the path.
    """
    pins = wall.createGrid(True)
    selectedPins = []
    for step, pose in enumerate(path):
        T_GS = tf.xyzRpyToMatrix(pose)
        anchors = [np.dot(T_GS, t) for t in spider.T_ANCHORS]
        selectedPinsOnEachStep = []
        for idx, anchor in enumerate(anchors):
            potentialPinsForSingleLeg = []
            anchorPosition = anchor[:,3][:3]
            for pin in pins:
                distanceToPin = np.linalg.norm(anchorPosition - pin)

                if spider.CONSTRAINS[0] < distanceToPin < spider.CONSTRAINS[1]:
                    rotatedIdealLegVector = np.dot(T_GS[:3,:3], np.append(spider.IDEAL_LEG_VECTORS[idx], 0))
                    angleBetweenIdealVectorAndPin = mt.calculateSignedAngleBetweenTwoVectors(
                        rotatedIdealLegVector[:2], 
                        np.array(np.array(pin) - np.array(anchorPosition))[:2])

                    if abs(angleBetweenIdealVectorAndPin) < spider.CONSTRAINS[2]:
                        if step not in (0, len(path) - 1):
                            previousPin = selectedPins[step - 1][idx]
                            distanceBetweenSelectedAndPreviousPin = np.linalg.norm(previousPin - pin)
                            isLegMoving = 0 if distanceBetweenSelectedAndPreviousPin == 0 else 1
                            criterionFunction = 0.4 * abs(angleBetweenIdealVectorAndPin) + 0.4 * isLegMoving + 0.2 * abs(distanceToPin - spider.CONSTRAINS[1])
                        else:
                            criterionFunction = abs(distanceToPin - spider.CONSTRAINS[1]) + abs(angleBetweenIdealVectorAndPin)

                        potentialPinsForSingleLeg.append([pin, criterionFunction])

            potentialPinsForSingleLeg = np.array(potentialPinsForSingleLeg, dtype = np.float32)
            potentialPinsForSingleLeg = potentialPinsForSingleLeg[potentialPinsForSingleLeg[:, 1].argsort()]
            selectedPinsOnEachStep.append(potentialPinsForSingleLeg[0][0])

        selectedPins.append(selectedPinsOnEachStep)

    return selectedPins

def calculateSelectedPins(path, returnPotentialPins = False):
    """Wrapper function for calculating selected pins from spider's path, using force-manipulability ellipsoids.

    Args:
        path (list): nx6 array of spider's poses for each step of the path.

    Returns:
        numpy.ndarray: nx5x3 array of positions of selected pins for each step of the path, where n is number of steps of the path.
    """
    potentialPins = _calculatePotentialPins(path)
    validCombinations = _calculateValidCombinationOfPotentialPins(potentialPins)
    selectedPins = _calculateIdealPinsFromValidCombinations(validCombinations, path)

    if not returnPotentialPins:
        return selectedPins
    return potentialPins, selectedPins
    
def calculateWalkingMovesFF(globalStartPose, globalGoalPose):
    """(Feed-forward) calculation of spider's poses and its legs positions during walking. Spider's body is moving 
    continuously untily one of the leg has to change its position.

    Args:
        globalStartPose (list): 1x4 array of spider's start pose in global origin, given as x, y, z and rotZ, where rotZ is rotation around global z axis.
        globalGoalPose (list): 1x4 array of spider's goal pose in global origin, given as x, y, z and rotZ, where rotZ is rotation around global z axis.

    Returns:
        tuple: Two numpy.ndarrays, first of shape nx4 representing all of the spider's body poses during the walking, second of shape nx5x3 representing all of 
        legs positions during walking, where n is number of walking steps.
    """
    path = calculateSpiderBodyPath(globalStartPose, globalGoalPose)
    selectedPins = calculateSpiderLegsPositionsXyzRpyFF(path)
    platformPoses = [np.array(globalStartPose)]
    startingPins = selectedPins[0]
    selectedDiffPins = [np.array(startingPins)]

    for idx, pins in enumerate(selectedPins):
        if idx == 0:
            continue
        if (np.array(pins) - np.array(selectedPins[idx - 1])).any():
            selectedDiffPins.append(np.array(pins))
            platformPoses.append(path[idx])

    return np.array(platformPoses), np.array(selectedDiffPins)
#endregion

#region private methods
def _calculatePotentialPins(path):
    """Calculate all potential pins for each leg on each step of the path.

    Args:
        path (list): nx6 array of spider's poses on each step of the path.

    Returns:
        list: nx5xm array of all potential pins for each leg on each step of the path, where n is number of steps on the path and m is a variable, representing
        number of potential pins for each leg on single step and cannot be determined in advance (it can also be different for each leg).
    """
    pins = wall.createGrid(True)
    potentialPins = []
    for pose in path:
        T_GS = tf.xyzRpyToMatrix(pose)
        anchorsPoses = [np.dot(T_GS, t) for t in spider.T_ANCHORS]
        potentialPinsOnStep = []
        for idx, anchorPose in enumerate(anchorsPoses):
            potentialPinsForLeg = []
            anchorPosition = anchorPose[:,3][:3]
            for pin in pins:
                distanceToPin = np.linalg.norm(anchorPosition - pin)
                if spider.CONSTRAINS[0] < distanceToPin < spider.CONSTRAINS[1]:
                    rotatedIdealLegVector = np.dot(T_GS[:3,:3], np.append(spider.IDEAL_LEG_VECTORS[idx], 0))
                    anchorToPinGlobal = np.array(np.array(pin) - np.array(anchorPosition))
                    angleBetweenIdealVectorAndPin = mt.calculateSignedAngleBetweenTwoVectors(rotatedIdealLegVector[:2], anchorToPinGlobal[:2])
                    if abs(angleBetweenIdealVectorAndPin) < spider.CONSTRAINS[2]:
                        potentialPinsForLeg.append(pin.tolist())

            if len(potentialPinsForLeg) == 0:
                print(f"Cannot find any potential pins for leg {idx} on spider's pose {pose}")
                return False
            potentialPinsOnStep.append(potentialPinsForLeg)
        potentialPins.append(potentialPinsOnStep)
    return potentialPins

def _calculateValidCombinationOfPotentialPins(potentialPins):
    """Calculate all valid combinations of potential pins for each step of the path. Combination is valid, if two (or more) legs do not
    share same pins.

    Args:
        potentialPins (list): nx5xm array of all potential pins for each leg on each step of the path, where n is number of steps on the path and m is a variable, 
        representing number of potential pins for each leg on single step and cannot be determined in advance.

    Returns:
        list: nxmx5 array of all possible valid combinations of pins for each step of the path, where n is number of steps on the path.
    """
    combinations = []
    for potentialPinsOnStep in potentialPins:
        combinationsOnStep = list(itertools.product(*potentialPinsOnStep))
        approvedCombinationOnStep = [combination for combination in combinationsOnStep if len(set(tuple(c) for c in combination)) == spider.NUMBER_OF_LEGS]
        combinations.append(approvedCombinationOnStep)

    return combinations

def _calculateIdealPinsFromValidCombinations(pinsCombinations, path):
    """Calculate selected pins for each step of the path, from given valid combinations of possible pins.

    Args:
        pinsCombinations (list): Array of possible pins combinations for each step of the path.
        path (list): nx6 array of spider's poses for each step of the path.

    Returns:
        numpy.ndarray: nx5x3 array of positions of selected pins for each step of the path, where n is number of steps of the path.
    """
    selectedPins = np.zeros([len(path), spider.NUMBER_OF_LEGS, 3])
    for step, pose in enumerate(path):
        T_GS = tf.xyzRpyToMatrix(pose)
        anchorsPoses = [np.dot(T_GS, t) for t in spider.T_ANCHORS]
        gravityVectorInSpider = np.dot(T_GS[:3,:3], np.array([0, -1, 0]))
        zVectorInSpider = np.dot(T_GS[:3,:3], np.array([0, 0, 1]))
        rgValuesSumArray = np.zeros(len(pinsCombinations[step]))
        for combIdx, pins in enumerate(pinsCombinations[step]):
            rgValuesSum = 0
            for legId, pin in enumerate(pins):
                anchorToPinGlobal = np.array(np.array(pin) - np.array(anchorsPoses[legId][:,3][:3]))
                anchorToPinLegLocal = np.dot(np.linalg.inv(anchorsPoses[legId][:3,:3]), anchorToPinGlobal)
                jointsValues = kin.legInverseKinematics(legId, anchorToPinLegLocal)
                rgValue = dyn.getForceEllipsoidLengthInGivenDirection(legId, jointsValues, [gravityVectorInSpider, zVectorInSpider])
                rgValuesSum += rgValue
            rgValuesSumArray[combIdx] = rgValuesSum

        selectedPins[step] = pinsCombinations[step][np.argmax(rgValuesSumArray)]
    
    return selectedPins
#endregion