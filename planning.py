""" Module for path planning ang calculating trajectories.
"""

import math
import numpy as np 
import inspect
import time
import itertools

import environment
import calculations
import config


class PathPlanner:
    """Class for calculating spiders path and its legs positions on each step of the path.
    """
    def __init__(self, maxLinStep, maxRotStep, gridPattern = 'squared'):
        self.spider = environment.Spider()
        self.wall = environment.Wall(gridPattern)
        self.geometryTools = calculations.GeometryTools()
        self.matrixCalculator = calculations.MatrixCalculator()
        self.kinematics = calculations.Kinematics()
        self.dynamics = calculations.Dynamics()

        self.maxLinStep = maxLinStep
        self.maxRotStep = maxRotStep
        self.maxLiftStep = 0.3

    def calculateSpiderBodyPath(self, startPose, goalPose):
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
        if startPose[2] == self.spider.LYING_HEIGHT:
            startWalkingPose = np.copy(startPose)
            startWalkingPose[2] = self.spider.WALKING_HEIGHT
            path.append(startWalkingPose)

        # Rotate towards goal point.
        # refAngle = math.atan2(goalPose[0] - startPose[0], goalPose[1] - startPose[1]) * (-1)
        # angleError = self.geometryTools.wrapToPi(refAngle - startPose[3])
        # if angleError != 0.0:
        #     numberOfSteps = math.ceil(abs(angleError) / self.maxRotStep) + 1
        #     rotatedPose = np.copy(path[-1])
        #     rotatedPose[3] = refAngle 
        #     rotatePath = np.linspace(path[-1], rotatedPose, numberOfSteps)
        #     for pose in rotatePath:
        #         path.append(pose)

        # Move towards goal point.
        distToTravel = np.linalg.norm(np.array(goalPose[:2]) - np.array(startPose[:2]))
        if distToTravel != 0.0:
            numberOfSteps = math.ceil(distToTravel / self.maxLinStep) + 1
            lastPose = path[-1]
            goalPoseWithStartOrientation = np.copy(goalPose)
            goalPoseWithStartOrientation[3] = lastPose[3]
            linPath = np.linspace(lastPose, goalPoseWithStartOrientation, numberOfSteps)
            for pose in linPath:
                path.append(pose)

        # Rotate towards goal orientation.
        # angleError = self.geometryTools.wrapToPi(path[-1][3] - goalPose[3])
        # if angleError != 0.0:
        #     numberOfSteps = math.ceil(abs(angleError) / self.maxRotStep) + 1
        #     rotatePath = np.linspace(path[-1], goalPose, numberOfSteps )
        #     for pose in rotatePath:
        #         path.append(pose)

        return np.array(path)

    def calculateSpiderLegsPositionsXyzRpyFF(self, path):
        """Calculate legs positions in global origin (same as positions of pins in wall's origin), for each step of the spider's path. 
        Orientation around global z axis is included in path.

        Args:
            path (list): nx4 array of poses on each step of the path, where n is number of steps.

        Returns:
            list: nx5x3 array of global positions of each leg on each step of the path, where n is number of steps in the path.
        """
        pins = self.wall.createGrid(True)
        selectedPins = []
        for step, pose in enumerate(path):
            T_GS = self.matrixCalculator.xyzRpyToMatrix(pose)
            anchors = [np.dot(T_GS, t) for t in self.spider.T_ANCHORS]
            selectedPinsOnEachStep = []
            for idx, anchor in enumerate(anchors):
                potentialPinsForSingleLeg = []
                anchorPosition = anchor[:,3][:3]
                for pin in pins:
                    distanceToPin = np.linalg.norm(anchorPosition - pin)

                    if self.spider.CONSTRAINS[0] < distanceToPin < self.spider.CONSTRAINS[1]:
                        rotatedIdealLegVector = np.dot(T_GS[:3,:3], np.append(self.spider.IDEAL_LEG_VECTORS[idx], 0))
                        angleBetweenIdealVectorAndPin = self.geometryTools.calculateSignedAngleBetweenTwoVectors(
                            rotatedIdealLegVector[:2], 
                            np.array(np.array(pin) - np.array(anchorPosition))[:2])

                        if abs(angleBetweenIdealVectorAndPin) < self.spider.CONSTRAINS[2]:
                            if step not in (0, len(path) - 1):
                                previousPin = selectedPins[step - 1][idx]
                                distanceBetweenSelectedAndPreviousPin = np.linalg.norm(previousPin - pin)
                                isLegMoving = 0 if distanceBetweenSelectedAndPreviousPin == 0 else 1
                                criterionFunction = 0.4 * abs(angleBetweenIdealVectorAndPin) + 0.4 * isLegMoving + 0.2 * abs(distanceToPin - self.spider.CONSTRAINS[1])
                            else:
                                criterionFunction = abs(distanceToPin - self.spider.CONSTRAINS[1]) + abs(angleBetweenIdealVectorAndPin)

                            potentialPinsForSingleLeg.append([pin, criterionFunction])

                potentialPinsForSingleLeg = np.array(potentialPinsForSingleLeg, dtype = np.float32)
                potentialPinsForSingleLeg = potentialPinsForSingleLeg[potentialPinsForSingleLeg[:, 1].argsort()]
                selectedPinsOnEachStep.append(potentialPinsForSingleLeg[0][0])

            selectedPins.append(selectedPinsOnEachStep)

        return selectedPins
    
    def calculateIdealLegsPositionsFF(self, path):
        """Calculate ideal legs positions during spider's movement, using force manipulability ellipsoids. Position (or pin) is considered ideal, 
        if the size of the vector from center to the surface of ellipsoid in direction of gravity is biggest among all others potential positions/pins.

        Args:
            path (list): nx4 array of poses on each step of the path, where n is number of steps.

        Returns:
            tuple: Three numpy.ndarrays, first a nx5x3 array of positions of selected pins on each step of spider's path, and second a nx5 array of
            sizes of force ellipsoids in gravity directions for each selected pins, where n is number of steps of spider's path. Third is an array of 
            all potential pins for each leg on each step, size cannot be determined in advanced. 
        """
        pins = self.wall.createGrid(True)
        globalGravityVector = np.array([0, -1, 0])
        selectedPins = np.zeros([len(path), 5, 3])
        selectedPinsRgValues = np.zeros([len(path), 5])
        allPotentialPins = []
        for step, pose in enumerate(path):
            T_GS = self.matrixCalculator.xyzRpyToMatrix(pose)
            spiderGravityVector = np.dot(T_GS[:3,:3], globalGravityVector)
            anchorsPositions = [np.dot(T_GS, t)[:,3][:3] for t in self.spider.T_ANCHORS]
            selectedPinsOnEachStep = np.zeros([5, 3])
            selectedPinsRgValuesOnEachStep = np.zeros([5])
            allPotentialPinsStep =  []
            for idx, anchorPosition in enumerate(anchorsPositions):
                potentialPinsForSingleLeg = []
                allPotentialSingleLeg = []
                for pin in pins:
                    distanceToPin = np.linalg.norm(anchorPosition - pin)
                    if self.spider.CONSTRAINS[0] < distanceToPin < self.spider.CONSTRAINS[1]:
                        rotatedIdealLegVector = np.dot(T_GS[:3,:3], np.append(self.spider.IDEAL_LEG_VECTORS[idx], 0))
                        pinInLegLocal = np.array(np.array(pin) - np.array(anchorPosition))
                        angleBetweenIdealVectorAndPin = self.geometryTools.calculateSignedAngleBetweenTwoVectors(rotatedIdealLegVector[:2], pinInLegLocal[:2])
                        if abs(angleBetweenIdealVectorAndPin) < self.spider.CONSTRAINS[2]:
                            jointsValues = self.kinematics.legInverseKinematics(idx, pinInLegLocal)
                            rg = self.dynamics.getForceEllipsoidLengthInGivenDirection(idx, jointsValues, spiderGravityVector)
                            potentialPinsForSingleLeg.append([pin, rg])
                            allPotentialSingleLeg.append(pin)
                allPotentialPinsStep.append(np.array(allPotentialSingleLeg))
                potentialPinsForSingleLeg = np.array(potentialPinsForSingleLeg, dtype = object)
                potentialPinsForSingleLeg = potentialPinsForSingleLeg[potentialPinsForSingleLeg[:, 1].argsort()]
                selectedPinsOnEachStep[idx] = potentialPinsForSingleLeg[-1][0]
                selectedPinsRgValuesOnEachStep[idx] = potentialPinsForSingleLeg[-1][1]

            allPotentialPins.append(allPotentialPinsStep)
            selectedPins[step] = selectedPinsOnEachStep
            selectedPinsRgValues[step] = selectedPinsRgValuesOnEachStep

        return selectedPins, selectedPinsRgValues, np.array(allPotentialPins, dtype = object)
    
    def calculatePotentialPins(self, path):
        """Calculate all potential pins for each leg on each step of the path.

        Args:
            path (list): nx6 array of spider's poses on each step of the path.

        Returns:
            list: nx5xm array of all potential pins for each leg on each step of the path, where n is number of steps on the path and m is a variable, representing
            number of potential pins for each leg on single step and cannot be determined in advance (it can also be different for each leg).
        """
        pins = self.wall.createGrid(True)
        potentialPins = []
        for step, pose in enumerate(path):
            T_GS = self.matrixCalculator.xyzRpyToMatrix(pose)
            anchorsPoses = [np.dot(T_GS, t) for t in self.spider.T_ANCHORS]
            potentialPinsOnStep = []
            for idx, anchorPose in enumerate(anchorsPoses):
                potentialPinsForLeg = []
                anchorPosition = anchorPose[:,3][:3]
                for pin in pins:
                    distanceToPin = np.linalg.norm(anchorPosition - pin)
                    if self.spider.CONSTRAINS[0] < distanceToPin < self.spider.CONSTRAINS[1]:
                        rotatedIdealLegVector = np.dot(T_GS[:3,:3], np.append(self.spider.IDEAL_LEG_VECTORS[idx], 0))
                        anchorToPinGlobal = np.array(np.array(pin) - np.array(anchorPosition))
                        angleBetweenIdealVectorAndPin = self.geometryTools.calculateSignedAngleBetweenTwoVectors(rotatedIdealLegVector[:2], anchorToPinGlobal[:2])
                        if abs(angleBetweenIdealVectorAndPin) < self.spider.CONSTRAINS[2]:
                            potentialPinsForLeg.append(pin.tolist())

                if len(potentialPinsForLeg) == 0:
                    print(f"Cannot find any potential pins for leg {idx} on spider's pose {pose}")
                    return False
                potentialPinsOnStep.append(potentialPinsForLeg)
            potentialPins.append(potentialPinsOnStep)
        
        return potentialPins

    def calculateValidCombinationOfPotentialPins(self, potentialPins):
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
            approvedCombinationOnStep = [combination for combination in combinationsOnStep if len(set(tuple(c) for c in combination)) == self.spider.NUMBER_OF_LEGS]
            combinations.append(approvedCombinationOnStep)
        
        return combinations
    
    def calculateIdealPinsFromValidCombinations(self, pinsCombinations, path):
        """Calculate selected pins for each step of the path, from given valid combinations of possible pins.

        Args:
            pinsCombinations (list): Array of possible pins combinations for each step of the path.
            path (list): nx6 array of spider's poses for each step of the path.

        Returns:
            numpy.ndarray: nx5x3 array of positions of selected pins for each step of the path, where n is number of steps of the path.
        """
        selectedPins = np.zeros([len(path), self.spider.NUMBER_OF_LEGS, 3])
        for step, pose in enumerate(path):
            T_GS = self.matrixCalculator.xyzRpyToMatrix(pose)
            anchorsPoses = [np.dot(T_GS, t) for t in self.spider.T_ANCHORS]
            gravityVectorInSpider = np.dot(T_GS[:3,:3], self.spider.SPIDER_GRAVITY_VECTOR)
            zVectorInSpider = np.dot(T_GS[:3,:3], np.array([0, 0, 1]))
            rgValuesSumArray = np.zeros(len(pinsCombinations[step]))
            for combIdx, pins in enumerate(pinsCombinations[step]):
                rgValuesSum = 0
                for legIdx, pin in enumerate(pins):
                    anchorToPinGlobal = np.array(np.array(pin) - np.array(anchorsPoses[legIdx][:,3][:3]))
                    anchorToPinLegLocal = np.dot(np.linalg.inv(anchorsPoses[legIdx][:3,:3]), anchorToPinGlobal)
                    jointsValues = self.kinematics.legInverseKinematics(legIdx, anchorToPinLegLocal)
                    rgValuesSum += self.dynamics.getForceEllipsoidLengthInGivenDirection(legIdx, jointsValues, [gravityVectorInSpider, zVectorInSpider])
                rgValuesSumArray[combIdx] = rgValuesSum

            selectedPins[step] = pinsCombinations[step][np.argmax(rgValuesSumArray)]
        
        return selectedPins
    
    def calculateSelectedPins(self, path):
        """Wrapper function for calculating selected pins from spider's path, using force-manipulability ellipsoids.

        Args:
            path (list): nx6 array of spider's poses for each step of the path.

        Returns:
            numpy.ndarray: nx5x3 array of positions of selected pins for each step of the path, where n is number of steps of the path.
        """
        potentialPins = self.calculatePotentialPins(path)
        validCombinations = self.calculateValidCombinationOfPotentialPins(potentialPins)
        selectedPins = self.calculateIdealPinsFromValidCombinations(validCombinations, path)

        return selectedPins
        
    def calculateWalkingMovesFF(self, globalStartPose, globalGoalPose):
        """(Feed-forward) calculation of spider's poses and its legs positions during walking. Spider's body is moving 
        continuously untily one of the leg has to change its position.

        Args:
            globalStartPose (list): 1x4 array of spider's start pose in global origin, given as x, y, z and rotZ, where rotZ is rotation around global z axis.
            globalGoalPose (list): 1x4 array of spider's goal pose in global origin, given as x, y, z and rotZ, where rotZ is rotation around global z axis.

        Returns:
            tuple: Two numpy.ndarrays, first of shape nx4 representing all of the spider's body poses during the walking, second of shape nx5x3 representing all of 
            legs positions during walking, where n is number of walking steps.
        """
        path = self.calculateSpiderBodyPath(globalStartPose, globalGoalPose)
        selectedPins = self.calculateSpiderLegsPositionsXyzRpyFF(path)
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


class TrajectoryPlanner:
    """ Class for calculating different trajectories.
    """
    def minJerkTrajectory(self, startPose, goalPose, duration):
        """Calculate minimum jerk trajectory of positions and velocities between two points.

        Args:
            startPose (list): 1xn array of starting pose, where n can be: 
                - 3 for representing x, y, and z starting position,
                - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
                - 6 for representing x, y, z, r, p and y pose given in global origin.
            goalPose (list): 1xn array of goal pose, where n can be: 
                - 3 for representing x, y, and z starting position,
                - 4 for representing x, y, z and rotZ, where rotZ is rotation around global z axis,
                - 6 for representing x, y, z, r, p and y pose given in global origin.
            duration (float): Duration of trajectory.

        Raises:
            ValueError: If lengths of start and goal pose are not the same.
            ValueError: If value of duration parameter is smaller or equal to 0.

        Returns:
            tuple: nx7 array, representing pose trajectory with x, y, z, r, p, y and t values, where t are time stamps and 
            nx6 array representing velocity trajectory with x, y, z, r, p, y velocities, where n is the number of steps in trajectory.
        """
        if len(startPose) == 3:
            startPose = [startPose[0], startPose[1], startPose[2], 0.0 , 0.0, 0.0]
        
        if len(goalPose) == 3:
            goalPose = [goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, 0.0]

        if len(startPose) == 4:
            startPose = [startPose[0], startPose[1], startPose[2], 0.0, 0.0, startPose[3]]

        if len(goalPose) == 4:
            goalPose = [goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, goalPose[3]]

        if (len(startPose) != len(goalPose)):
            raise ValueError("Lengths of startPose and goalPose do not match.")
        if duration <= 0:
            raise ValueError("Movement duration cannot be shorter thatn 0 seconds.")

        timeStep = 1 / config.CONTROLLER_FREQUENCY
        timeVector = np.linspace(0, duration, int(duration / timeStep))

        trajectory = np.empty([len(timeVector), len(startPose) + 1], dtype = np.float32)
        velocities = np.empty([len(timeVector), len(startPose)], dtype = np.float32)
        for idx, t in enumerate(timeVector):
            trajectoryRow = np.empty([len(startPose) + 1], dtype = np.float32)
            velocityRow = np.empty([len(startPose)], dtype = np.float32)
            for i in range(len(startPose)):
                trajectoryRow[i] = startPose[i] + (goalPose[i] - startPose[i]) * (6 * math.pow(t/duration, 5) - 15 * math.pow(t/duration, 4) + 10 * math.pow(t/duration, 3))
                velocityRow[i] = (30 * math.pow(t, 2) * math.pow(duration - t, 2) * (goalPose[i] - startPose[i])) / math.pow(duration, 5)
            trajectoryRow[-1] = t
            trajectory[idx] = trajectoryRow
            velocities[idx] = velocityRow
        return trajectory, velocities

    def bezierTrajectory(self, startPosition, goalPosition, duration):
        """Calculate cubic bezier trajectory between start and goal point with fixed intermediat control points.

        Args:
            startPosition (list): 1x3 array of x, y, z values, representing starting position of trajectory, given in global origin.
            goalPosition (list): 1x3 array of x, y, z values, representing goal position of trajectory, given in global origin.
            duration (float): Duration of trajectory.

        Raises:
            ValueError: If lengths of given start and/or goal pose are not equal to 3.
            ValueError: If value of duration parameter is smaller or equal to 0.

        Returns:
            tuple: nx4 array, representing position trajectory with x, y, z and t values, where t are time stamps and 
            nx3 array representing velocity trajectory with x, y and z velocities, where n is the number of steps in trajectory.
        """

        if len(startPosition) != 3 or len(goalPosition) != 3:
            raise ValueError("Start and/or goal position were not given correctly.")
        if duration <= 0:
            raise ValueError("Movement duration cannot be shorter thatn 0 seconds.")

        timeStep = 1 / config.CONTROLLER_FREQUENCY
        numberOfSteps = math.floor(duration / timeStep)

        heightPercent = 0.6

        startPosition, goalPosition = np.array(startPosition), np.array(goalPosition)
        firstInterPoint = np.array([startPosition[0], startPosition[1], startPosition[2] + heightPercent * np.linalg.norm(goalPosition - startPosition)])
        secondInterPoint = np.array([goalPosition[0], goalPosition[1], goalPosition[2] + heightPercent * np.linalg.norm(goalPosition - startPosition)])
        controlPoints =  np.array([startPosition, firstInterPoint, secondInterPoint, goalPosition])

        timeVector = np.linspace(0, duration, numberOfSteps)
        trajectory = np.empty([len(timeVector), len(startPosition) + 1], dtype = np.float32)

        for idx, time in enumerate(timeVector):
            param = time / duration
            trajectoryPoint = controlPoints[0] * math.pow(1 - param, 3) + controlPoints[1] * 3 * param * math.pow(1 - param, 2) + controlPoints[2] * 3 * math.pow(param, 2) * (1 - param) + controlPoints[3] * math.pow(param, 3)
            trajectory[idx] = [trajectoryPoint[0], trajectoryPoint[1], trajectoryPoint[2], time]

        d = np.array([
            trajectory[:,0][-1] - trajectory[:,0][0],
            trajectory[:,1][-1] - trajectory[:,1][0],
            max(trajectory[:,2]) - trajectory[:,2][0] + max(trajectory[:,2]) - trajectory[:,2][-1]])

        vMax = 2 * (d / duration)
        a = 3 * vMax / duration
        t1 = duration / 3.0
        t2 = 2 * duration / 3.0

        velocity = np.empty([len(timeVector), len(startPosition)], dtype = np.float32)
        for idx, time in enumerate(timeVector):
            if 0 <= time <= t1:
                velocity[idx] = a * time
            elif t1 < time < t2:
                velocity[idx] = vMax
            elif t2 <= time <= duration:
                velocity[idx] = vMax - a * (time- t2)
        
        return trajectory, velocity

    def calculateTrajectory(self, start, goal, duration, type):
        """Wrapper for calcuating trajectories of desired type.

        Args:
            start: Start pose or position.
            goal: Goal pose or position.
            duration: Desired duration of movement.
            type: Desired trajectory type (bezier or minJerk).

        Raises:
            ValueError: If trajectory type is unknown.      

        Returns:
            Position and velocity trajectory if trajectory calculation was succesfull.
        """
        if type == 'bezier':
            return self.bezierTrajectory(start, goal, duration)
        elif type == 'minJerk':
            return self.minJerkTrajectory(start, goal, duration)
        else:
            raise ValueError("Unknown trajectory type!")
        
        
