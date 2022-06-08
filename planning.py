""" Module for path planning ang calculating trajectories.
"""

import math
import numpy as np 

import environment
import calculations


class PathPlanner:
    """Class for calculating spiders path and its legs positions on each step of the path.
    """
    def __init__(self, maxLinStep, maxRotStep, gridPattern = 'squared'):
        self.spider = environment.Spider()
        self.wall = environment.Wall(gridPattern)
        self.geometryTools = calculations.GeometryTools()
        self.matrixCalculator = calculations.MatrixCalculator()

        self.maxLinStep = maxLinStep
        self.maxRotStep = maxRotStep
        self.maxLiftStep = 0.3
    
    def calculateSpiderBodyPath(self, startPose, goalPose):
        """Calculate descrete path of spider's body including rotation around z axis. First rotate toward goal point,
        than move towards this point and lastly, if rotateInGoalOrientation is True, rotate into goal orientation.

        :param startPose: Starting pose as (x, y, z, rotZ) where rotZ is rotation around global z axis.
        :param goalPose: Goal pose as (x, y, z, rotZ).
        :return: Array of (x, y, z, rotZ) poses, representing the descrete path.
        """
        # Path segments: first rotate towards goal position, than move from start to goal position
        # and finally rotate into goal orientation.

        path = [startPose]

        # If spider is lying on the pins first lift it up on the walking height.
        if startPose[2] == self.spider.LYING_HEIGHT:
            startWalkingPose = np.copy(startPose)
            startWalkingPose[2] = self.spider.WALKING_HEIGHT
            path.append(startWalkingPose)

        # Rotate towards goal point.
        refAngle = math.atan2(goalPose[0] - startPose[0], goalPose[1] - startPose[1]) * (-1)
        angleError = self.geometryTools.wrapToPi(refAngle - startPose[3])
        if angleError != 0.0:
            numberOfSteps = math.ceil(abs(angleError) / self.maxRotStep) + 1
            rotatedPose = np.copy(path[-1])
            rotatedPose[3] = refAngle 
            rotatePath = np.linspace(path[-1], rotatedPose, numberOfSteps)
            for pose in rotatePath:
                path.append(pose)

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

        # Rotate in goal orientation.
        angleError = self.geometryTools.wrapToPi(path[-1][3] - goalPose[3])
        if angleError != 0.0:
            numberOfSteps = math.ceil(abs(angleError) / self.maxRotStep) + 1
            rotatePath = np.linspace(path[-1], goalPose, numberOfSteps )
            for pose in rotatePath:
                path.append(pose)

        return np.array(path)

    def calculateSpiderLegsPositionsXyzRpyFF(self, path):
        """Calculate legs positions for each step on the path, including spider's rotations.

        :param path: Spider's path.
        :return: Array of selected pins for each leg on each step of the path.
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

                potentialPinsForSingleLeg = np.array(potentialPinsForSingleLeg, dtype = object)
                potentialPinsForSingleLeg = potentialPinsForSingleLeg[potentialPinsForSingleLeg[:, 1].argsort()]
                selectedPinsOnEachStep.append(potentialPinsForSingleLeg[0][0])

            selectedPins.append(selectedPinsOnEachStep)

        return selectedPins                   

    def calculateSpiderLegsPositionsFF(self, path, params = [1/3, 1/3, 1/3]):
        """Calculate legs positions for each step on the path.

        :param path: Spider's path.
        :param params: Values of parameters for calculating best pin to put a leg on, defaults to [1/3, 1/3, 1/3]
        :return: Array of selected pins for each leg on each step on the path.
        """
        pins = self.wall.createGrid()
        selectedPins = []
        for step, (x, y) in enumerate(path):
            # Leg anchors in global origin.
            legAnchors = self.spider.LEG_ANCHORS + [x, y]
            # Selected pins for each leg on each step.
            selectedPinsOnEachStep = []
            for idx, legAnchor in enumerate(legAnchors):
                # Potential pins for each leg.
                potentialPinsForSingleLeg = []
                for pin in pins:
                    # First check distance from anchor to pin.
                    distanceToPin = self.geometryTools.calculateEuclideanDistance2d(legAnchor, pin)
                    if self.spider.CONSTRAINS[0] < distanceToPin < self.spider.CONSTRAINS[1]:                       
                        # Than check angle between ideal leg direction and pin.
                        angleBetweenIdealVectorAndPin = self.geometryTools.calculateSignedAngleBetweenTwoVectors(
                            self.spider.IDEAL_LEG_VECTORS[idx], 
                            np.array(np.array(pin) - np.array(legAnchor)))
                        if abs(angleBetweenIdealVectorAndPin) < self.spider.CONSTRAINS[2]:
                            if step not in (0, len(path) - 1):
                                previousPin = selectedPins[step - 1][idx]
                                # criterion - pick pin with best distance, minimum angle and those with minimum distance from previous pin (avoid unnecesarry movement).
                                distanceBetweenSelectedAndPreviousPin = self.geometryTools.calculateEuclideanDistance2d(previousPin, pin)
                                isLegMoving = 0 if distanceBetweenSelectedAndPreviousPin == 0 else 1
                                distanceFromPinToEndPoint = self.geometryTools.calculateEuclideanDistance2d(pin, path[-1])
                                criterionFunction = params[0] * abs(angleBetweenIdealVectorAndPin) + params[1] * isLegMoving + params[2] * distanceFromPinToEndPoint
                            else:
                                # criterion - pick pin with best distance and minimal angle.
                                criterionFunction = abs(distanceToPin - self.spider.CONSTRAINS[1]) + abs(angleBetweenIdealVectorAndPin)
                            potentialPinsForSingleLeg.append([pin, criterionFunction])

                potentialPinsForSingleLeg = np.array(potentialPinsForSingleLeg)
                # Sort potential pins based on minimal value of criterion function.
                potentialPinsForSingleLeg = potentialPinsForSingleLeg[potentialPinsForSingleLeg[:, 1].argsort()]
                # Append potential pin with minimum criterion function value to selected pins on each step.
                selectedPinsOnEachStep.append(potentialPinsForSingleLeg[0][0])

            selectedPins.append(selectedPinsOnEachStep)

        return selectedPins
    
    def calculateWalkingMovesFF(self, globalStartPose, globalGoalPose):
        """Feed forward calculations of platform and legs positions during walking procedure. 
        Platform is moving until one (or more) of the legs has to move on new pin.

        :param globalStartPose: Spider's starting pose in global (wall) origin
        :param globalGoalPose: Spider's goal pose in global (wall) origin.
        :param maxStep: Platform step.
        :return: Arrays of calculated platform poses and selected pins.
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
    def minJerkTrajectory(self, startPose, goalPose, duration, startVelocity = [0, 0, 0, 0, 0, 0], goalVelocity = [0, 0, 0, 0, 0, 0]):
        """Calculate minimum jerk trajectory from start to goal position.

        :param startPose: Start pose (x, y, z, roll, pitch, yaw).
        :param goalPose: Goal pose (x, y, z, roll, pitch, yaw).
        :param duration: Duration of movement in seconds.
        :param startVelocity: Start velocity, defaults to 0
        :param goalVelocity: Goal velocity, defaults to 0
        :return: Trajectory with pose and time stamp for each step and velocities in each pose.
        """
        if len(startPose) == 3:
            startPose = [startPose[0], startPose[1], startPose[2], 0.0 , 0.0, 0.0]
        
        if len(goalPose) == 3:
            goalPose = [goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, 0.0]

        if len(startPose) == 4:
            startPose = [startPose[0], startPose[1], startPose[2], 0.0, 0.0, startPose[3]]

        if len(goalPose) == 4:
            goalPose = [goalPose[0], goalPose[1], goalPose[2], 0.0, 0.0, goalPose[3]]

        if (len(startPose) != len(goalPose) or len(startPose) != len(startVelocity) or len(startPose) != len(goalVelocity)):
            print("Invalid parameters.")
            return
        if duration <= 0:
            print("Invalid duration parameter.")
            return

        timeStep = 0.02
        timeVector = np.linspace(0, duration, int(duration / timeStep))

        trajectory = []
        velocities = []
        for t in timeVector:
            trajectoryRow = []
            velocityRow = []
            for i in range(len(startPose)):
                trajectoryRow.append(startPose[i] + (goalPose[i] - startPose[i]) * (6 * math.pow(t/duration, 5) - 15 * math.pow(t/duration, 4) + 10 * math.pow(t/duration, 3)))
                velocityRow.append((30 * math.pow(t, 2) * math.pow(duration - t, 2) * (goalPose[i] - startPose[i])) / math.pow(duration, 5))
            trajectoryRow.append(t)
            trajectory.append(trajectoryRow)
            velocities.append(velocityRow)
        return np.array(trajectory), np.array(velocities)

    def bezierTrajectory(self, startPoint, goalPoint, duration):
        """ Calculate cubic bezier trajectory between start and goal pose with fixed intermediate control points.

        :param startPoint: Starting point
        :param goalPoint: Goal point.
        :param duration: Duration of movement between start and goal point.
        :return: Cubic Bezier trajectory with positions, time steps and velocities in each step.
        """

        if (len(startPoint) != len(goalPoint)):
            print("Invalid start and goal points.")
            return
        if duration <= 0:
            print("Invalid duration.")
            return

        timeStep = 0.02
        numberOfSteps = math.floor(duration / timeStep)

        heightPercent = 0.4

        startPoint, goalPoint = np.array(startPoint), np.array(goalPoint)
        firstInterPoint = np.array([startPoint[0], startPoint[1], startPoint[2] + heightPercent * np.linalg.norm(goalPoint - startPoint)])
        secondInterPoint = np.array([goalPoint[0], goalPoint[1], goalPoint[2] + heightPercent * np.linalg.norm(goalPoint - startPoint)])
        controlPoints =  np.array([startPoint, firstInterPoint, secondInterPoint, goalPoint])

        trajectory = []
        timeVector = np.linspace(0, duration, numberOfSteps)

        for time in timeVector:
            param = time / duration
            trajectoryPoint = controlPoints[0] * math.pow(1 - param, 3) + controlPoints[1] * 3 * param * math.pow(1 - param, 2) + controlPoints[2] * 3 * math.pow(param, 2) * (1 - param) + controlPoints[3] * math.pow(param, 3)
            trajectory.append([trajectoryPoint[0], trajectoryPoint[1], trajectoryPoint[2], time])

        trajectory = np.array(trajectory)

        d = np.array([
            trajectory[:,0][-1] - trajectory[:,0][0],
            trajectory[:,1][-1] - trajectory[:,1][0],
            max(trajectory[:,2]) - trajectory[:,2][0] + max(trajectory[:,2]) - trajectory[:,2][-1]])

        vMax = 2 * (d / duration)
        a = 3 * vMax / duration
        t1 = duration / 3.0
        t2 = 2 * duration / 3.0

        velocity = []
        for time in timeVector:
            if 0 <= time <= t1:
                velocity.append(a * time)
            elif t1 < time < t2:
                velocity.append(vMax)
            elif t2 <= time <= duration:
                velocity.append(vMax - a * (time- t2))

        return np.array(trajectory), np.array(velocity)

    

        
        
