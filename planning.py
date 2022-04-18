""" Module for path planning ang calculating trajectories.
"""

import math
import numpy as np 

import environment
import calculations
from calculations import GeometryTools

class PathPlanner:
    """Class for calculating spiders path and its legs positions on each step of the path.
    """
    def __init__(self):
        self.spider = environment.Spider()
        self.wall = environment.Wall()
        self.pins = self.wall.createGrid()
        self.geometryTools = calculations.GeometryTools()

    def calculateSpiderBodyPath(self, start, goal, maxStep):
        """Calculate descrete path of spiders body.

        :param start: Start point.
        :param goal: Goal point.
        :param maxStep: Max step between two points in meters.
        :return: Array of (x, y) points, representing the descrete path.
        """
        distanceToTravel = self.geometryTools.calculateEuclideanDistance2d(start, goal)                   
        numberOfSteps = math.floor(distanceToTravel / maxStep)
        # Discrete path 
        path = np.array([np.linspace(start[0], goal[0], numberOfSteps),
                        np.linspace(start[1], goal[1], numberOfSteps)])
        
        path = np.transpose(path)
        return path

    def calculateSpiderLegsPositionsFF(self, path, params = [1/3, 1/3, 1/3]):
        """Calculate legs positions for each step on the path.

        :param path: Spiders path.
        :param params: Values of parameters for calculating best pin to put a leg on, defaults to [1/3, 1/3, 1/3]
        :return: Array of selected pins for each leg on each step on the path.
        """
        selectedPins = []
        for step, (x, y) in enumerate(path):
            # Leg anchors in global origin.
            legAnchors = self.spider.LEG_ANCHORS + [x, y]
            # Selected pins for each leg on each step.
            selectedPinsOnEachStep = []
            for idx, legAnchor in enumerate(legAnchors):
                # Potential pins for each leg.
                potentialPinsForSingleLeg = []
                for pin in self.pins:
                    # First check distance from anchor to pin.
                    distanceToPin = self.geometryTools.calculateEuclideanDistance2d(legAnchor, pin)
                    if self.spider.CONSTRAINS[0] < distanceToPin < self.spider.CONSTRAINS[1]:                       
                        # Than check angle between ideal leg direction and pin.
                        angleBetweenIdealVectorAndPin = self.geometryTools.calculateSignedAngleBetweenTwoVectors(
                            self.spider.IDEAL_LEG_VECTORS[idx], 
                            np.array(np.array(pin) - np.array(legAnchor)))
                        if abs(angleBetweenIdealVectorAndPin) < self.spider.CONSTRAINS[2]:
                            if step != 0 and step != len(path) - 1:
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

class TrajectoryPlanner:
    """ Class for calculating different trajectories.
    """
    def platformLinearTrajectory(self, startPose, goalPose):
        """ Calculate linear trajectory for platform movement.

        :param startPose: Start pose in global, 1x6 vector with position and rpy orientation.
        :param goalPose: Goal pose in global, 1x6 vector with position and rpy orientation
        :return: Array of points in global origin which represents trajectory.
        """

        startPose = np.array(startPose)
        goalPose = np.array(goalPose)

        # Max allowed distance between two steps is 1cm.
        maxStep = 0.01

        # Calculate biggest difference between single coordinates.
        biggestDiff = max(
            abs(startPose[0] - goalPose[0]), 
            abs(startPose[1] - goalPose[1]),
            abs(startPose[2] - goalPose[2]),
            abs(startPose[3] - goalPose[3]),
            abs(startPose[4] - goalPose[4]),
            abs(startPose[5] - goalPose[5]))
        
        # Calculate number of steps from biggest difference.
        numberOfSteps = math.floor(biggestDiff / maxStep)

        return np.linspace(startPose, goalPose, numberOfSteps)
    
    def legCircularTrajectory(self, startPose, goalPose):
        """ Calculate half-circular trajectory for leg movement.

        :param startPose: Leg's start position in leg-based origin (read with MotorDriver.readLegPositon).
        :param goalPose: Desired position in leg-based origin.
        :return: Array of points in leg-based origin which represents trajectory.
        """
        startPose = np.array(startPose)
        goalPose = np.array(goalPose)

        # Distance between start and goal point.
        d = GeometryTools().calculateEuclideanDistance3d(startPose, goalPose)
        r = d / 2.0
        if r < 0.05:
            r = 0.05

        # Length of the curve with half-circular shape.
        l = 0.5 * math.pi * d
        maxStep = 0.02
        numberOfSteps = math.floor(l / maxStep)

        if numberOfSteps <= 1:
            numberOfSteps = 2

        startZ = startPose[2]
        endZ = goalPose[2]

        circularTraj = np.linspace(startPose, goalPose, numberOfSteps)

        zFirst = np.linspace(startZ, startZ + r, numberOfSteps / 2)
        zSecond = np.linspace(startZ + r, endZ, (numberOfSteps / 2) + 1)

        try:
            z = np.append(zFirst, zSecond)
            circularTraj[:, 2] = z
        except:
            z = np.append(zFirst[:-1], zSecond)
            circularTraj[:, 2] = z
        
        return circularTraj

    def minJerkTrajectory(self, startPose, goalPose, duration, startVelocity = [0, 0, 0, 0, 0, 0], goalVelocity = [0, 0, 0, 0, 0, 0]):
        """Calculate minimum jerk trajectory from start to goal position.

        :param startPose: Start pose (x, y, z, roll, pitch, yaw).
        :param goalPose: Goal pose (x, y, z, roll, pitch, yaw).
        :param duration: Duration of movement in seconds.
        :param startVelocity: Start velocity, defaults to 0
        :param goalVelocity: Goal velocity, defaults to 0
        :return: Trajectory with pose and time stamp for each step and velocities in each pose.
        """

        if (len(startPose) != len(goalPose) and len(startPose) != len(startVelocity) and len(startPose) != len(goalVelocity)):
            print("Invalid parameters.")
            return

        timeStep = 0.1
        timeVector = np.linspace(0, duration, duration / timeStep)

        trajectory = []
        velocities = []
        for t in timeVector:
            tau = t / duration
            trajectoryRow = [startPose[i] + (goalPose[i] - startPose[i]) * (6 * math.pow(tau, 5) - 15 * math.pow(tau, 4) + 10 * math.pow(tau, 3)) for i in range(len(startPose))]
            velocityRow = [goalPose[i] * (30 * math.pow(tau, 4) - 60 * math.pow(tau, 3) + 30 * math.pow(tau, 2)) for i in range(len(startPose))]
            trajectoryRow.append(t)
    
            trajectory.append(trajectoryRow)
            velocities.append(velocityRow)



        return np.array(trajectory), np.array(velocities)

    def bezierTrajectory(self, startPose, goalPose, duration):
        timeStep = 0.1
        numberOfSteps = math.floor(duration / timeStep)
        trajectory = []
        startPose, goalPose = np.array(startPose), np.array(goalPose)
        firstInterPoint = np.array([startPose[0], startPose[1] + 0.15])
        secondInterPoint = np.array([goalPose[0], goalPose[1] + 0.15])
        controlPoints =  np.array([startPose, firstInterPoint, secondInterPoint, goalPose]) 
        parameterVector = np.linspace(0, 1, numberOfSteps)
        timeVector = np.linspace(0, duration, numberOfSteps)
        for idx, param in enumerate(parameterVector):
            point = controlPoints[0] * math.pow(1 - param, 3) + controlPoints[1] * 3 * param * math.pow(1 - param, 2) + controlPoints[2] * 3 * math.pow(param, 2) * (1 - param) + controlPoints[3] * math.pow(param, 3)
            trajectory.append([point[0], point[1], timeVector[idx]])

        return np.array(trajectory)

        
        
