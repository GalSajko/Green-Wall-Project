""" Module for path planning ang calculating trajectories.
"""

import math
import numpy as np 

import environment
import calculations

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
                        angleBetweenIdealVectorAndPin = self.geometryTools.calculateAngleBetweenTwoVectors2d(
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
        """

        startPose = np.array(startPose)
        goalPose = np.array(goalPose)

        # Max allowed distance between two steps is 1mm.
        maxStep = 0.01

        # Calculate biggest difference between single coordinates.
        biggestDiff = max(
            abs(startPose[0] - goalPose[0]), 
            abs(startPose[1] - goalPose[1]),
            abs(startPose[2] - goalPose[2]))
        
        # Calculate number of steps from biggest difference.
        numberOfSteps = math.floor(biggestDiff / maxStep)

        print(numberOfSteps)

        return np.linspace(startPose, goalPose, numberOfSteps)
