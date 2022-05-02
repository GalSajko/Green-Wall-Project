""" Module for path planning ang calculating trajectories.
"""

import math
from multiprocessing.sharedctypes import Value
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
        if duration <= 0:
            print("Invalid duration parameter.")
            return

        timeStep = 0.05
        timeVector = np.linspace(0, duration, duration / timeStep)

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

        :param startPoint: Starting poing.
        :param goalPoint: Goal point.
        :param duration: Duration of movement between start and goal point.
        :return: Cubic Bezier trajectory with positions and time steps and velocities in each step.
        """

        if (len(startPoint) != len(goalPoint)):
            print("Invalid start and goal points.")
            return
        if duration <= 0:
            print("Invalid duration.")
            return

        timeStep = 0.02
        numberOfSteps = math.floor(duration / timeStep)

        startPoint, goalPoint = np.array(startPoint), np.array(goalPoint)
        firstInterPoint = np.array([startPoint[0], startPoint[1], startPoint[2] + 0.3 * np.linalg.norm(goalPoint - startPoint)])
        secondInterPoint = np.array([goalPoint[0], goalPoint[1], goalPoint[2] + 0.3 * np.linalg.norm(goalPoint - startPoint)])
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

    

        
        
