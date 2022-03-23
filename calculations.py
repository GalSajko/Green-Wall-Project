"""Module for all calculations, includes classes for kinematics calculations, path planning and
helper class for geometry calculations.
"""

import numpy as np
import math

import environment

class PathPlanner:
    """Class for calculating spiders path and its legs positions on each step of the path.
    """
    def __init__(self):
        self.spider = environment.Spider()
        self.wall = environment.Wall()
        self.pins = self.wall.createGrid()
        self.geometryTools = GeometryTools()

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

class Kinematics:
    """Class for calculating kinematics of spider robot. Includes direct and reverse kinematics for spiders legs.
    """
    def __init__(self):
        self.spider = environment.Spider()
        # leg DH parameters - note: DH model is slightly different than real spiders leg, because of the shape of second link.
        # Here, DH model takes a second link as a straight line.
        # Missing thetas are variables and will come as a joints values.
        self.DH_MODEL = {
            "alpha" : [math.pi / 2, 0, 0],
            "r" : [0.064, 0.301, 0.275],
            "d" : 0
        }
        # Length of a virtual representation of second link - as it would not be in L shape, but directly
        # connecting second and third joint.
        self.VIRTUAL_SECOND_LINK = np.hypot(self.spider.LEGS[0][1][0], self.spider.LEGS[0][1][1])

    def legDirectKinematics(self, joints):
        """ Calculate direct kinematics for spiders leg, using DH parameters.  

        :param joints: Joints values in degrees from motors.
        :return: Transformation matrix from base to end effector.
        """
        q1, q2, q3 = joints

        # CONVERT MOTOR VALUES TO DH MODEL

        return H03
    
    def calculateHMatrix(self, theta, alpha, r, d):
        """ Helper function for calculating transformation matrix between two origins, using DH model.

        :param theta: DH parameter theta.
        :param alpha: DH parameter alpha.
        :param r: DH parameter r.
        :param d: DH parameter d.
        :return: Transformation matrix between two origins.
        """
        return np.array([
            [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), r * math.cos(theta)],
            [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), r * math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def legInverseKinematics(self, legIdx, endEffectorPosition):
        """ Calculate inverse kinematics for leg, using geometry - considering L shape of second link.

        :param legIdx: Index of leg (0 - 4).
        :param endEffectorPosition: Desired end effector positions in leg-base origin.
        :return: Joint values in radians.
        """
        endEffectorPosition = np.array(endEffectorPosition)

        base = np.array([0, 0, 0])
        firstLink = self.spider.LEGS[legIdx][0]
        # REV: move this calculation to Spider().
        virtualSecondLink = np.hypot(self.spider.LEGS[legIdx][1][0], self.spider.LEGS[legIdx][1][1])
        thirdLink = self.spider.LEGS[legIdx][2]

        # Angle in first joint.
        q1 = math.atan2(endEffectorPosition[1], endEffectorPosition[0])

        # Position of second joint in base origin.
        secondJointPosition = np.array([
            base[0] + firstLink * math.cos(q1), 
            base[0] + firstLink * math.sin(q1), 
            base[0]])

        # Vector from second joint to end effector.
        secondJointToEndVector = np.array(endEffectorPosition - secondJointPosition)
        # Distance between second joint and end effector.
        r = GeometryTools().calculateEuclideanDistance3d(secondJointPosition, endEffectorPosition)
        # Angle in third joint.
        q3 = -math.acos((r**2 - virtualSecondLink**2 - thirdLink**2) / (2 * virtualSecondLink * thirdLink))

        # Angle in second joint.
        q2 = math.atan2(abs(secondJointToEndVector[2]), np.linalg.norm(secondJointToEndVector[0:2])) + \
            math.atan2(thirdLink * math.sin(q3), virtualSecondLink + thirdLink * math.cos(q3))
        
        # Add offset because 2nd and 3rd joints are not aligned.
        q2 = -(q2 + self.spider.SECOND_JOINTS_OFFSETS[legIdx])
        q3 = q3 + self.spider.SECOND_JOINTS_OFFSETS[legIdx]

        ######
        # q1 = q1 + math.pi
        # q2 = math.pi - q2
        # q3 = math.pi + q3
        ######

        return q1, q2, q3


class GeometryTools:
    """Helper class for geometry calculations.
    """
    def calculateEuclideanDistance2d(cls, firstPoint, secondPoint):
        """Calculate euclidean distance between two points in 2d.

        :param firstPoint: First point.
        :param secondPoint: Second point.
        :return: Distance between two points.
        """
        return math.sqrt((firstPoint[0] - secondPoint[0])**2 + (firstPoint[1] - secondPoint[1])**2)

    def calculateEuclideanDistance3d(cls, firstPoint, secondPoint):
        """Calculate euclidean distance between two points in 3d.

        :param firstPoint: First point.
        :param secondPoint: Second point.
        :return: Distance between two points in 3d.
        """
        return math.sqrt((firstPoint[0] - secondPoint[0])**2 + (firstPoint[1] - secondPoint[1])**2 + (firstPoint[2] - secondPoint[2])**2)
    
    def calculateAngleBetweenTwoVectors2d(cls, firstVector, secondVector):
        """Calculate signed angle between two vectors in 2d.

        :param firstVector: First vector.
        :param secondVector: Second vector.
        :return: Signed angle between two vectors in radians.    
        """
        dotProduct = np.dot(firstVector, secondVector)
        productOfNorms = np.linalg.norm(firstVector) * np.linalg.norm(secondVector)
        angle = math.acos(dotProduct / productOfNorms)
        crossProduct = np.cross(firstVector, secondVector)
        if crossProduct < 0:
            angle = -angle
        return angle

    def wrapToPi(cls, angle):
        """Wrap angle to Pi.

        :param angle: Angle
        :return: Angle wrapped to Pi.
        """
        if angle < -math.pi:
            angle += math.pi * 2
        elif angle > math.pi:
            angle -= math.pi * 2
        return angle
