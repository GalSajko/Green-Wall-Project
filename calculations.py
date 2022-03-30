"""Module for all calculations, includes classes for kinematics, geometry and matrix calculations.
"""

import numpy as np
import math

import environment
import mappers


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

    def legDirectKinematics(self, legIdx, motorValues):
        """ Calculate direct kinematics for spiders leg, using DH parameters.  

        :param joints: Motors values in degrees.
        :return: Transformation matrix from base to end effector.
        """

        motorValues[1] = motorValues[1] + self.spider.SECOND_JOINTS_OFFSETS[legIdx]
        motorValues[2] = motorValues[2] - self.spider.SECOND_JOINTS_OFFSETS[legIdx]
        jointValues = mappers.mapMotorRadiansToJointRadians(np.radians(motorValues))
        q1, q2, q3 = jointValues
        
        q2 = q2 - self.spider.SECOND_JOINTS_OFFSETS[legIdx]
        q3 = q3 + self.spider.SECOND_JOINTS_OFFSETS[legIdx]       

        H01 = MatrixCalculator().calculateHMatrix(q1, self.DH_MODEL["alpha"][0], self.DH_MODEL["r"][0], self.DH_MODEL["d"])
        H12 = MatrixCalculator().calculateHMatrix(q2, self.DH_MODEL["alpha"][1], self.DH_MODEL["r"][1], self.DH_MODEL["d"])
        H23 = MatrixCalculator().calculateHMatrix(q3, self.DH_MODEL["alpha"][2], self.DH_MODEL["r"][2], self.DH_MODEL["d"])

        H03 = np.dot(H01, np.dot(H12, H23)) 
        return H03
    
    def legInverseKinematics(self, legIdx, endEffectorPosition):
        """ Calculate inverse kinematics for leg, using geometry - considering L shape of second link.

        :param legIdx: Index of leg (0 - 4).
        :param endEffectorPosition: Desired end effector positions in leg-base origin.
        :return: Joint values in radians.
        """
        endEffectorPosition = np.array(endEffectorPosition)
        endEffectorPosition[2] = -endEffectorPosition[2]

        firstLink = self.spider.LEGS[legIdx][0]
        # REV: move this calculation to Spider().
        virtualSecondLink = np.hypot(self.spider.LEGS[legIdx][1][0], self.spider.LEGS[legIdx][1][1])
        thirdLink = self.spider.LEGS[legIdx][2]

        # Angle in first joint.
        q1 = math.atan2(endEffectorPosition[1], endEffectorPosition[0])

        # Position of second joint in leg-base origin.
        secondJointPosition = np.array([
            firstLink * math.cos(q1), 
            firstLink * math.sin(q1), 
            0])

        # Vector from second joint to end effector.
        secondJointToEndVector = np.array(endEffectorPosition - secondJointPosition)

        # Distance between second joint and end effector.
        r = GeometryTools().calculateEuclideanDistance3d(secondJointPosition, endEffectorPosition)
        # Angle in third joint.
        q3 = -math.acos((r**2 - virtualSecondLink**2 - thirdLink**2) / (2 * virtualSecondLink * thirdLink))
        # Angle in second joint.
        q2 = math.atan2(secondJointToEndVector[2], np.linalg.norm(secondJointToEndVector[0:2])) + \
            math.atan2(thirdLink * math.sin(q3), virtualSecondLink + thirdLink * math.cos(q3))
        
        # Add offset because 2nd and 3rd joints are not aligned.
        q2 = -(q2 - self.spider.SECOND_JOINTS_OFFSETS[legIdx])
        q3 = q3 - self.spider.SECOND_JOINTS_OFFSETS[legIdx]

        return q1, q2, q3

    def platformInverseKinematics(self, goalPose, pins):
        """Calculate inverse kinematics for spiders platform.

        :param goalPose: Goal pose in global, given as a 1x6 array with positions and rpy orientation.
        :param pins: Global positions of used pins.
        :return joints: 3x5 matrix with joints values for all legs.
        """
        # Get transformation matrix from origin to spider base, from desired goalPose.
        globalTransformMatrix = MatrixCalculator().transformMatrixFromGlobalPose(goalPose)

        # Array to store calculated joints values for all legs.
        joints = []
        for idx, t in enumerate(self.spider.T_ANCHORS):
            anchorInGlobal = np.dot(globalTransformMatrix, t)
            anchorInGlobalPosition = anchorInGlobal[:,3][0:3]
            pinToAnchorGlobal = np.array(pins[idx] - anchorInGlobalPosition)          

            pinToAnchorLocal = np.dot(np.linalg.inv(t), np.array([
                [1, 0, 0, pinToAnchorGlobal[0]],
                [0, 1, 0, pinToAnchorGlobal[1]],
                [0, 0, 1, pinToAnchorGlobal[2]],
                [0, 0, 0, 1]]))
            pinToAnchorLocalPosition = pinToAnchorLocal[:,3][0:3]

            # With inverse kinematics for single leg calculate joints values.
            q1, q2, q3 = self.legInverseKinematics(idx, pinToAnchorLocalPosition)
            joints.append(np.array([q1, q2, q3]))

        return np.array(joints)

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

class MatrixCalculator:
    """ Class for calculating matrices."""

    def transformMatrixFromGlobalPose(cls, globalPose):
        """Calculate global transformation matrix for transforming between global origin and spider base.

        :param globalPose: Desired global pose of a spiders platform, 1x6 array with positions and rpy orientation.
        """
        position = globalPose[0:3]
        rpy = globalPose[3:]

        roll = np.array([
            [1, 0, 0],
            [0, math.cos(rpy[0]), -math.sin(rpy[0])],
            [0, math.sin(rpy[0]), math.cos(rpy[0])]
        ])
        pitch = np.array([
            [math.cos(rpy[1]), 0, math.sin(rpy[1])],
            [0, 1, 0],
            [-math.sin(rpy[1]), 0, math.cos(rpy[1])]
        ])
        yaw = np.array([
            [math.cos(rpy[2]), -math.sin(rpy[2]), 0],
            [math.sin(rpy[2]), math.cos(rpy[2]), 0],
            [0, 0, 1]
        ])

        rotationMatrix = np.dot(roll, np.dot(pitch, yaw))

        transformMatrix = np.c_[rotationMatrix, position]
        transformMatrix = np.r_[transformMatrix, [[0, 0, 0, 1]]]
        
        return transformMatrix

    def calculateHMatrix(cls, theta, alpha, r, d):
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
