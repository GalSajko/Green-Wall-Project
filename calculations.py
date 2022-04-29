"""Module for all calculations, includes classes for kinematics, geometry and matrix calculations.
"""

import numpy as np
import math

import environment as env


class Kinematics:
    """Class for calculating kinematics of spider robot. Includes direct and reverse kinematics for spiders legs.
    """
    def __init__(self):
        self.spider = env.Spider()
        # leg DH parameters - note: DH model is slightly different than real spiders leg, because of the shape of second link.
        # Here, DH model takes a second link as a straight line.
        # Missing thetas are variables and will come as a joints values.
        self.DH_MODEL = {
            "alpha" : [math.pi / 2, 0, 0],
            "r" : [0.064, 0.301, 0.275],
            "d" : 0
        }

    def legDirectKinematics(self, legIdx, jointValues):
        """ Calculate direct kinematics for spiders leg, using transformation matrices.  

        :param jointValues: Joint values in radians.
        :return: Transformation matrix from base to end effector.
        """
        q1, q2, q3 = jointValues
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]
        L4 = self.spider.LEGS[legIdx][2]

        return np.array([
            [math.cos(q1)*math.cos(q2+q3), -math.cos(q1)*math.sin(q2+q3), math.sin(q1), math.cos(q1)*(L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2))],
            [math.cos(q2+q3)*math.sin(q1), -math.sin(q1)+math.sin(q2+q3), -math.cos(q1), math.sin(q1)*(L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2))],
            [math.sin(q2+q3), math.cos(q2+q3), 0, -L3*math.cos(q2) + L2*math.sin(q2) + L4*math.sin(q2+q3)],
            [0, 0, 0, 1]
        ])
    
    def legInverseKinematics(self, legIdx, endEffectorPosition):
        """ Calculate inverse kinematics for leg, using geometry - considering L shape of second link.

        :param legIdx: Index of leg (0 - 4).
        :param endEffectorPosition: Desired end effector positions in leg-base origin.
        :return: Joint values in radians.
        """
        endEffectorPosition = np.array(endEffectorPosition)
        endEffectorPosition[2] = -endEffectorPosition[2]

        firstLink = self.spider.LEGS[legIdx][0]
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

    def platformInverseKinematics(self, goalPose, legs):
        """Calculate inverse kinematics for spiders platform.

        :param goalPose: Goal pose in global, given as a 1x6 array with positions and xyz orientation.
        :param legs: Global positions of used legs.
        :return: 3x5 matrix with joints values for all legs.
        """
        # Get transformation matrix from spiders xyzrpy.
        globalTransformMatrix = MatrixCalculator().xyzRpyToMatrix(goalPose)

        # Array to store calculated joints values for all legs.
        joints = []
        for idx, t in enumerate(self.spider.T_ANCHORS):
            # Pose of leg anchor in global
            anchorInGlobal = np.dot(globalTransformMatrix, t)
            # Position of leg anchor in global.
            anchorInGlobalPosition = anchorInGlobal[:,3][0:3]

            # Vector from anchor to end of leg in global.
            anchorToPinGlobal = np.array(legs[idx] - anchorInGlobalPosition)
            # Transform this vector in legs local origin - only rotate.
            rotationMatrix = anchorInGlobal[:3, :3]
            anchorToPinLocal = np.dot(np.linalg.inv(rotationMatrix), anchorToPinGlobal)

            # With inverse kinematics for single leg calculate joints values.
            q1, q2, q3 = self.legInverseKinematics(idx, anchorToPinLocal)
            joints.append(np.array([q1, q2, q3]))

        return np.array(joints)

    def legJacobi(self, legIdx, jointValues):
        """ Calculate Jacobian matrix for single leg.

        :param legId: Leg id.
        :jointValues: Joint values in radians.   
        :return: 3x3 Jacobian matrix.
        """
        q1, q2, q3 = jointValues
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]
        L4 = self.spider.LEGS[legIdx][2]

        return np.array([
            [-math.sin(q1)*(L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2)), math.cos(q1)*(L3*math.cos(q2) - L2*math.sin(q2) - L4*math.sin(q2+q3)), -L4*math.cos(q1)*math.sin(q2+q3)],
            [math.cos(q1)*(L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2)), math.sin(q1)*(L3*math.cos(q2) - L2*math.sin(q2) - L4*math.sin(q2+q3)), -L4*math.sin(q1)*math.sin(q2+q3)],
            [0, L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2), L4*math.cos(q2+q3)] 
            ])

    def legToSpiderBaseDirectKinematic(self, legIdx, jointValues):
        """Calculate direct kinematics from leg-tip to spider base.
        
        :param legIdx: Leg id.
        :param jointValues: Joint values in radians.
        :return: Transformation matrix from leg-tip to spider base.
        """
        qb = legIdx * self.spider.ANGLE_BETWEEN_LEGS + math.pi / 2
        q1, q2, q3 = jointValues
        r = self.spider.BODY_RADIUS
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]
        L4 = self.spider.LEGS[legIdx][2]

        Hb3 = np.array([
            [math.cos(q2+q3) * math.cos(q1+qb), -math.cos(q1+qb) * math.sin(q2+q3), math.sin(q1+qb), r*math.cos(qb) + math.cos(q1+qb) * (L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2))],
            [math.cos(q2+q3) * math.sin(q1+qb), -math.sin(q2+q3) * math.sin(q1+qb), -math.cos(q1+qb), L1*math.cos(q1)*math.sin(q1) + (r + L1*math.cos(q1)) * math.sin(qb) + (L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2)) * math.sin(q1+qb)],
            [math.sin(q2+q3), math.cos(q2+q3), 0, -L3*math.cos(q2) + L2*math.sin(q2) + L4*math.sin(q2+q3)],
            [0, 0, 0, 1]
        ])

        H3b = np.linalg.inv(Hb3)
        return H3b

    def legTipToSpiderBaseJacobi(self, legIdx, jointValues):
        """Calculate Jacobian matrix for spiders origin - leg-tip relation.

        :param legIdx: Leg id.
        :param jointValues: Joint values in radians.
        :return: 3x3 Jacobian matrix.
        """
        q1, q2, q3 = jointValues
        r = self.spider.BODY_RADIUS
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]
        L4 = self.spider.LEGS[legIdx][2]

        return np.array([
            [r*math.cos(q2+q3)*math.sin(q1), (L1 + r*math.cos(q1))*math.sin(q2+q3), math.cos(q3)*(L3 + (L1 + r*math.cos(q1))*math.sin(q2)) + (L2 + (L1 + r*math.cos(q1))*math.cos(q2))*math.sin(q3)],
            [-r*math.sin(q1)*math.sin(q2+q3), (L1 + r*math.cos(q1))*math.cos(q2+q3), (L2 + (L1 + r*math.cos(q1))*math.cos(q2))*math.cos(q3) - (L3 + (L1 + r*math.cos(q1))*math.sin(q2))*math.sin(q3)],
            [-r*math.cos(q1), 0, 0]
        ])

    def getSpiderToLegReferenceVelocities(self, spiderVelocity):
        """Calculate current reference leg velocities from current spider velocity.

        :param spiderVelocity: Current spider's velocity.
        :return: Reference legs velocities.
        """
        linearSpiderVelocity = spiderVelocity[:3]
        anguarSpiderVelocity = spiderVelocity[3:]

        # Rotate spiders reference velocity into anchors velocities which represent reference velocities for single leg.
        anchorsVelocities = [np.dot(np.linalg.inv(tAnchor[:3,:3]), linearSpiderVelocity) for tAnchor in self.spider.T_ANCHORS]
        # Add angular velocities.
        for i, anchorVelocity in enumerate(anchorsVelocities):
            anchorPosition = self.spider.LEG_ANCHORS[i]
            wx, wy, wz = anguarSpiderVelocity
            anchorsVelocities[i] = np.array([
                anchorVelocity[0],
                anchorVelocity[1] + self.spider.BODY_RADIUS * wz,
                anchorVelocity[2] - anchorPosition[0] * wy + anchorPosition[1] * wx
            ])
        refereneceLegVelocities = np.copy(anchorsVelocities) * (-1)

        return np.array(refereneceLegVelocities)

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
    
    def calculateSignedAngleBetweenTwoVectors(cls, firstVector, secondVector):
        """Calculate signed angle between two vectors in 2d.

        :param firstVector: First vector.
        :param secondVector: Second vector.
        :return: Signed angle between two vectors in radians.    
        """
        dotProduct = np.dot(firstVector, secondVector)
        productOfNorms = np.linalg.norm(firstVector) * np.linalg.norm(secondVector)
        angle = math.acos(dotProduct / productOfNorms)
        crossProduct = np.cross(firstVector, secondVector)
        # 2d vector.
        if len(firstVector) <= 2 and crossProduct < 0:
            angle = -angle
            return angle
        # 3d vector.
        if (crossProduct < 0).any() < 0:
            angle = -angle
        return angle

class MatrixCalculator:
    """ Class for calculating matrices."""
    def xyzRpyToMatrix(cls, xyzrpy):
        """Calculate global transformation matrix for transforming between global origin and spider base.

        :param globalPose: Desired global pose of a spiders platform, 1x6 array with positions and rpy orientation.
        """
        position = xyzrpy[0:3]
        rpy = xyzrpy[3:]

        roll = np.array([
            [math.cos(rpy[1]), 0, math.sin(rpy[1])],
            [0, 1, 0],
            [-math.sin(rpy[1]), 0, math.cos(rpy[1])]
        ])
        pitch = np.array([
            [1, 0, 0],
            [0, math.cos(rpy[0]), -math.sin(rpy[0])],
            [0, math.sin(rpy[0]), math.cos(rpy[0])] 
        ])
        yaw = np.array([
            [math.cos(rpy[2]), -math.sin(rpy[2]), 0],
            [math.sin(rpy[2]), math.cos(rpy[2]), 0],
            [0, 0, 1]
        ])

        rotationMatrix = np.dot(pitch, np.dot(roll, yaw))

        transformMatrix = np.c_[rotationMatrix, position]
        transformMatrix = np.r_[transformMatrix, [[0, 0, 0, 1]]]
        
        return transformMatrix
    
    def getLegsInGlobal(cls, localLegsPositions, globalPose):
        """ Calculate global positions of legs.

        :param localLegsPositions: Legs positions in leg-based origins.
        :param globalPose: Spider's position in global origin.
        :return: 5x3 array of legs positions in global origin.
        """
        legs = []
        for idx, t in enumerate(env.Spider().T_ANCHORS):
            T_GS = cls.xyzRpyToMatrix(globalPose)
            anchorInGlobal = np.dot(T_GS, t)
            pinMatrix = np.array([
                [1, 0, 0, localLegsPositions[idx][0]],
                [0, 1, 0, localLegsPositions[idx][1]],
                [0, 0, 1, localLegsPositions[idx][2]],
                [0, 0, 0, 1]])
            pinInGlobal = np.dot(anchorInGlobal, pinMatrix)
            legs.append(pinInGlobal[:,3][0:3])

        return np.array(legs)
       
