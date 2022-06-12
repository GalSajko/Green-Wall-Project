"""Module for all calculations, includes classes for kinematics, geometry and matrix calculations.
"""

import numpy as np
import math

import environment as env


class Kinematics:
    """Class for calculating spider's kinematics.
    """
    def __init__(self):
        self.spider = env.Spider()

    def legForwardKinematics(self, legIdx, jointValues):
        """ Calculate forward kinematics for spiders leg, using transformation matrices.  

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
    
    def legBaseToThirdJointForwardKinematics(self, legIdx, jointValues):
        q1, q2, _ = jointValues
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]

        return np.array([
            [math.cos(q1)*math.cos(q2), -math.cos(q1)*math.cos(q2), math.sin(q1), math.cos(q1)*(L1 + L2*math.cos(q2) + L3 * math.cos(q2))],
            [math.cos(q2)*math.sin(q1), -math.sin(q1)*math.sin(q2), -math.cos(q1), math.sin(q1)*(L1 + L2*math.cos(q2) + L3*math.sin(q2))],
            [math.sin(q2), math.cos(q2), 0, -L3*math.cos(q2) + L2*math.sin(q2)],
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
        q3 = -math.acos(np.round((r**2 - virtualSecondLink**2 - thirdLink**2) / (2 * virtualSecondLink * thirdLink), 4))
        # Angle in second joint.
        q2 = math.atan2(secondJointToEndVector[2], np.linalg.norm(secondJointToEndVector[0:2])) + \
            math.atan2(thirdLink * math.sin(q3), virtualSecondLink + thirdLink * math.cos(q3))
        
        # Add offset because 2nd and 3rd joints are not aligned.
        q2 = -(q2 - self.spider.SECOND_JOINTS_OFFSETS[legIdx])
        q3 = q3 - self.spider.SECOND_JOINTS_OFFSETS[legIdx]

        return q1, q2, q3

    def platformInverseKinematics(self, goalPose, legsGlobalPositions):
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
            anchorInGlobalPosition = anchorInGlobal[:,3][:3]

            # Vector from anchor to end of leg in global.
            anchorToPinGlobal = np.array(legsGlobalPositions[idx] - anchorInGlobalPosition)

            # Transform this vector in legs local origin - only rotate.
            rotationMatrix = anchorInGlobal[:3, :3]
            anchorToPinLocal = np.dot(np.linalg.inv(rotationMatrix), anchorToPinGlobal)

            # With inverse kinematics for single leg calculate joints values.
            q1, q2, q3 = self.legInverseKinematics(idx, anchorToPinLocal)
            joints.append(np.array([q1, q2, q3]))

        return np.array(joints)
    
    def platformForwardKinematics(self, legsIds, legsGlobalPositions, legsLocalPoses):
        """Calculate forward kinematics of platform. Use only those legs, that are in contact with pins.

        :param legsIds: Legs used in calculations.
        :param legsGlobalPositions: Global positions of used legs - positions of used pins in wall-origin.
        :param legsLocalPositions: Local poses of used legs given in spider's origin. 
        :return: Transformation matrix from wall-origin to spider.
        """

        if len(legsIds) != 3 or len(legsGlobalPositions) != 3 or len(legsLocalPoses) != 3:
            print("Use exactly three legs for calculations of forward kinematics.")
            return False
        
        legsLocalPoses = np.array(legsLocalPoses)
        legsGlobalPositions = np.array(legsGlobalPositions)
        l1, l2, l3 = legsLocalPoses
        p1, p2, _ = legsGlobalPositions

        # Compute coordinate system of a wall-plane (in spider's origin)
        l12 = l2[:,3][:3] - l1[:,3][:3]
        l13 = l3[:,3][:3] - l1[:,3][:3]
        l23 = l3[:,3][:3] - l2[:,3][:3]
        n = [
            np.cross(l12, l13) if np.cross(l12, l13)[2] >= 0.0 else np.cross(l13, l12),
            np.cross(l12, l23) if np.cross(l12, l23)[2] >= 0.0 else np.cross(l23, l12),
            np.cross(-l13, -l23) if np.cross(-l13, -l23)[2] >= 0.0 else np.cross(-l23, -l13)
        ]
        n = np.mean(n, axis = 0)
        ez = n / np.linalg.norm(n)
        ex = l12 / np.linalg.norm(l12)
        ey = np.cross(ez, ex)
        P = np.array([ex, ey, ez])

        p12 = p2 - p1
        phi = GeometryTools().calculateSignedAngleBetweenTwoVectors(p12[:2], np.array([1, 0]))

        # Rotate P for angle phi aroud z axis to align it with global origin.
        rot = np.array([
            [math.cos(phi), -math.sin(phi), 0],
            [math.sin(phi), math.cos(phi), 0],
            [0, 0, 1]
        ])
        Pglobal = np.dot(np.linalg.inv(rot), P)
        Pglobal = np.c_[Pglobal, np.zeros(3)]
        Pglobal = np.r_[Pglobal, [[0, 0, 0, 1]]]

        positions = [np.dot(Pglobal[:3,:3], leg[:,3][:3]) + np.dot(Pglobal[:3,:3], -legsGlobalPositions[idx]) for idx, leg in enumerate(legsLocalPoses)]
        Pglobal[:,3][:3] = np.mean(positions, axis = 0)

        pose = np.linalg.inv(Pglobal)

        yaw = math.atan2(Pglobal[1, 0], Pglobal[0, 0])
        pitch = math.atan2(-Pglobal[2, 0], math.sqrt(math.pow(Pglobal[2, 1], 2) + math.pow(Pglobal[2, 2], 2)))
        roll = math.atan2(Pglobal[2, 1], Pglobal[2, 2])

        x, y, z = pose[:,3][:3]
        xyzrpy = [x, y, z, roll, pitch, yaw]
        
        return xyzrpy

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

    def spiderBaseToLegTipForwardKinematics(self, legIdx, jointsValues):
        """Calculate forward kinematics from spider base to leg-tip.
        
        :param legIdx: Leg id.
        :param jointsValues: Joints values in radians.
        :return: Transformation matrix from leg-tip to spider base.
        """
        qb = legIdx * self.spider.ANGLE_BETWEEN_LEGS + math.pi / 2
        q1, q2, q3 = jointsValues
        r = self.spider.BODY_RADIUS
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]
        L4 = self.spider.LEGS[legIdx][2]

        Hb3 = np.array([
            [math.cos(q2+q3) * math.cos(q1+qb), -math.cos(q1+qb) * math.sin(q2+q3), math.sin(q1+qb), r*math.cos(qb) + math.cos(q1+qb) * (L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2))],
            [math.cos(q2+q3) * math.sin(q1+qb), -math.sin(q2+q3) * math.sin(q1+qb), -math.cos(q1+qb), L1*math.cos(qb)*math.sin(q1) + (r + L1*math.cos(q1)) * math.sin(qb) + (L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2)) * math.sin(q1+qb)],
            [math.sin(q2+q3), math.cos(q2+q3), 0, -L3*math.cos(q2) + L2*math.sin(q2) + L4*math.sin(q2+q3)],
            [0, 0, 0, 1]
        ])

        return Hb3
        

    def spiderBaseToLegTipJacobi(self, legIdx, jointValues):
        """Calculate Jacobian matrix for spiders origin - leg-tip relation.

        :param legIdx: Leg id.
        :param jointValues: Joint values in radians.
        :return: 3x3 Jacobian matrix.
        """
        qb = legIdx * self.spider.ANGLE_BETWEEN_LEGS + math.pi / 2
        q1, q2, q3 = jointValues
        r = self.spider.BODY_RADIUS
        L1 = self.spider.LEGS[legIdx][0]
        L2 = self.spider.LEGS[legIdx][1][0]
        L3 = self.spider.LEGS[legIdx][1][1]
        L4 = self.spider.LEGS[legIdx][2]

        return np.array([
            [-((L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2)) * math.sin(q1+qb)), math.cos(q1+qb) * (L3*math.cos(q2) - L2*math.sin(q2) - L4*math.sin(q2+q3)), -L4*math.cos(q1+qb) * math.sin(q2 + q3)],
            [math.cos(q1+qb) * (L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2)), math.sin(q1+qb) * (L3*math.cos(q2) - L2*math.sin(q2) - L4*math.sin(q2+q3)), -L4*math.sin(q2+q3) * math.sin(q1+qb)],
            [0, L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2), L4*math.cos(q2+q3)]
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
        """Calculate signed angle between two vectors.

        :param firstVector: First vector.
        :param secondVector: Second vector.
        :return: Signed angle between two vectors in radians.    
        """
        dotProduct = np.dot(firstVector, secondVector)
        productOfNorms = np.linalg.norm(firstVector) * np.linalg.norm(secondVector)
        angle = math.acos(np.round(dotProduct / productOfNorms, 4))
        crossProduct = np.cross(firstVector, secondVector)
        # 2d vector.
        if len(firstVector) <= 2 and crossProduct < 0:
            angle = -angle
        # 3d vector.
        elif (crossProduct < 0).any() < 0:
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
    def xyzRpyToMatrix(cls, xyzrpy):
        """Calculate global transformation matrix for transforming between global origin and spider base.

        :param xyzrpy: Desired global pose of a spiders platform, 1x6 array with positions and rpy orientation.
        :param xyzy: Wheter or not the pose is given only as a x, y, z and yaw.
        """
        if len(xyzrpy) == 4:
            xyzrpy = [xyzrpy[0], xyzrpy[1], xyzrpy[2], 0, 0, xyzrpy[3]]
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

    def getLegInLocal(cls, legId, globalLegPosition, spiderPose):
        """Calculate local position of leg from given global position.

        :param legId: Leg id.
        :param globalLegPosition: Global position of leg.
        :param spiderPose: Spider's global pose.
        :return: 1x3 array of local leg's position.
        """
        T_GS = cls.xyzRpyToMatrix(spiderPose)
        T_GA = np.dot(T_GS, env.Spider().T_ANCHORS[legId])
        globalLegPosition = np.append(globalLegPosition, 1)
        return np.dot(np.linalg.inv(T_GA), globalLegPosition)[:3]

    
    def getLegsInGlobal(cls, legsIds, localLegsPositions, spiderPose):
        """ Calculate global positions of legs from given local positions.

        :param localLegsPositions: Legs positions in leg-based origins.
        :param globalPose: Spider's position in global origin.
        :return: Array of legs positions in global origin.
        """

        if len(spiderPose) == 4:
            spiderPose = [spiderPose[0], spiderPose[1], spiderPose[2], 0.0, 0.0, spiderPose[3]]
            
        legs = []
        T_GS = cls.xyzRpyToMatrix(spiderPose)
        for idx, leg in enumerate(legsIds):
            anchorInGlobal = np.dot(T_GS, env.Spider().T_ANCHORS[leg])
            legInGlobal = np.dot(anchorInGlobal, np.append(localLegsPositions[idx], 1))
            legs.append(legInGlobal[:3])
        return np.array(legs)
    
    def getLegsApproachPositionsInGlobal(cls, legsIds, spiderPose, pinsPositions, offset = 0.03):
        """ Calculate approach point for leg-to-pin movement, so that gripper would fit on pin.

        :param legId: Leg id.
        :param spiderPose: Spider's pose in global.
        :param pinPosition: Pin position in global.
        :param offset: Distance from pin, defaults to 0.02.
        :raises ValueError: If length of legsIds and pinsPositions parameters are not the same.
        :return: Position of approach point in global.
        """
        if len(legsIds) != len(pinsPositions):
            raise ValueError("Invalid values of legsIds or pinsPositions parameters.")
        if len(spiderPose) == 4:
            spiderPose = [spiderPose[0], spiderPose[1], spiderPose[2], 0.0, 0.0, spiderPose[3]]
        
        approachPointsInGlobal = []
        for idx, leg in enumerate(legsIds):
            jointsValues = Kinematics().legInverseKinematics(leg, cls.getLegInLocal(leg, pinsPositions[idx], spiderPose))
            T_GA = np.dot(cls.xyzRpyToMatrix(spiderPose), env.Spider().T_ANCHORS[leg])
            thirdJointLocalPosition = Kinematics().legBaseToThirdJointForwardKinematics(leg, jointsValues)[:,3][:3]
            thirdJointGlobalPosition = np.dot(T_GA, np.append(thirdJointLocalPosition, 1))[:3]

            pinToThirdJoint = thirdJointGlobalPosition - pinsPositions[idx]
            pinToThirdJoint = (pinToThirdJoint / np.linalg.norm(pinToThirdJoint)) * offset
            approachPointsInGlobal.append(pinsPositions[idx] + pinToThirdJoint)

        return np.array(approachPointsInGlobal)
