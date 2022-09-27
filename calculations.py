"""Module for all calculations, includes classes for kinematics, geometry and matrix calculations.
"""

import numpy as np
import math
import itertools as itt

import environment as env


class Kinematics:
    """Class for calculating spider's kinematics.
    """
    def __init__(self):
        self.spider = env.Spider()

    def legForwardKinematics(self, legId, jointsValues):
        """Calculate forward kinematics for spiders leg, using transformation matrices.  

        :param jointValues: Joint values in radians.
        :return: Transformation matrix from base to end effector.

        Args:
            legId (int):Leg id.
            jointsValues (list): Joints values in radians.

        Returns:
            numpy.ndarray: 4x4 transformation matrix from base to end effector.
        """
        q1, q2, q3 = jointsValues
        L1 = self.spider.LEGS_DIMENSIONS[legId][0]
        L2 = self.spider.LEGS_DIMENSIONS[legId][1]
        L3 = self.spider.LEGS_DIMENSIONS[legId][2]

        return np.array([
            [math.cos(q1) * math.cos(q2 + q3), -math.cos(q1) * math.sin(q2 + q3), math.sin(q1), math.cos(q1) * (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3))],
            [math.cos(q2 + q3) * math.sin(q1), -math.sin(q1) * math.sin(q2 + q3), -math.cos(q1), (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1)],
            [math.sin(q2 + q3), math.cos(q2 + q3), 0, L2 * math.sin(q2) + L3 * math.sin(q2 + q3)],
            [0, 0, 0, 1]
        ])
    
    def legBaseToThirdJointForwardKinematics(self, legId, jointValues):
        """Calculate forward kinematics from leg base to third joint.

        Args:
            legId (int): Leg id.
            jointValues (list): Joints values in radians.

        Returns:
            numpy.ndarray: 4x4 transformation matrix from leg-base to third joint.
        """
        q1, q2, _ = jointValues
        L1 = self.spider.LEGS_DIMENSIONS[legId][0]
        L2 = self.spider.LEGS_DIMENSIONS[legId][1]

        return np.array([
            [math.cos(q1) * math.cos(q2), -math.cos(q1) * math.sin(q2), math.sin(q1), math.cos(q1) * (L1 + L2 * math.cos(q2))],
            [math.cos(q2) * math.sin(q1), -math.sin(q1) * math.sin(q2), -math.cos(q1), (L1 + L2 * math.cos(q2)) * math.sin(q1)],
            [math.sin(q2), math.cos(q2), 0, L2 * math.sin(q2)],
            [0, 0, 0, 1]
        ])

    def legInverseKinematics(self, legId, endEffectorPosition):
        """Calculate inverse kinematics for leg, using geometry.

        Args:
            legId (int): Leg id.
            endEffectorPosition (list): Desired position of end effector in leg-base origin.

        Returns:
            tuple: Angles in radians in first, second and third joint.
        """
        endEffectorPosition = np.array(endEffectorPosition)

        L1 = self.spider.LEGS_DIMENSIONS[legId][0]
        L2 = self.spider.LEGS_DIMENSIONS[legId][1]
        L3 = self.spider.LEGS_DIMENSIONS[legId][2]

        # Angle in first joint.
        q1 = math.atan2(endEffectorPosition[1], endEffectorPosition[0])

        # Position of second joint in leg-base origin.
        secondJointPosition = np.array([
            L1 * math.cos(q1),
            L1 * math.sin(q1),
            0])

        # Vector from second joint to end effector.
        secondJointToEndVector = np.array(endEffectorPosition - secondJointPosition)

        # Distance between second joint and end effector.
        r = np.linalg.norm(secondJointToEndVector)
        # Angle in third joint, note Pi - acos(x) = acos(-x).
        q3 =  -math.acos(np.round((r**2 - L2**2 - L3**2) / (2 * L2 * L3), 4))
        # Angle in second joint.
        alpha = abs(math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3)))
        gamma = math.atan2(secondJointToEndVector[2], np.linalg.norm(secondJointToEndVector[0:2]))
        q2 = alpha + gamma

        return q1, q2, q3

    def platformInverseKinematics(self, legsIds, legsGlobalPositions, goalPose):
        """Calculate inverse kinematics for spider platform.

        Args:
            legsIds (list): Ids of used legs.
            legsGlobalPositions (list): Global positions of used legs.
            goalPose (list): Goal pose in global, given as 1x6 array of xyzrpy values.

        Raises:
            ValueError: If length of legsIds and legsGlobalPositions are not the same.

        Returns:
            numpy.ndarray: nx3 matrix with joints values for all used legs, where n is number of used legs.
        """
        if len(legsIds) != len(legsGlobalPositions):
            raise ValueError("Lengths of legsIds and legsGlobalPositions do not match.")

        # Get transformation matrix from spiders xyzrpy.
        globalTransformMatrix = TransformationCalculator().xyzRpyToMatrix(goalPose)

        # Array to store calculated joints values for all legs.
        joints = np.zeros([len(legsIds, self.spider.NUMBER_OF_MOTORS_IN_LEG)])
        for idx, leg in enumerate(legsIds):
            # Pose of leg anchor in global
            anchorInGlobal = np.dot(globalTransformMatrix, self.spider.T_ANCHORS[leg])
            # Position of leg anchor in global.
            anchorInGlobalPosition = anchorInGlobal[:,3][:3]

            # Vector from anchor to end of leg in global.
            anchorToPinGlobal = np.array(legsGlobalPositions[idx] - anchorInGlobalPosition)

            # Transform this vector in legs local origin - only rotate.
            rotationMatrix = anchorInGlobal[:3, :3]
            anchorToPinLocal = np.dot(np.linalg.inv(rotationMatrix), anchorToPinGlobal)

            # With inverse kinematics for single leg calculate joints values.
            q1, q2, q3 = self.legInverseKinematics(leg, anchorToPinLocal)
            joints[idx] = np.array([q1, q2, q3])

        return joints

    def platformForwardKinematics(self, legsIds, legsGlobalPositions, legsLocalPoses):
        """Calculate forward kinematics of platform. Use only those legs, that are in contact with pins.

        Args:
            legsIds (list): Ids of used legs.
            legsGlobalPositions (list): Global positions of used legs.
            legsLocalPoses (list):4x4 transformation matrices, representing a legs poses in legs-local origins.

        Returns:
            list: 1x6 list with global xyzrpy values.
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
        phi = MathTools().calculateSignedAngleBetweenTwoVectors(p12[:2], np.array([1, 0]))

        # Rotate P for angle phi aroud z axis to align it with global origin.
        rot = np.array([
            [math.cos(phi), -math.sin(phi), 0],
            [math.sin(phi), math.cos(phi), 0],
            [0, 0, 1]
        ])
        Pglobal = np.dot(np.linalg.inv(rot), P)
        Pglobal = np.c_[Pglobal, np.zeros(3)]
        Pglobal = np.r_[Pglobal, [[0, 0, 0, 1]]]

        positions = np.zeros([len(legsLocalPoses, 3)])
        for idx, leg in enumerate(legsLocalPoses):
            positions[idx] = np.dot(Pglobal[:3,:3], leg[:,3][:3]) + np.dot(Pglobal[:3,:3], -legsGlobalPositions[idx])
        Pglobal[:,3][:3] = np.mean(positions, axis = 0)

        pose = np.linalg.inv(Pglobal)

        yaw = math.atan2(Pglobal[1, 0], Pglobal[0, 0])
        pitch = math.atan2(-Pglobal[2, 0], math.sqrt(math.pow(Pglobal[2, 1], 2) + math.pow(Pglobal[2, 2], 2)))
        roll = math.atan2(Pglobal[2, 1], Pglobal[2, 2])

        x, y, z = pose[:,3][:3]
        xyzrpy = [x, y, z, roll, pitch, yaw]
        
        return xyzrpy

    def legJacobi(self, legId, jointValues):
        """Calculate Jacobian matrix for given leg.

        Args:
            legId (int): Leg id.
            jointValues (list): Joints values in radians.

        Returns:
            numpy.ndarray: 3x3 Jacobian matrix.
        """
        q1, q2, q3 = jointValues
        L1 = self.spider.LEGS_DIMENSIONS[legId][0]
        L2 = self.spider.LEGS_DIMENSIONS[legId][1]
        L3 = self.spider.LEGS_DIMENSIONS[legId][2]

        return np.array([
            [-(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1), -math.cos(q1) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.cos(q1) * math.sin(q2 + q3)],
            [math.cos(q1) * (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)), -math.sin(q1) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.sin(q1) * math.sin(q2 + q3)],
            [0, L2 * math.cos(q2) + L3 * math.cos(q2 + q3), L3 * math.cos(q2 + q3)]
            ])

    def spiderBaseToLegTipForwardKinematics(self, legId, jointsValues):
        """Calculate forward kinematics from spider base to leg-tip.

        Args:
            legId (int): Leg id.
            jointsValues (list): Joints values in radians.

        Returns:
            numpy.ndarray: 4x4 transformation matrix from spider base to leg-tip.
        """
        qb = legId * self.spider.ANGLE_BETWEEN_LEGS + math.pi / 2
        q1, q2, q3 = jointsValues
        r = self.spider.BODY_RADIUS
        L1 = self.spider.LEGS_DIMENSIONS[legId][0]
        L2 = self.spider.LEGS_DIMENSIONS[legId][1]
        L3 = self.spider.LEGS_DIMENSIONS[legId][2]

        Hb3 = np.array([
            [math.cos(q2 + q3) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2 + q3), math.sin(q1 + qb), r * math.cos(qb) + (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.cos(q1 + qb)],
            [math.cos(q2 + q3) * math.sin(q1 + qb), -math.sin(q2 + q3) * math.sin(q1 + qb), -math.cos(q1 + qb), r * math.sin(qb) + (L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1 + qb)],
            [math.sin(q2 + q3), math.cos(q2 + q3), 0, L2 * math.sin(q2) + L3 * math.sin(q2 + q3)],
            [0, 0, 0, 1]
        ])

        return Hb3

    def spiderBaseToLegTipJacobi(self, legId, jointValues):
        """Calculate Jacobian matrix for spider's origin - leg-tip relation.

        Args:
            legId (int): Leg id.
            jointValues (list): Joints values in radians.

        Returns:
            numpy.ndarray: 3x3 Jacobian matrix.
        """
        qb = legId * self.spider.ANGLE_BETWEEN_LEGS + math.pi / 2
        q1, q2, q3 = jointValues
        r = self.spider.BODY_RADIUS
        L1 = self.spider.LEGS_DIMENSIONS[legId][0]
        L2 = self.spider.LEGS_DIMENSIONS[legId][1]
        L3 = self.spider.LEGS_DIMENSIONS[legId][2]

        return np.array([
            [-(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.sin(q1 + qb), -math.cos(q1 + qb) * (L2 * math.sin(q2) + L3 * math.sin(q2 + q3)), -L3 * math.cos(q1 + qb) * math.sin(q2 + q3)],
            [(L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)) * math.cos(q1 + qb), -(L2 * math.sin(q2) + L3 * math.sin(q2 + q3)) * math.sin(q1 + qb), -L3 * math.sin(q2 + q3) * math.sin(q1 + qb)],
            [0, L2 * math.cos(q2) + L3 * math.cos(q2 + q3), L3 * math.cos(q2 + q3)]
        ])

    def getSpiderToLegReferenceVelocities(self, legsIds, spiderVelocity):
        """Calculate needed legs velocities to match given spider velocity.

        Args:
            legsIds (list): Ids of used legs.
            spiderVelocity (list): 1x6 list with velocities in global xyzrpy directions.

        Returns:
            numpy.ndarray: nx3 array of reference legs velocities in leg-based origin, where n is number of used legs.
        """
        
        linearSpiderVelocity = spiderVelocity[:3]
        anguarSpiderVelocity = spiderVelocity[3:]

        # Rotate spiders reference velocity into anchors velocities which represent reference velocities for single leg.
        anchorsVelocities = [np.dot(np.linalg.inv(self.spider.T_ANCHORS[leg][:3,:3]), linearSpiderVelocity) for leg in legsIds]
        # Add angular velocities.
        for idx, leg in enumerate(legsIds):
            anchorPosition = self.spider.LEG_ANCHORS[leg]
            wx, wy, wz = anguarSpiderVelocity
            anchorsVelocities[idx] = np.array([
                anchorsVelocities[0],
                anchorsVelocities[1] + self.spider.BODY_RADIUS * wz,
                anchorsVelocities[2] - anchorPosition[0] * wy + anchorPosition[1] * wx
            ])
        refereneceLegVelocities = np.copy(anchorsVelocities) * (-1)

        return np.array(refereneceLegVelocities)
    
class Dynamics:
    """Class for calculating spider's dynamics.
    """
    def __init__(self):
        self.spider = env.Spider()
        self.kinematics = Kinematics()

    def getForceOnLegTip(self, legId, jointsValues, currentsInMotors):
        """Calculate force, applied to leg-tip, from currents in motors.

        Args:
            legId (int): Leg id.
            jointsValues (list): 1x3 array of leg's joints values in radians.
            currentsInMotors (list): Currents in motors in Ampers.

        Returns:
            list: 3x1 array of forces in x, y and z direction.
        """
        currentsInMotors = np.array(currentsInMotors)
        currentsInMotors[1] *= -1
        J = self.kinematics.spiderBaseToLegTipJacobi(legId, jointsValues)
        # Parabola fitting constants (derived from least squares method).
        a = -0.3282
        b = 2.9326
        c = -0.1779
        torques = a + b * currentsInMotors + c * currentsInMotors**2
        
        return np.dot(np.linalg.inv(np.transpose(J)), torques)
    
    def getForcesOnLegsTips(self, jointsValues, currentsInMotors, spiderGravityVector):
        """Calculate forces, applied to tips of all legs, from currents in motors.

        Args:
            jointsValues (list): 5x3 array of angles in joints.
            currentsInMotors (list): 5x3 array of currents in motors.
            spiderGravityVector(list): 1x3 gravity vector in spider's origin.

        Returns:
            numpy.ndarray: 5x3 array of forces, applied to leg tips in x, y, z direction of spider's origin.
        """
        currentsInMotors = np.array(currentsInMotors)
        currentsInMotors[:, 1] *= -1
        # Torque(current) parabola fitting constants (derived from least squares method).
        a = -0.3282
        b = 2.9326
        c = -0.1779

        gravityTorques = self.__getGravityCompensationTorques(jointsValues, spiderGravityVector)
        torques = (a + b * currentsInMotors + c * currentsInMotors**2)

        forces = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        for legId, jointsInLeg in enumerate(jointsValues):
            J = self.kinematics.spiderBaseToLegTipJacobi(legId, jointsInLeg)
            forces[legId] = np.dot(np.linalg.inv(np.transpose(J)), torques[legId])
        
        return forces, torques
    
    def getForceEllipsoidLengthInGivenDirection(self, jointsValues, direction):
        """Calculate size of vector from center to the surface of force manipulability ellipsoid, in given direction.

        Args:
            legId (int): Leg's id.
            jointsValues (list): 1x3 array of leg's joints values in radians.
            direction (list): 1x3 vector of direction in spider's origin.

        Returns:
            float: Length of vector from center to the surface of force ellipsoid in given direction.
        """
        J = self.kinematics.spiderBaseToLegTipJacobi(legId, jointsValues)
        A = np.linalg.inv(np.dot(J, np.transpose(J)))
        eigVals, eigVects = np.linalg.eig(A)

        # Unit vector in given direction in ellipsoid origin.
        eGrav = np.dot(eigVects, direction[0])
        ez = np.dot(eigVects, direction[1])

        ellipsoidSizeInGlobalNegY = math.sqrt(np.prod(eigVals) / (math.pow(eGrav[0], 2) * eigVals[1] * eigVals[2] + math.pow(eGrav[1], 2) * eigVals[0] * eigVals[2] + math.pow(eGrav[2], 2) * eigVals[0] * eigVals[1]))
        ellipsoidSizeInGlobalZ = math.sqrt(np.prod(eigVals) / (math.pow(ez[0], 2) * eigVals[1] * eigVals[2] + math.pow(ez[1], 2) * eigVals[0] * eigVals[2] + math.pow(ez[2], 2) * eigVals[0] * eigVals[1]))

        ed = np.array([0, -ellipsoidSizeInGlobalNegY, ellipsoidSizeInGlobalZ])
        ed = ed / np.linalg.norm(ed)
        ed = np.dot(np.linalg.inv(eigVects), ed)
        t = math.sqrt(np.prod(eigVals) / (math.pow(ed[0], 2) * eigVals[1] * eigVals[2] + math.pow(ed[1], 2) * eigVals[0] * eigVals[2] + math.pow(ed[2], 2) * eigVals[0] * eigVals[1]))

        return t
    
    def __getGravityCompensationTorques(self, jointsValues, spiderGravityVector):
        """Calculate torques in joints (for all legs), required to compensate movement, caused only by gravity.

        Args:
            jointsValues (list): 5x3 array of angles in joints, in radians.
            spiderGravityVector (list): 1x3 gravity vector in spider's origin.

        Returns:
            numpy.ndarray: 5x3 array of required torques in joints.
        """
        torques = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        for legId, jointsInLeg in enumerate(jointsValues): 
            qb = legId * self.spider.ANGLE_BETWEEN_LEGS + math.pi / 2.0
            gravityRotationMatrices = self.__getGravityRotationMatrices(legId, jointsInLeg, qb)
            forceRotationMatrices = self.__getForceRotationMatrices(jointsInLeg)

            # Calculate gravity vectors in segments' origins (from first to last segment).
            localGravityVectors = np.zeros([3, 3])
            for i in range(3):
                localGravityVectors[i] = np.dot(np.transpose(gravityRotationMatrices[i]), spiderGravityVector)

            # Calculate forces and torques (from last to first segment).
            torquesVectorsInLeg = np.zeros([3, 3])
            torquesValuesInLeg = np.zeros(3)
            forces = np.zeros([3, 3])
            for i in range(3):
                lc = np.array([1, 0, 0]) * self.spider.VECTORS_TO_COG_SEGMENT[legId][2 - i]
                l = np.array([1, 0, 0]) * self.spider.LEGS_DIMENSIONS[legId][2 - i]
                
                if i != 0:
                    fgSegment = self.spider.SEGMENTS_MASSES[legId][2 - i] * localGravityVectors[2 - i]
                    forces[i] = np.dot(forceRotationMatrices[i - 1], forces[i - 1]) - fgSegment
                    torquesVectorsInLeg[i] = np.dot(forceRotationMatrices[i - 1], torquesVectorsInLeg[i - 1]) + np.cross(fgSegment, lc) - np.cross(np.dot(forceRotationMatrices[i - 1], forces[i - 1]), l)
                    torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
                    continue

                forces[i] = -self.spider.SEGMENTS_MASSES[legId][2 - i] * localGravityVectors[2 - i]
                torquesVectorsInLeg[i] = (-1) * np.cross(forces[i], lc)
                torquesValuesInLeg[i] = torquesVectorsInLeg[i][2]
            # Flip array, so first value in array will correspond with first joint and so on.
            torques[legId] = np.flip(torquesValuesInLeg)

        return torques 
    
    def __getGravityRotationMatrices(self, legId, jointsValues, qb):
        q1, q2, q3 = jointsValues
        return [
            TransformationCalculator().R_B1(qb, q1),
            TransformationCalculator().R_B2(qb, q1, q2),
            TransformationCalculator().R_B3(qb, q1, q2, q3)]
    
    def __getForceRotationMatrices(self, jointsValues):
        return [TransformationCalculator().R_23(jointsValues[2]), TransformationCalculator().R_12(jointsValues[1])]


class MathTools:
    """Helper class for geometry calculations.
    """
    def calculateSignedAngleBetweenTwoVectors(cls, firstVector, secondVector):
        """Calculate signed angle between two vectors.

        Args:
            firstVector (list): First vector.
            secondVector (list): Second vector.

        Returns:
            float: Signed angle in radians.
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

        Args:
            angle (float): Angle.

        Returns:
            float: Wrapped angle.
        """
        if angle < -math.pi:
            angle += math.pi * 2
        elif angle > math.pi:
            angle -= math.pi * 2

        return angle
    
    def runningAverage(cls, buffer, counter, newValue):
        """Calculate running average of values in buffer.

        Args:
            buffer (list): Values, used to calculate average.
            counter (int): Index at which new values are writen in buffer, if it is not already full.
            newValue (list): New values, which will be written in the buffer.

        Returns:
            tuple: Average of the buffer (element wise), shifted buffer and updated counter.
        """
        if counter < len(buffer):
            buffer[counter] = newValue
            counter += 1
            average = np.mean(buffer[:counter], axis = 0)

            return average, buffer, counter
        
        buffer = np.roll(buffer, -1, axis = 0)
        buffer[-1] = newValue
        average = np.mean(buffer, axis = 0)

        return average, buffer, counter

class TransformationCalculator:
    """ Class for calculating matrices."""
    def xyzRpyToMatrix(cls, xyzrpy):
        """Calculate global transformation matrix for global origin - spider relation.

        Args:
            xyzrpy (list): Global spider's pose to be transformed into matrix. Could be given as 1x4 array, representing xyzy values or 
            1x6 array, representing xyzrpy values.

        Returns:
            numpy.ndarray: 4x4 transformation matrix from global origin to spider.
        """
        if len(xyzrpy) == 4:
            xyzrpy = [xyzrpy[0], xyzrpy[1], xyzrpy[2], 0, 0, xyzrpy[3]]
        position = xyzrpy[0:3]
        rpy = xyzrpy[3:]

        roll = np.array([
            [math.cos(rpy[1]), 0, math.sin(rpy[1])],
            [0, 1, 0],
            [-math.sin(rpy[1]), 0, math.cos(rpy[1])]
        ], dtype = np.float32)
        pitch = np.array([
            [1, 0, 0],
            [0, math.cos(rpy[0]), -math.sin(rpy[0])],
            [0, math.sin(rpy[0]), math.cos(rpy[0])]
        ], dtype = np.float32)
        yaw = np.array([
            [math.cos(rpy[2]), -math.sin(rpy[2]), 0],
            [math.sin(rpy[2]), math.cos(rpy[2]), 0],
            [0, 0, 1]
        ], dtype = np.float32)

        rotationMatrix = np.dot(pitch, np.dot(roll, yaw))

        transformMatrix = np.c_[rotationMatrix, position]
        addRow = np.array([0, 0, 0, 1], dtype = np.float32)
        transformMatrix = np.r_[transformMatrix, [addRow]]
        
        return transformMatrix

    def getLegInLocal(cls, legId, globalLegPosition, spiderPose):
        """Calculate local leg's position from given global position.

        Args:
            legId (int): Leg id.
            globalLegPosition (list): Global position of leg.
            spiderPose (list): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.

        Returns:
            numpy.ndarray: 1x3 array of with x, y and z leg's positions in leg-local origin.
        """
        T_GS = cls.xyzRpyToMatrix(spiderPose)
        T_GA = np.dot(T_GS, env.Spider().T_ANCHORS[legId])
        globalLegPosition = np.append(globalLegPosition, 1)

        return np.dot(np.linalg.inv(T_GA), globalLegPosition)[:3]
    
    def getGlobalDirectionInLocal(cls, legId, spiderPose, globalDirection):
        T_GS = cls.xyzRpyToMatrix(spiderPose)
        T_GA = np.dot(T_GS, env.Spider().T_ANCHORS[legId])[:3,:3]
        localDirection = np.dot(np.linalg.inv(T_GA), globalDirection)

        return localDirection

    def getLegsInGlobal(cls, legsIds, localLegsPositions, spiderPose):
        """Calculate global positions of legs from given local positions.

        Args:
            legsIds (list): Legs ids.
            localLegsPositions (list): nx3 array of legs positions in their local origins, where n should be same as length of legsIds list.
            spiderPose (list): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.

        Returns:
            numpy.ndarray: nx3 array of legs positions in global origin, where n is number of given legs.
        """ 
        legsGlobalPositions = np.empty([len(legsIds), 3], dtype = np.float32)
        T_GS = cls.xyzRpyToMatrix(spiderPose)
        for idx, leg in enumerate(legsIds):
            anchorInGlobal = np.dot(T_GS, env.Spider().T_ANCHORS[leg])
            legInGlobal = np.dot(anchorInGlobal, np.append(localLegsPositions[idx], 1))
            legsGlobalPositions[idx] = legInGlobal[:3]

        return legsGlobalPositions

    def getLegsApproachPositionsInGlobal(cls, legsIds, spiderPose, globalPinsPositions, offset = 0.03):
        """Calculate approach point in global origin, so that gripper would fit on pin.

        Args:
            legsIds (list): Ids of used legs.
            spiderPose (list): Global spider's pose. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.
            globalPinsPositions (list): nx3 array of used pins positions in global origin, where n should be same as number of used legs.
            offset (float, optional): Distance from pin to the approach point. Defaults to 0.03.

        Raises:
            ValueError: If lengths of legsIds and globalPinsPositions parameters are not the same.

        Returns:
            numpy.ndarray: nx3 array of approach positions in global origin, where n is number of used legs.
        """
        if len(legsIds) != len(globalPinsPositions):
            raise ValueError("Invalid values of legsIds or pinsPositions parameters.")
        
        approachPointsInGlobal = np.empty([len(legsIds), 3])
        for idx, leg in enumerate(legsIds):
            jointsValues = Kinematics().legInverseKinematics(leg, cls.getLegInLocal(leg, globalPinsPositions[idx], spiderPose))
            T_GA = np.dot(cls.xyzRpyToMatrix(spiderPose), env.Spider().T_ANCHORS[leg])
            thirdJointLocalPosition = Kinematics().legBaseToThirdJointForwardKinematics(leg, jointsValues)[:,3][:3]
            thirdJointGlobalPosition = np.dot(T_GA, np.append(thirdJointLocalPosition, 1))[:3]

            pinToThirdJoint = thirdJointGlobalPosition - globalPinsPositions[idx]
            pinToThirdJoint = (pinToThirdJoint / np.linalg.norm(pinToThirdJoint)) * offset
            approachPointsInGlobal[idx] = globalPinsPositions[idx] + pinToThirdJoint

        return approachPointsInGlobal

    def getSpiderPose(self, legsIds, legsGlobalPositions, qA):
        """Calculate spider's pose in global origin. If more than three legs are given, it calculates spider's pose from each
        combination of these three legs. Finally pose is determined as mean value of all calculations.

        Args:
            legsIds (list): Ids of legs to use for calculating spider's pose. Should not use less than three legs.
            legsGlobalPositions (list): nx3 array of x, y, z positions of used legs in global origin, where n should be the same as length of legsIds.

        Returns:
            list: 1x6 array of xyzrpy pose.
        """
        legsGlobalPositions = np.array(legsGlobalPositions)
        poses = []
        for legsSubset in itt.combinations(legsIds, 3):
            legsSubset = np.array(legsSubset)
            subsetIdxs = [legsIds.index(leg) for leg in legsSubset]
            jointsValues = qA[legsSubset]
            legsPoses = np.zeros([3, 4, 4])
            for idx, leg in enumerate(legsSubset):
                legsPoses[idx] = Kinematics().spiderBaseToLegTipForwardKinematics(leg, jointsValues[idx])
            poses.append(Kinematics().platformForwardKinematics(legsSubset, legsGlobalPositions[(subsetIdxs)], legsPoses))
        pose = np.mean(np.array(poses), axis = 0)

        return pose

    def R_B1(cls, qb, q1):
        """Rotation matrix from spider's to 1st segment's origin.

        Args:
            qb (float): Angle from spider's origin to leg-base origin, in radians. 
            q1 (float): Angle in first joint, in radians.

        Returns:
            numpy.ndarray: 3x3 rotation matrix.
        """
        return np.array([
            [math.cos(q1 + qb), -math.sin(q1 + qb), 0],
            [math.sin(q1 + qb), math.cos(q1 + qb), 0],
            [0, 0, 1]
        ])
    
    def R_12(cls, q2):
        """Rotation matrix from 1st to 2nd leg-segment.

        Args:
            q2 (float): Angle in second joint, in radians.

        Returns:
            numpy.ndarray: 3x3 rotation matrix.
        """
        return np.array([
            [math.cos(q2), -math.sin(q2), 0],
            [0, 0, -1],
            [math.sin(q2), math.cos(q2), 0]
        ])
    
    def R_23(cls, q3):
        """Rotation matrix from 2nd to 3rd leg-segment.

        Args:
            q3 (float): Angle in third joint, in radians.

        Returns:
            numpy.ndarray: 3x3 rotation matrix.
        """
        return np.array([
            [math.cos(q3), -math.sin(q3), 0],
            [math.sin(q3), math.cos(q3), 0],
            [0, 0, 1]
        ])
    
    def R_B2(cls, qb, q1, q2):
        """Rotation matrix from spider's to 2nd segment's origin.

        Args:
            qb (float): Angle from spider's origin to leg-base origin, in radians.
            q1 (float): Angle in first joint, in radians.
            q2 (float): Angle in second joint, in radians.

        Returns:
            numpy.ndarray: 3x3 rotation matrix.
        """
        return np.array([
            [math.cos(q2) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2), math.sin(q1 + qb)],
            [math.cos(q2) * math.sin(q1 + qb), -math.sin(q2) * math.sin(q1 + qb), -math.cos(q1 + qb)],
            [math.sin(q2), math.cos(q2), 0]
        ])
    
    def R_B3(cls, qb, q1, q2, q3):
        return np.array([
            [math.cos(q2 + q3) * math.cos(q1 + qb), -math.cos(q1 + qb) * math.sin(q2 + q3), math.sin(q1 + qb)],
            [math.cos(q2 + q3) * math.sin(q1 + qb), -math.sin(q2 + q3) * math.sin(q1 + qb), -math.cos(q1 + qb)],
            [math.sin(q2 + q3), math.cos(q2 + q3), 0],
        ])
