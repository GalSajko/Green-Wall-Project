""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import numpy as np
from dynamixel_sdk import *
import threading
import time

import calculations
import mappers
import environment
import planning

class MotorDriver:
    """ Class for controlling Dynamixel motors.
    """
    def __init__(self, motorsIds, enableMotors = True):
        """Set motors control table addresses and initialize USB port. 

        :param motorIds: Ids of motors to use. It should be 2d array, with motors from one leg grouped together.
        :param enableMotors: If True, enable motors on initialization.
        """
        # Motor series - we are using XM series.
        self.MOTOR_SERIES = "X_SERIES"
        # Control table addresses.
        self.TORQUE_ENABLE_ADDR = 64
        self.GOAL_POSITION_ADDR = 116
        self.PRESENT_POSITION_ADDR = 132
        self.DRIVE_MODE_ADDR = 10
        self.PROFILE_ACCELERATION_ADDR = 108
        self.PROFILE_VELOCITY_ADDR = 112
        self.READ_MOVING_ADDR = 122
        self.POSITION_P_GAIN_ADDR = 84
        self.POSITION_I_GAIN_ADDR = 82
        self.POSITION_D_GAIN_ADDR = 80
        self.BAUDRATE_ADDR = 8
        # Treshold to reach before sending new goal position - equals 5 degrees.
        self.MOVING_TRESHOLD = 100
        self.BAUDRATE = 1000000
        self.PROTOCOL_VERSION = 2.0      
        self.USB_DEVICE_NAME = "/dev/ttyUSB0"
        
        self.motorsIds = np.array(motorsIds)

        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.setCompBaudRate(self.BAUDRATE)

        self.initPort()
        if enableMotors:
            self.enableMotors()

        self.kinematics = calculations.Kinematics()
        self.spider = environment.Spider()

    def initPort(self):
        """Initialize USB port and set baudrate.
        """
        if self.portHandler.openPort():
            print("Port successfully opened.")
        else:
            print("Failed to open the port.")

        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Baudrate successfully changed.")
        else:
            print("Failed to change the baudrate.")

    def enableMotors(self):
        """ Enable all of the motors from self.MOTOR_IDS, if they are connected.
        """
        motorsArray = self.motorsIds.flatten()
        for motorId in motorsArray:
            # Enable torque.
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 1)
            comm = self.commResultAndErrorReader(result, error)
            if comm:
                print("Motor %d has been successfully enabled" % motorId) 
    
    def disableMotors(self):
        """ Disable all of the motors."""
        motorsArray = self.motorsIds.flatten()
        for motorId in motorsArray:
            # Disable torque.
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
            comm = self.commResultAndErrorReader(result, error)
            if comm:
                print("Motor %d has been successfully disabled" % motorId)

    def readLegPosition(self, legIdx):
        """Read legs position, using direct kinematics.

        :param legIdx: Leg id
        :return: Position of the end effector in leg-base origin.
        """
        motorsInLeg = self.motorsIds[legIdx]

        presentPositions = []
        for motorId in motorsInLeg:
            presentPosition, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
            self.commResultAndErrorReader(result, error)
            presentPositions.append(presentPosition)
        
        # Calculate transformation matrix from base to end effector.
        motorDegrees = mappers.mapEncoderToDegrees(presentPositions)
        pose = self.kinematics.legDirectKinematics(legIdx, motorDegrees)
        # Read position from matrix
        position = pose[:,3]
        position = position[0:3]

        return position

    def moveLeg(self, legId, goalPosition):
        """Move leg on given position in leg-base origin. Meant to use for testing purposes only, 
        otherwise use moveLegs() method.

        :param legId: Leg id.
        :param goalPosition: Goal position in leg-base origin.
        """
        # Compute encoder values from given goal position of end effector.
        q1, q2, q3 = self.kinematics.legInverseKinematics(legId, goalPosition)
        motorValues = mappers.mapJointRadiansToMotorRadians([q1, q2, q3])
        encoderValues = mappers.mapMotorRadiansToEncoder(motorValues).astype(int)

        # Write goal position (goal angle) for each motor.
        for idx, motorId in enumerate(self.motorsIds[legId]):
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motorId, self.GOAL_POSITION_ADDR, encoderValues[idx])
            self.commResultAndErrorReader(result, error)

        # Moving motors to the desired position.
        while True:
            presentPositions = np.array([0, 0, 0])
            for idx, motorId in enumerate(self.motorsIds[legId]):
                # Read current motors position.
                presentPositions[idx], result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
                self.commResultAndErrorReader(result, error)

            if (abs(encoderValues - presentPositions) < self.MOVING_STATUS_TRESHOLD).all():
                break
    
    def moveLegs(self, legIds, goalPositions):
        """ Move legs in quasi-synchronous manner - note that it is still series communication.

        :param legIds: Leg ids.
        :param goalPositions: Goal positions.
        """
        if (len(legIds) != len(goalPositions)):
            print("Given parameters are not in correct shape.")
            return

        # Compute encoder values for each motor in each leg.
        legsEncoderValues = np.empty_like(goalPositions)
        for idx, legId in enumerate(legIds):
            q1, q2, q3 = self.kinematics.legInverseKinematics(legId, goalPositions[idx])
            motorValues = mappers.mapJointRadiansToMotorRadians([q1, q2, q3])
            legsEncoderValues[idx] = mappers.mapMotorRadiansToEncoder(motorValues)
        legsEncoderValues = np.array(legsEncoderValues).astype(int)

        # Write goal positions for each motor.
        for i, legId in enumerate(legIds):
            for idx, motorId in enumerate(self.motorsIds[legId]):
                result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motorId, self.GOAL_POSITION_ADDR, legsEncoderValues[i][idx])
                self.commResultAndErrorReader(result, error)
        
        allPresentPositions = np.empty_like(goalPositions)
        while True:
            for i, legId in enumerate(legIds):
                presentPositions = np.array([0, 0, 0])
                for idx, motorId in enumerate(self.motorsIds[legId]):
                    presentPositions[idx], result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
                    self.commResultAndErrorReader(result, error)
            allPresentPositions[i] = presentPositions
        
            if (abs(legsEncoderValues - allPresentPositions) < self.MOVING_STATUS_TRESHOLD).all():
                break

    def movePlatform(self, goalPose, pins):
        """Move platform on given pose in global origin. Meant to use for testing purposes only, 
        otherwise use movePlatformTrajectory() method.

        :param goalPose: Goal pose as 1x6 array with position and rpy rotation.
        :param pins: 
        """
        # Calculate required joint values.
        jointsInLegs = self.kinematics.platformInverseKinematics(goalPose, pins)
        legsEncoderValues = np.empty((5, 3))
        for legIdx, jointsInLeg in enumerate(jointsInLegs):
            motorValues = mappers.mapJointRadiansToMotorRadians(jointsInLeg)
            encoderValues = mappers.mapMotorRadiansToEncoder(motorValues)
            legsEncoderValues[legIdx] = mappers.mapMotorRadiansToEncoder(motorValues)
        legsEncoderValues = np.array(legsEncoderValues).astype(int)

        # Use all legs for moving the platform.
        legIds = [0, 1, 2, 3, 4]
        # Write goal position (goal angle) for each motor on single leg.
        for i, legId in enumerate(legIds):
            for idx, motorId in enumerate(self.motorsIds[legId]):
                result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motorId, self.GOAL_POSITION_ADDR, legsEncoderValues[i][idx])
                self.commResultAndErrorReader(result, error)
        
        allPresentPositions = np.empty((len(legIds), 3))
        while True:
            for i, legId in enumerate(legIds):
                presentPositions = np.array([0, 0, 0])
                for idx, motorId in enumerate(self.motorsIds[legId]):
                    presentPositions[idx], result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
                    self.commResultAndErrorReader(result, error)
            allPresentPositions[i] = presentPositions
        
            if (abs(legsEncoderValues - allPresentPositions) < self.MOVING_STATUS_TRESHOLD).all():
                break  

    def movePlatformTrajectory(self, trajectory, pins):
        """Move platform along given trajectory.

        :param trajectory: Trajectory.
        :param pins: Positions of used pins in global origin.
        """
        poseIdx = 0
        while True:
            if poseIdx >= len(trajectory):
                break
            # Calculate required joint values.
            jointsInLegs = self.kinematics.platformInverseKinematics(trajectory[poseIdx], pins)
            legsEncoderValues = np.empty((5, 3))
            for legIdx, jointsInLeg in enumerate(jointsInLegs):
                motorValues = mappers.mapJointRadiansToMotorRadians(jointsInLeg)
                encoderValues = mappers.mapMotorRadiansToEncoder(motorValues)
                legsEncoderValues[legIdx] = mappers.mapMotorRadiansToEncoder(motorValues)
            legsEncoderValues = np.array(legsEncoderValues).astype(int)  

            # Use all legs for moving the platform.
            legIds = [i for i in range(5)]
            # Write goal position for each motor on each leg.
            for i, legId in enumerate(legIds):
                for idx, motorId in enumerate(self.motorsIds[legId]):
                    result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motorId, self.GOAL_POSITION_ADDR, legsEncoderValues[i][idx])
                    self.commResultAndErrorReader(result, error)
            
            while True:
                allPresentPositions = np.empty((len(legIds), 3))
                for i, legId in enumerate(legIds):
                    presentPositions = np.array([0, 0, 0])
                    for idx, motorId in enumerate(self.motorsIds[legId]):
                        presentPositions[idx], result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
                        self.commResultAndErrorReader(result, error)
                allPresentPositions[i] = presentPositions

                if (abs(legsEncoderValues - allPresentPositions) < self.MOVING_TRESHOLD).all():
                    poseIdx += 1
                    break

    
    def isLegMoving(self, legId):
        """Check if leg with legId is moving or not.

        :param motorId: Leg Id.
        :return: True if leg is moving, False otherwise.
        """

        movingArray = []
        for motorId in self.motorsIds[legId]:
            isMoving, result, error = self.packetHandler.read1ByteTxRx(self.portHandler, motorId, self.READ_MOVING_ADDR)
            self.commResultAndErrorReader(result, error)
            isMoving = bool(isMoving)
            movingArray.append(isMoving) 

        return all(movingArray)

    def setVelocityProfile(self, motorId, t1, t3):
        """Write values in Profile_velocity and Profile_acceleration registers.

        :param motorId: Motor id.
        :param t1: t1 - look in dynamixel_sdk documentation about Velocity profile.
        :param t3: t2 - look in dynamixel_sd documentation about Velocity profile.
        """
        if not t1 < t3:
            print("Unvalid values.")
            return 

        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motorId, self.PROFILE_ACCELERATION_ADDR, t1)
        firstComm = self.commResultAndErrorReader(result, error)
        
        result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motorId, self.PROFILE_VELOCITY_ADDR, t3)
        secondComm = self.commResultAndErrorReader(result, error)

        if firstComm and secondComm:
            print("Motor %d velocity profile has been succesully enabled." % motorId)

    def setPositionPids(self, motorId, p, i, d):
        """Set position PIDs for single motor.

        :param motorId: Motor id
        :param p: P value.
        :param i: I value.
        :param d: D value.
        """
        result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motorId, self.POSITION_P_GAIN_ADDR, p)
        firstComm = self.commResultAndErrorReader(result, error)
        
        result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motorId, self.POSITION_I_GAIN_ADDR, i)
        secondComm = self.commResultAndErrorReader(result, error)
        
        result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motorId, self.POSITION_D_GAIN_ADDR, d)
        thirdComm = self.commResultAndErrorReader(result, error)
        
        if firstComm and secondComm and thirdComm:
            print("Motor %d PIDs have been succesully changed." % motorId)

    def setMotorBaudRate(self, motorId, baudrateBit):
        """ Set baudrate for single motor.

        :param motorId: Motor id.
        :param baudrate: Baudrate bit (look in control table).
        """
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.BAUDRATE_ADDR, baudrateBit)
        comm = self.commResultAndErrorReader(result, error)
        if comm:
            print("Motor %d baudrate has been succesully enabled." % motorId)

    def setMotorDriveMode(self, motorId, driveModeBit):
        """Set drive mode for single motor.

        :param motorId: Motor id.
        :param driveModeBit: Drive mode bit to write on drive mode address.
        """   
        result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.DRIVE_MODE_ADDR, driveModeBit)
        comm = self.commResultAndErrorReader(result, error)
        if comm:
            print("Motor %d drive mode has been succesully changed." % motorId)

    def setCompBaudRate(self, baudrate):
        self.portHandler.setBaudRate(baudrate)
    
    def readMotorSettings(self, motorId):
        """ Read motors baudrate, velocity and acceleration profile values, PIDs and drive mode and print them.

        :param motorId: Motor's id.
        :return: Baudrate bit, velocity and acceleration profile values.
        """
        # Read baudrate.
        baudrate, result, error = self.packetHandler.read1ByteTxRx(self.portHandler, motorId, self.BAUDRATE_ADDR)
        firstComm = self.commResultAndErrorReader(result, error)
        # Read velocity profile.
        velocity, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PROFILE_VELOCITY_ADDR)
        secondComm = self.commResultAndErrorReader(result, error)    
        # Read acceleration profile.
        acc, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PROFILE_ACCELERATION_ADDR)
        thirdComm = self.commResultAndErrorReader(result, error)
        # Read PIDs.
        p, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, motorId, self.POSITION_P_GAIN_ADDR)
        fourtComm = self.commResultAndErrorReader(result, error)
        i, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, motorId, self.POSITION_I_GAIN_ADDR)
        fifthComm = self.commResultAndErrorReader(result, error)
        d, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, motorId, self.POSITION_D_GAIN_ADDR)
        sixthComm = self.commResultAndErrorReader(result, error)
        # Read drive mode.
        driveMode, result, error = self.packetHandler.read1ByteTxRx(self.portHandler, motorId, self.DRIVE_MODE_ADDR)
        seventhComm = self.commResultAndErrorReader(result, error)

        # if not (firstComm or secondComm or thirdComm or fourtComm or fifthComm or sixthComm):
        print("Motor {0} baudrate: {1}, velocity: {2}, acc: {3}, p: {4}, i: {5}, d: {6}, drive mode: {7}.".format(motorId, baudrate, velocity, acc, p, i, d, driveMode))
    
    def commResultAndErrorReader(self, result, error):
        """Helper function for reading communication result and error.

        :param result: Result.
        :param error: Error.
        :return: True if everything was ok, False otherwise.
        """
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
            return False
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
            return False
        
        return True
