""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import numpy as np
from dynamixel_sdk import *
import threading

import calculations
import mappers

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
        self.MINIMUM_POSITION_VALUE = 0
        self.MAXIMUM_POSITION_VALUE = 4095
        self.MOVING_STATUS_TRESHOLD = 5
        self.READ_MOVING_ADDR = 122
        self.BAUDRATE = 57600
        self.PROTOCOL_VERSION = 2.0
        
        self.USB_DEVICE_NAME = "/dev/ttyUSB1"
        
        self.motorsIds = np.array(motorsIds)

        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize USB port connection and enable torque in motors.
        self.initPort()

        if enableMotors:
            self.enableMotors()

        self.kinematics = calculations.Kinematics()

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
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            else:
                print("Motor %d has been successfully enabled" % motorId) 
    
    def disableMotors(self):
        """ Disable all of the motors."""
        motorsArray = self.motorsIds.flatten()
        for motorId in motorsArray:
            # Disable torque.
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            else:
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
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            presentPositions.append(presentPosition)
        
        # print("ENCODER VALUES: ", presentPositions)
        # Calculate transformation matrix from base to end effector.
        motorDegrees = mappers.mapEncoderToDegrees(presentPositions)
        # print("MOTOR DEGREES: ", motorDegrees)
        pose = self.kinematics.legDirectKinematics(legIdx, motorDegrees)
        # Read position from matrix
        position = pose[:,3]
        position = position[0:3]

        return position

    def readingInThread(self, legIdx):
        while True:
            position = self.readLegPosition(legIdx)
            print(position)

    def moveLeg(self, legId, goalPosition):
        """Move leg on given position in leg-base origin.

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
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))

        # Moving motors to the desired position.
        while True:
            presentPositions = np.array([0, 0, 0])
            for idx, motorId in enumerate(self.motorsIds[legId]):
                # Read current motors position.
                presentPositions[idx], result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
                if result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(result))
                elif error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(error))

            # Check if current position is within given treshold (20 encoder ticks).
            if abs(goalPosition - presentPositions).all() < self.MOVING_STATUS_TRESHOLD:
                break

        print("Motors on desired position.")  
    
    def isLegMoving(self, legId):
        """Check if leg with legId is moving or not.

        :param motorId: Leg Id.
        :return: True if leg is moving, False otherwise.
        """

        movingArray = []
        for motorId in self.motorsIds[legId]:
            isMoving, result, error = self.packetHandler.read1ByteTxRx(self.portHandler, motorId, self.READ_MOVING_ADDR)
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            isMoving = bool(isMoving)
            movingArray.append(isMoving)
        print(movingArray)
        

        

        return all(movingArray)