""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import numpy as np
from dynamixel_sdk import *

import calculations
import mappers

class MotorDriver:
    """ Class for controlling Dynamixel motors.
    """
    def __init__(self, motorsIds):
        """Set motors control table addresses and initialize USB port. 

        :param motorIds: Ids of motors to use. It should be 2d array, with motors from one leg grouped together.
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
        
        self.USB_DEVICE_NAME = "/dev/ttyUSB0"
        
        self.motorsIds = np.array(motorsIds)

        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize USB port connection and enable torque in motors.
        self.initPort()
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
            # Enable torque.
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            else:
                print("Motor %d has been successfully disabled" % motorId)

    def readLegPosition(self, legIdx):
        motorsInLeg = self.motorsIds[legIdx]

        presentPositions = []
        for motorId in motorsInLeg:
            presentPosition, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, motorId, self.PRESENT_POSITION_ADDR)
            if result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            presentPositions.append(presentPosition)
        
        # Calculate transformation matrix from base to end effector.
        motorDegrees = mappers.mapEncoderToDegrees(presentPositions)
        pose = self.kinematics.legDirectKinematics(legIdx, motorDegrees)
        # Read position from matrix
        position = pose[:,3]
        position = position[0:3]

        return position

    def moveLeg(self, legId, goalPosition):
        
        # Compute encoder values from given goal position of end effector.
        q1, q2, q3 = self.kinematics.legInverseKinematics(legId, goalPosition)
        motorValues = mappers.mapJointRadiansToMotorRadians([q1, q2, q3])
        encoderValues = mappers.mapDegreesToEncoder(np.degrees(motorValues)).astype(int)

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