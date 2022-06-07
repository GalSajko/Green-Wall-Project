""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import numpy as np
from dynamixel_sdk import *

import calculations
import mappers
import environment

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
        self.GOAL_VELOCITY_ADDR = 104
        self.PRESENT_POSITION_ADDR = 132
        self.BAUDRATE = 2000000
        self.PROTOCOL_VERSION = 2.0      
        self.USB_DEVICE_NAME = "/dev/ttyUSB0"
        self.PRESENT_POSITION_DATA_LENGTH = 4
        self.GOAL_VELOCITY_DATA_LENGTH = 4
        
        self.motorsIds = np.array(motorsIds)
        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Group sync positions reader and group sync velocity writer.
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.GOAL_VELOCITY_ADDR, self.GOAL_VELOCITY_DATA_LENGTH)

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
    
    def disableLegs(self, legId = 5):
        """ Disable all of the motors if value of motors parameter is 5. Otherwise, disable motors in given leg."""
        motorsArray = self.motorsIds.flatten()
        if legId == 5:
            for motorId in motorsArray:
                # Disable torque.
                result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
                comm = self.commResultAndErrorReader(result, error)
                if comm:
                    print("Motor %d has been successfully disabled" % motorId)
            return

        for motorId in self.motorsIds[legId]:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
            comm = self.commResultAndErrorReader(result, error)
            if comm:
                print("Motor %d has been successfully disabled" % motorId)

    def addGroupSyncReadParams(self, legIdx):
        """Add parameters (motors ids of legs) to group sync reader parameters storage.

        :param legIdx: Array of leg ids. 
        """
        motors = np.array(self.motorsIds[legIdx]).flatten()
        for motor in motors:
            result = self.groupSyncRead.addParam(motor)
            if not result:
                print("Failed adding parameter %d to Group Sync Reader" % motor)
                return False
        return True
    
    def syncReadMotorsPositionsInLegs(self, legsIds, calculateLegPositions = False):
        """Read motors positions in given legs with sync reader.

        :param legsIds: Legs ids.
        :return: nx3 matrix with motors positions where n is number of given legs.
        """
        self.groupSyncRead.txRxPacket()
        mappedPositions = []
        for leg in legsIds:
            positions = [self.groupSyncRead.getData(motorInLeg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH) for motorInLeg in self.motorsIds[leg]]
            jointsValues = mappers.mapEncoderToJointRadians(positions)
            if not calculateLegPositions:
                mappedPositions.append(jointsValues)
            else:
                mappedPositions.append(self.kinematics.legDirectKinematics(leg, jointsValues)[:,3][:3])

        return np.array(mappedPositions)

    def syncWriteMotorsVelocitiesInLegs(self, legIdx, qCd, add = True):
        """Write motors velocities to given motors in legs with sync writer.

        :param legIdx: Legs ids.
        :param qCd: Reference velocities.
        :param add: If true add parameters to storage, otherwise only change them, defaults to True
        :return: True if writing was successfull, false otherwise.
        """
        
        for idx, leg in enumerate(legIdx):
            motorsInLeg = self.motorsIds[leg]
            encoderVelocoties = mappers.mapJointVelocitiesToEncoderValues(qCd[idx]).astype(int)
            for i, motor in enumerate(motorsInLeg):
                qCdBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocoties[i]))]
                # Add or change parameters in storage.
                if add:
                    result = self.groupSyncWrite.addParam(motor, qCdBytes)
                    if not result:
                        print("Failed adding parameter %d to Group Sync Writer" % motor)
                        return False
                else:
                    result = self.groupSyncWrite.changeParam(motor, qCdBytes)
                    if not result:
                        print("Failed changing Group Sync Writer parameter %d" % motor)
                        return False   
        # Write velocities to motors.
        result = self.groupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            print("Failed to write velocities to motors.")
            return False
        return True              

    def clearGroupSyncReadParams(self):
        """Clear group sync reader parameters storage.
        """
        self.groupSyncRead.clearParam()
    
    def clearGroupSyncWriteParams(self):
        """Clear group sync writer parameters storage.
        """
        self.groupSyncWrite.clearParam()

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

        jointRadians = mappers.mapEncoderToJointRadians(presentPositions)
        position = self.kinematics.legDirectKinematics(legIdx, jointRadians)[:,3][:3]

        return np.array(position)

    def syncReadMotorsPositionsInLeg(self, legIdx):
        """Read all motors positions in given leg with sync reader.

        :param legIdx: Leg id.
        :return: 1x3 array with positions of all three joints in leg.
        """
        self.groupSyncRead.txRxPacket()
        positions = [self.groupSyncRead.getData(motorInLeg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH) for motorInLeg in self.motorsIds[legIdx]]
        return mappers.mapEncoderToJointRadians(positions)

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
