""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import numpy as np
from dynamixel_sdk import *
import time
import itertools as itt

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
        self.USB_DEVICE_NAME = "/dev/ttyUSB1"
        self.PRESENT_POSITION_DATA_LENGTH = 4
        self.GOAL_VELOCITY_DATA_LENGTH = 4
        self.ERROR_ADDR = 70

        self.kinematics = calculations.Kinematics()
        self.spider = environment.Spider()
        
        self.motorsIds = np.array(motorsIds)
        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Group sync positions reader and group sync velocity writer.
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.GOAL_VELOCITY_ADDR, self.GOAL_VELOCITY_DATA_LENGTH)

        self.initGroupReadWrite()

        self.initPort()
        if enableMotors:
            self.enableMotors()

        # self.readHardwareErrorRegister()

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
    
    def initGroupReadWrite(self):
        """Add parameters for all motors into storage for group-sync reading and writing.
        """
        resultRead = self.addGroupSyncReadParams(self.spider.LEGS_IDS)
        if not resultRead:
            return False

        for leg in self.spider.LEGS_IDS:
            initVelocities = np.zeros(self.spider.NUMBER_OF_MOTORS_IN_LEG)
            encoderVelocoties = mappers.mapJointVelocitiesToEncoderValues(initVelocities).astype(int)
            for i, motor in enumerate(self.motorsIds[leg]):
                initVelocityBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocoties[i]))]
                resultWrite = self.groupSyncWrite.addParam(motor, initVelocityBytes)
                if not resultWrite:
                    print("Failed adding parameter %d to Group Sync Writer" % motor)
                    return False
        
        return True

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
    
    def disableMotors(self, motorsIds):
        """Disable given motors.

        :param motorsIds: Array of motors ids to disable.
        """
        if motorsIds == 5:
            motorsIds = self.motorsIds.flatten()
        # Disable torque.
        for motorId in motorsIds:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
            comm = self.commResultAndErrorReader(result, error)
            if comm:
                print("Motor %d has been successfully disabled" % motorId)

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
    
    def syncReadMotorsPositionsInLegs(self, legsIds, calculateLegPositions = False, base = 'leg'):
        """Read motors positions in given legs with sync reader.

        :param legsIds: Legs ids.
        :param calculateLegPositions: If true, calculate legs positions from joints values.
        :param base: Origin in which to calculate legs positions.
        :return: nx3 matrix with motors positions in radians, if calculateLegPositions is False, nx3 matrix with legs positions. 
        otherwise.
        """
        if base is not None and base != 'leg' and base != 'spider':
            print("Invalid value of base parameter.")
            return False
        
        _ = self.groupSyncRead.fastSyncRead()

        mappedPositions = []
        for leg in legsIds:
            positions = [self.groupSyncRead.getData(motorInLeg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH) for motorInLeg in self.motorsIds[leg]]
            jointsValues = mappers.mapEncoderToJointsRadians(positions)
            if not calculateLegPositions:
                mappedPositions.append(jointsValues)
            else:
                if base == 'leg':
                    mappedPositions.append(self.kinematics.legForwardKinematics(leg, jointsValues)[:,3][:3])
                elif base == 'spider':
                    mappedPositions.append(self.kinematics.spiderBaseToLegTipForwardKinematics(leg, jointsValues)[:,3][:3])

        return np.array(mappedPositions)
    
    def syncReadPlatformPose(self, legsIds, legsGlobalPositions):
        """Read platform pose in global.

        :param legsIds: Legs used for calculating platform pose - those legs that are attached to pins.
        :param legsGlobalPositions: Global positions (pins) of used legs.
        """
        legsGlobalPositions = np.array(legsGlobalPositions)
        spiderXyz = []
        for legsSubset in itt.combinations(legsIds, 3):
            legsSubset = np.array(legsSubset)
            subsetIdxs = [legsIds.index(leg) for leg in legsSubset]
            jointsValues = self.syncReadMotorsPositionsInLegs(legsSubset)
            legsPoses = [self.kinematics.spiderBaseToLegTipForwardKinematics(leg, jointsValues[idx]) for idx, leg in enumerate(legsSubset)]
            spiderXyz.append(self.kinematics.platformForwardKinematics(legsSubset, legsGlobalPositions[(subsetIdxs)], legsPoses))
        spiderXyz = np.mean(np.array(spiderXyz), axis = 0)

        return spiderXyz

    def syncWriteMotorsVelocitiesInLegs(self, legIdx, qCd):
        """Write motors velocities to given motors in legs with sync writer. Note that parameters are added at class initialization.

        :param legIdx: Legs ids.
        :param qCd: Reference velocities.
        :param add: If true add parameters to storage, otherwise only change them, defaults to True.
        :return: True if writing was successfull, false otherwise.
        """
        
        for idx, leg in enumerate(legIdx):
            motorsInLeg = self.motorsIds[leg]
            encoderVelocoties = mappers.mapJointVelocitiesToEncoderValues(qCd[idx]).astype(int)
            for i, motor in enumerate(motorsInLeg):
                qCdBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocoties[i]))]
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

        jointRadians = mappers.mapEncoderToJointsRadians(presentPositions)
        position = self.kinematics.legForwardKinematics(legIdx, jointRadians)[:,3][:3]

        return np.array(position)

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

    def readHardwareErrorRegister(self):
        """Read hardware error register for each motor.
        """
        for motorId in self.motorsIds.flatten():
            hwError, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, motorId, self.ERROR_ADDR)
            binaryError = format(hwError, '0>8b')
            print(f"Motor {motorId} - error code {binaryError}")
