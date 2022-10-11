""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import numpy as np
from dynamixel_sdk import *
import threading
import os

import calculations
import mappers
import environment

class MotorDriver:
    """ Class for controlling Dynamixel motors.
    """
    def __init__(self, motorsIds, enableMotors = True):
        """Initialize USB port and enable motors.

        Args:
            motorsIds (list): List of motors ids.
            enableMotors (bool, optional): If True, enable torque in motors. Defaults to True.
        """
        # Motor series - we are using XM series.
        self.MOTOR_SERIES = "X_SERIES"
        # Control table addresses.
        self.TORQUE_ENABLE_ADDR = 64
        self.GOAL_VELOCITY_ADDR = 104
        self.PRESENT_POSITION_ADDR = 132
        self.PRESENT_CURRENT_ADDR = 126
        self.BAUDRATE = 4000000
        self.PROTOCOL_VERSION = 2.0
        self.USB_DEVICE_NAME = "/dev/ttyUSB0"
        self.PRESENT_POSITION_DATA_LENGTH = 4
        self.PRESENT_CURRENT_DATA_LENGTH = 2
        self.GOAL_VELOCITY_DATA_LENGTH = 4
        self.ERROR_ADDR = 70

        self.__setUSBLowLatency()

        self.locker = threading.Lock()
        self.readingThreadRunning = False

        self.kinematics = calculations.Kinematics()
        self.spider = environment.Spider()
        self.matrixCalculator = calculations.TransformationCalculator()

        self.motorsIds = np.array(motorsIds)
        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
        self.groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.GOAL_VELOCITY_ADDR, self.GOAL_VELOCITY_DATA_LENGTH)

        self.__initGroupReadWrite()

        self.__initPort()
        if enableMotors:
            self.enableMotors()

    #region public methods 
    def syncReadAnglesAndCurrentsWrapper(self):
        """Read positions and currents from all connected motors.

        Returns:
            tuple: Two 5x3 numpy.ndarrays with positions in radians and currents in Ampers in each motor.
        """
        _ = self.groupSyncReadCurrent.fastSyncRead()
        _ = self.groupSyncReadPosition.fastSyncRead()

        currents = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)
        positions = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)

        for leg in self.spider.LEGS_IDS:
            for idx, motorInLeg in enumerate(self.motorsIds[leg]):
                positions[leg][idx] = self.groupSyncReadPosition.getData(motorInLeg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
                currents[leg][idx] = self.groupSyncReadCurrent.getData(motorInLeg, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
            positions[leg] = mappers.mapPositionEncoderValuesToModelAnglesRadians(positions[leg])
            currents[leg] = mappers.mapCurrentEncoderValuesToMotorsCurrentsAmpers(currents[leg])

        return positions, currents

    def syncWriteMotorsVelocitiesInLegs(self, legIds, qCd):
        """Write velocities to motors in given legs with sync writer.

        Args:
            legId (list): Legs ids.
            qCd (list): nx3 array of desired velocities, where n is number of given legs.

        Returns:
            bool: True if writing was successfull, False otherwise.
        """
        for idx, leg in enumerate(legIds):
            motorsInLeg = self.motorsIds[leg]
            encoderVelocities = mappers.mapModelVelocitiesToVelocityEncoderValues(qCd[idx]).astype(int)
            for i, motor in enumerate(motorsInLeg):
                qCdBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocities[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocities[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocities[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocities[i]))]
                with self.locker:
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

    def enableMotors(self):
        """ Enable all of the motors, given at class initialization, if they are connected.
        """
        motorsArray = self.motorsIds.flatten()
        for motorId in motorsArray:
            # Enable torque.
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 1)
            comm = self.__commResultAndErrorReader(result, error)
            if comm:
                print("Motor %d has been successfully enabled" % motorId)

    def disableMotors(self, motorsIds):
        """Disable motors.

        Args:
            motorsIds (list): Ids of motors, which are to be disabled.
        """
        if motorsIds == 5:
            motorsIds = self.motorsIds.flatten()
        # Disable torque.
        for motorId in motorsIds:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)
            comm = self.__commResultAndErrorReader(result, error)
            if comm:
                print("Motor %d has been successfully disabled" % motorId)

    def disableLegs(self, legsIds = 5):
        """Disable all of the motors in given legs.

        Args:
            legsIds (list, optional): Ids of legs, which are to be disabled, if value is 5 all legs will be disabled. Defaults to 5.
        """
        if legsIds == 5:
            motorsArray = self.motorsIds.flatten()
        else:
            motorsArray = self.motorsIds[legsIds]

        for motorId in motorsArray:
            # Disable torque.
            _, _ = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 0)

    def enableLegs(self, legsIds = 5):
        """Enable all of the motors in given legs.

        Args:
            legId (list, optional): Ids of legs, which are to be enabled, if value is 5 all legs will be enabled. Defaults to 5.
        """
        if legsIds == 5:
            motorsArray = self.motorsIds.flatten()
        else:
            motorsArray = self.motorsIds[legsIds]

        for motorId in motorsArray:
            # Enable torque.
            _, _ = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 1)

    def readHardwareErrorRegister(self):
        """Read hardware error register for each motor.
        """
        for motorId in self.motorsIds.flatten():
            hwError, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, motorId, self.ERROR_ADDR)
            binaryError = format(hwError, '0>8b')
            print(f"Motor {motorId} - error code {binaryError}")
    #endregion

    #region private methods
    def __initPort(self):
        """Initialize USB port and set baudrate. Note that baudrate should match with baudrate that is already set on motors.
        """
        if self.portHandler.openPort():
            print("Port successfully opened.")
        else:
            print("Failed to open the port.")

        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Baudrate successfully changed.")
        else:
            print("Failed to change the baudrate.")

    def __initGroupReadWrite(self):
        """Add parameters for all motors into storage for group-sync reading and writing.
        """
        self.groupSyncReadCurrent.clearParam()
        self.groupSyncReadPosition.clearParam()

        resultAddParams = self.__addGroupSyncReadParams()
        if not resultAddParams:
            return False

        for leg in self.spider.LEGS_IDS:
            initVelocities = np.zeros(self.spider.NUMBER_OF_MOTORS_IN_LEG)
            encoderVelocoties = mappers.mapModelVelocitiesToVelocityEncoderValues(initVelocities).astype(int)
            for i, motor in enumerate(self.motorsIds[leg]):
                initVelocityBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocoties[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocoties[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocoties[i]))]
                resultWrite = self.groupSyncWrite.addParam(motor, initVelocityBytes)
                if not resultWrite:
                    print("Failed adding parameter %d to Group Sync Writer" % motor)
                    return False

        return True
    
    def __addGroupSyncReadParams(self):
        """Add parameters (motors ids) to group sync reader parameters storages - for position and current reading.

        Returns:
            bool: True if adding was successfull, False otherwise.
        """
        for motor in self.motorsIds.flatten():
            resultPosition = self.groupSyncReadPosition.addParam(motor)
            resultCurrent = self.groupSyncReadCurrent.addParam(motor)
            if not (resultPosition and resultCurrent):
                print("Failed adding parameter %d to Group Sync Reader" % motor)
                return False
        return True

    def __commResultAndErrorReader(self, result, error):
        """Read communication result and error.

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

    def __setUSBLowLatency(self):
        """Set USB device latency on 1ms.
        """
        cmd = f'sudo setserial {self.USB_DEVICE_NAME} low_latency'
        os.system(cmd)
    #endregion