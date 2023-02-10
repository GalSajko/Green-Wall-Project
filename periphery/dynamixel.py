""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import threading
import os
import numpy as np
from dynamixel_sdk import *

import mappers
from environment import spider

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
        self.BUS_WATCHDOG_ADDR = 98
        self.HARDWARE_ERROR_ADDR = 70
        self.PRESENT_TEMPERATURE_ADDR = 146

        self.BAUDRATE = 4000000
        self.PROTOCOL_VERSION = 2.0
        self.USB_DEVICE_NAME = "/dev/ttyUSB1"

        self.PRESENT_POSITION_DATA_LENGTH = 4
        self.PRESENT_CURRENT_DATA_LENGTH = 2
        self.GOAL_VELOCITY_DATA_LENGTH = 4
        self.HARDWARE_ERROR_DATA_LENGTH = 1
        self.PRESENT_TEMPERATURE_DATA_LENGTH = 1

        self.MAX_WORKING_TEMPERATURE = 40

        self.__setUSBLowLatency()

        self.locker = threading.Lock()

        self.motorsIds = np.array(motorsIds, dtype = np.int8)
        self.portHandler = PortHandler(self.USB_DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
        self.groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
        self.groupSyncReadHardwareError = GroupSyncRead(self.portHandler, self.packetHandler, self.HARDWARE_ERROR_ADDR, self.HARDWARE_ERROR_DATA_LENGTH)
        self.groupSyncReadTemperature = GroupSyncRead(self.portHandler, self.packetHandler, self.PRESENT_TEMPERATURE_ADDR, self.PRESENT_TEMPERATURE_DATA_LENGTH)

        self.groupSyncWriteVelocity = GroupSyncWrite(self.portHandler, self.packetHandler, self.GOAL_VELOCITY_ADDR, self.GOAL_VELOCITY_DATA_LENGTH)

        self.__initGroupReadWrite()

        self.__initPort()
        if enableMotors:
            self.enableLegs()
        
        # Disable (reset) watchdogs on motors.
        self.setBusWatchdog(0)

    #region public methods 
    def syncReadMotorsData(self):
        """Read positions, currents, hardware errors and temperature registers from all connected motors.

        Raises:
            KeyError: If error in fastSyncRead() functions.

        Returns:
            tuple: Four 5x3 numpy.ndarrays of positions, currents, harware error codes and temperatures.
        """
        try:
            with self.locker:
                _ = self.groupSyncReadCurrent.fastSyncRead()
                _ = self.groupSyncReadPosition.fastSyncRead()
                _ = self.groupSyncReadHardwareError.fastSyncRead()
                _ = self.groupSyncReadTemperature.fastSyncRead()

            currents = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            positions = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            hardwareErrors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)   
            temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)    
               
            for leg in spider.LEGS_IDS:
                for idx, motorInLeg in enumerate(self.motorsIds[leg]):
                    with self.locker:
                        positions[leg][idx] = self.groupSyncReadPosition.getData(motorInLeg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
                        currents[leg][idx] = self.groupSyncReadCurrent.getData(motorInLeg, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
                        hardwareErrors[leg][idx] = self.groupSyncReadHardwareError.getData(motorInLeg, self.HARDWARE_ERROR_ADDR, self.HARDWARE_ERROR_DATA_LENGTH)
                        temperatures[leg][idx] = self.groupSyncReadTemperature.getData(motorInLeg, self.PRESENT_TEMPERATURE_ADDR, self.PRESENT_TEMPERATURE_DATA_LENGTH)

            mappedPositions = mappers.mapPositionEncoderValuesToModelAnglesRadians(positions)
            mappedCurrents = mappers.mapCurrentEncoderValuesToMotorsCurrentsAmpers(currents)

            return mappedPositions, mappedCurrents, hardwareErrors, temperatures
        
        except KeyError as ke:
            raise ke

    def syncWriteMotorsVelocitiesInLegs(self, qCd):
        """Write velocities to motors in all legs with sync writer.

        Args:
            qCd (list): 5x3 array of desired joints velocities.

        Returns:
            bool: True if writing was successfull, False otherwise.
        """
        for leg in spider.LEGS_IDS:
            motorsInLeg = self.motorsIds[leg]
            encoderVelocities = mappers.mapModelVelocitiesToVelocityEncoderValues(qCd[leg]).astype(int)
            for i, motor in enumerate(motorsInLeg):
                qCdBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocities[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocities[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocities[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocities[i]))]
                with self.locker:
                    result = self.groupSyncWriteVelocity.changeParam(motor, qCdBytes)
                if not result:
                    print("Failed changing Group Sync Writer parameter %d" % motor)
                    return False
        # Write velocities to motors.
        with self.locker:
            result = self.groupSyncWriteVelocity.txPacket()
        if result != COMM_SUCCESS:
            # print("Failed to write velocities to motors.")
            return False
        return True

    def setBusWatchdog(self, value):
        """Set watchdog on all motors to desired value.
        """
        motorsArray = self.motorsIds.flatten()
        for motorId in motorsArray:
            with self.locker:
                result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.BUS_WATCHDOG_ADDR, value)
            comm = self.__commResultAndErrorReader(result, error)
            if comm:
                print(f"Watchdog on motor {motorId} has been successfully set to {value}")

    def disableLegs(self, legsIds = 5):
        """Disable all of the motors in given legs.

        Args:
            legsIds (list, optional): Ids of legs, which are to be disabled, if value is 5 all legs will be disabled. Defaults to 5.
        """
        if legsIds == 5:
            motorsArray = self.motorsIds.flatten()
        else:
            motorsArray = self.motorsIds[legsIds].flatten()

        for motorId in motorsArray:
            # Disable torque.
            with self.locker:
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
            with self.locker:
                result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motorId, self.TORQUE_ENABLE_ADDR, 1)
            comm = self.__commResultAndErrorReader(result, error)
            if comm:
                print(f"Motor {motorId} has been successfully enabled.")
            
    
    def rebootMotor(self, motorsIds):
        """Reboot motors with given IDs. Used only, when motors are in hardware error state.

        Args:
            motorsIds (int): Ids of a motors.
        """
        for motorId in motorsIds:
            with self.locker:
                self.packetHandler.reboot(self.portHandler, motorId)
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
        self.groupSyncReadHardwareError.clearParam()
        self.groupSyncReadTemperature.clearParam()

        resultAddParams = self.__addGroupSyncReadParams()
        if not resultAddParams:
            return False

        for leg in spider.LEGS_IDS:
            initVelocities = np.zeros(spider.NUMBER_OF_MOTORS_IN_LEG)
            encoderVelocities = mappers.mapModelVelocitiesToVelocityEncoderValues(initVelocities).astype(int)
            for i, motor in enumerate(self.motorsIds[leg]):
                initVelocityBytes = [DXL_LOBYTE(DXL_LOWORD(encoderVelocities[i])), DXL_HIBYTE(DXL_LOWORD(encoderVelocities[i])), DXL_LOBYTE(DXL_HIWORD(encoderVelocities[i])), DXL_HIBYTE(DXL_HIWORD(encoderVelocities[i]))]
                resultWrite = self.groupSyncWriteVelocity.addParam(motor, initVelocityBytes)
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
            resultHardwareError = self.groupSyncReadHardwareError.addParam(motor)
            resultTemperature = self.groupSyncReadTemperature.addParam(motor)
            if not (resultPosition and resultCurrent and resultHardwareError and resultTemperature):
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
        if error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
            return False

        return True

    def __setUSBLowLatency(self):
        """Set USB device latency on 1ms.
        """
        cmd = f'sudo setserial {self.USB_DEVICE_NAME} low_latency'
        os.system(cmd)
    #endregion