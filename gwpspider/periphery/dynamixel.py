""" Wrapper module for controlling Dynamixel motors. Wraps some of the functions from dynamixel_sdk module.
"""

import threading
import os
import numpy as np
from dynamixel_sdk import *

from utils import mappers
import spider
import config

class MotorDriver:
    """ Class for controlling Dynamixel motors.
    """
    def __init__(self, motors_ids: list, enable_motors: bool = True):
        """Initialize USB port and enable motors.

        Args:
            motors_ids (list): List of motors ids.
            enable_motors (bool, optional): If True, enable torque in motors. Defaults to True.
        """
        self.__set_usb_low_latency()

        self.locker = threading.Lock()

        self.motors_ids = np.array(motors_ids, dtype = np.int8)
        self.port_handler = PortHandler(self.USB_DEVICE_NAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        self.group_sync_read_position = GroupSyncRead(self.port_handler, self.packet_handler, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
        self.group_sync_read_current = GroupSyncRead(self.port_handler, self.packet_handler, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
        self.group_sync_read_hardware_error = GroupSyncRead(self.port_handler, self.packet_handler, self.HARDWARE_ERROR_ADDR, self.HARDWARE_ERROR_DATA_LENGTH)
        self.group_sync_read_temperature = GroupSyncRead(self.port_handler, self.packet_handler, self.PRESENT_TEMPERATURE_ADDR, self.PRESENT_TEMPERATURE_DATA_LENGTH)

        self.group_sync_write_velocity = GroupSyncWrite(self.port_handler, self.packet_handler, self.GOAL_VELOCITY_ADDR, self.GOAL_VELOCITY_DATA_LENGTH)

        self.__init_group_read_write()

        self.__init_port()
        time.sleep(5)
        if enable_motors:
            self.enable_legs()
        
        # Disable (reset) watchdogs on motors.
        self.set_bus_watchdog(0)
    
    #region properties
    @property
    def MOTOR_SERIES(self):
        return 'X_SERIES'

    @property
    def TORQUE_ENABLE_ADDR(self):
        return 64
    
    @property
    def GOAL_VELOCITY_ADDR(self):
        return 104
    
    @property
    def PRESENT_POSITION_ADDR(self):
        return 132
    
    @property
    def PRESENT_CURRENT_ADDR(self):
        return 126
    
    @property
    def BUS_WATCHDOG_ADDR(self):
        return 98
    
    @property
    def HARDWARE_ERROR_ADDR(self):
        return 70
    
    @property
    def PRESENT_TEMPERATURE_ADDR(self):
        return 146
    
    @property
    def BAUDRATE(self):
        return 4000000
    
    @property
    def PROTOCOL_VERSION(self):
        return 2.0
    
    @property
    def USB_DEVICE_NAME(self):
        return '/dev/ttyUSB_DXL'
    
    @property
    def PRESENT_POSITION_DATA_LENGTH(self):
        return 4
    
    @property
    def PRESENT_CURRENT_DATA_LENGTH(self):
        return 2
    
    @property
    def GOAL_VELOCITY_DATA_LENGTH(self):
        return 4
    
    @property
    def HARDWARE_ERROR_DATA_LENGTH(self):
        return 1
    
    @property
    def PRESENT_TEMPERATURE_DATA_LENGTH(self):
        return 1
    
    @property
    def MAX_WORKING_TEMPERATURE(self):
        return 55
    

    #endregion

    #region public methods 
    def sync_read_motors_data(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Read positions, currents, hardware errors and temperature registers from all connected motors.

        Raises:
            KeyError: If error in fastSyncRead() functions.

        Returns:
            tuple: Four 5x3 numpy.ndarrays of positions, currents, harware error codes and temperatures.
        """
        try:
            with self.locker:
                _ = self.group_sync_read_current.fastSyncRead()
                _ = self.group_sync_read_position.fastSyncRead()
                _ = self.group_sync_read_hardware_error.fastSyncRead()
                _ = self.group_sync_read_temperature.fastSyncRead()

            currents = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            positions = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            hardware_errors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)   
            temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)    
               
            for leg in spider.LEGS_IDS:
                for idx, motor_in_leg in enumerate(self.motors_ids[leg]):
                    with self.locker:
                        positions[leg][idx] = self.group_sync_read_position.getData(motor_in_leg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
                        currents[leg][idx] = self.group_sync_read_current.getData(motor_in_leg, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
                        hardware_errors[leg][idx] = self.group_sync_read_hardware_error.getData(motor_in_leg, self.HARDWARE_ERROR_ADDR, self.HARDWARE_ERROR_DATA_LENGTH)
                        temperatures[leg][idx] = self.group_sync_read_temperature.getData(motor_in_leg, self.PRESENT_TEMPERATURE_ADDR, self.PRESENT_TEMPERATURE_DATA_LENGTH)

            mapped_positions = mappers.map_position_encoder_values_to_model_angles_radians(positions)
            mapped_currents = mappers.map_current_encoder_values_to_motors_currents_ampers(currents)

            return mapped_positions, mapped_currents, hardware_errors, temperatures
        
        except KeyError as ke:
            raise ke

    def sync_write_motors_velocities_in_legs(self, dq_c: np.ndarray) -> bool:
        """Write velocities to motors in all legs with sync writer.

        Args:
            dq_c (np.ndarray): 5x3 array of desired joints velocities.

        Returns:
            bool: True if writing was successfull, False otherwise.
        """
        for leg in spider.LEGS_IDS:
            motors_in_leg = self.motors_ids[leg]
            encoder_velocities = mappers.map_model_velocities_to_velocity_encoder_values(dq_c[leg]).astype(int)
            for i, motor in enumerate(motors_in_leg):
                commanded_joints_velocities_in_bytes = [
                    DXL_LOBYTE(DXL_LOWORD(encoder_velocities[i])),
                    DXL_HIBYTE(DXL_LOWORD(encoder_velocities[i])),
                    DXL_LOBYTE(DXL_HIWORD(encoder_velocities[i])),
                    DXL_HIBYTE(DXL_HIWORD(encoder_velocities[i])),
                ]
                with self.locker:
                    result = self.group_sync_write_velocity.changeParam(motor, commanded_joints_velocities_in_bytes)
                if not result:
                    print("Failed changing Group Sync Writer parameter %d" % motor)
                    return False
        # Write velocities to motors.
        with self.locker:
            result = self.group_sync_write_velocity.txPacket()
        if result != COMM_SUCCESS:
            # print("Failed to write velocities to motors.")
            return False
        return True

    def set_bus_watchdog(self, value: float):
        """Set watchdog on all motors to desired value.

        Args:
            value (float): Desired value.
        """
        motors_array = self.motors_ids.flatten()
        for motor_id in motors_array:
            with self.locker:
                result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.BUS_WATCHDOG_ADDR, value)
            comm = self.__comm_result_and_error_reader(result, error)
            if comm:
                print(f"Watchdog on motor {motor_id} has been successfully set to {value}")

    def enable_disable_legs(self, command: str, legs_ids: list | int = 5):
        """Enable or disable motors in legs.

        Args:
            command (str): Command to execute.
            legs_ids (list | int, optional): Ids of legs which are to be enabled or disabled. If value is 5 all legs are selected. Defaults to 5.
        """
        action = command == config.ENABLE_LEGS_COMMAND
        message = 'enabled' if action else 'disabled'

        if legs_ids == 5:
            motors_array = self.motors_ids.flatten()
        else:
            motors_array = self.motors_ids[legs_ids].flatten()

        for motor_id in motors_array:
            # Disable torque.
            with self.locker:
                result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE_ADDR, action)
            comm = self.__comm_result_and_error_reader(result, error)
            if comm:
                print(f"Motor {motor_id} has been successfully {message}.")

    def reboot_motors(self, motors_ids: list):
        """Reboot motors with given IDs. Used only, when motors are in hardware error state.

        Args:
            motors_ids (list): Ids of a motors.
        """
        motors_ids = motors_ids.flatten()
        for motor_id in motors_ids:
            with self.locker:
                self.packet_handler.reboot(self.port_handler, int(motor_id))
            if str(motor_id)[1] == '3':
                with self.locker:
                    self.packet_handler.reboot(self.port_handler, int(motor_id + 1))
    #endregion

    #region private methods
    def __init_port(self):
        """Initialize USB port and set baudrate. Note that baudrate should match with baudrate that is already set on motors.
        """
        if self.port_handler.openPort():
            print("Port successfully opened.")
        else:
            print("Failed to open the port.")

        if self.port_handler.setBaudRate(self.BAUDRATE):
            print("Baudrate successfully changed.")
        else:
            print("Failed to change the baudrate.")

    def __init_group_read_write(self) -> bool:
        """Add parameters for all motors into storage for group-sync reading and writing.

        Returns:
            bool: True if adding parameters was successful, False otherwise.
        """
        self.group_sync_read_current.clearParam()
        self.group_sync_read_position.clearParam()
        self.group_sync_read_hardware_error.clearParam()
        self.group_sync_read_temperature.clearParam()

        result_add_params = self.__add_group_sync_read_params()
        if not result_add_params:
            return False

        for leg in spider.LEGS_IDS:
            init_velocities = np.zeros(spider.NUMBER_OF_MOTORS_IN_LEG)
            encoder_velocities = mappers.map_model_velocities_to_velocity_encoder_values(init_velocities).astype(int)
            for i, motor in enumerate(self.motors_ids[leg]):
                init_velocity_in_bytes = [
                    DXL_LOBYTE(DXL_LOWORD(encoder_velocities[i])),
                    DXL_HIBYTE(DXL_LOWORD(encoder_velocities[i])),
                    DXL_LOBYTE(DXL_HIWORD(encoder_velocities[i])),
                    DXL_HIBYTE(DXL_HIWORD(encoder_velocities[i])),
                ]
                result_write = self.group_sync_write_velocity.addParam(motor, init_velocity_in_bytes)
                if not result_write:
                    print("Failed adding parameter %d to Group Sync Writer" % motor)
                    return False

        return True
    
    def __add_group_sync_read_params(self) -> bool:
        """Add parameters (motors ids) to group sync reader parameters storages - for position and current reading.

        Returns:
            bool: True if adding was successfull, False otherwise.
        """
        for motor in self.motors_ids.flatten():
            result_position = self.group_sync_read_position.addParam(motor)
            result_current = self.group_sync_read_current.addParam(motor)
            result_hardware_error = self.group_sync_read_hardware_error.addParam(motor)
            result_temperature = self.group_sync_read_temperature.addParam(motor)
            if not (result_position and result_current and result_hardware_error and result_temperature):
                print("Failed adding parameter %d to Group Sync Reader" % motor)
                return False
        return True

    def __comm_result_and_error_reader(self, result: str, error: str) -> bool:
        """Read communication result and error.

        Args:
            result (str): Result.
            error (str): Error.

        Returns:
            bool: True if communication was successfull, False otherwise.
        """
        if result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(result))
            return False
        if error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False

        return True

    def __set_usb_low_latency(self):
        """Set USB device latency on 1ms.
        """
        cmd = f'sudo setserial {self.USB_DEVICE_NAME} low_latency'
        os.system(cmd)
    #endregion