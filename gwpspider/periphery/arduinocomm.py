import serial
import threading
import time
import numpy as np

#TODO: Test new implementation with inheritance.
class ArduinoComm:
    """Parent class for communication with all Arduinos in system.
    """
    def __init__(self, device):
        self.received_message = str()

        self.device = device
        full_device_name = f'/dev/{device}'
        self.comm = serial.Serial(full_device_name, 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.locker = threading.Lock()

        self._init_reading_thread()
        self._handshake()

    #region properties
    @property
    def INIT_MESSAGE(self):
        return 'init'
    
    @property
    def INIT_RESPONSE(self):
        return 'ok'
    #endregion
    
    def _init_reading_thread(self):
        """Initialize reading data thread.
        """
        thread_name = f'{self.device}_reading_thread'
        reading_thread = threading.Thread(target = self._read_data, name = thread_name, daemon = True)
        reading_thread.start()

    def _read_data(self):
        """Constantly receiving data from Arduino. Runs in separate thread.
        """
        while True:
            time.sleep(0.001)
            with self.locker:
                msg = self.comm.readline()
            while '\\n' not in str(msg):
                time.sleep(0.001)
                with self.locker:
                    msg += self.comm.readline()

            self.received_message = msg.decode("utf-8", errors = "ignore").rstrip()

    def _send_data(self, msg):
        """Send data to Arduino.

        Args:
            msg (str): Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        with self.locker:
            self.comm.write(msg)

    def _handshake(self):
        """Handshake procedure with Arduino.
        """
        self._send_data(self.INIT_MESSAGE)
        time.sleep(0.01)
        while self.received_message != self.INIT_RESPONSE:
            time.sleep(0.01)
            self._send_data(self.INIT_MESSAGE)
        print("Handshake with grippers Arduino successfull.")

class GrippersArduino(ArduinoComm):
    """Class for controlling grippers via serial communication with Arduino.

    Args:
        ArduinoComm (ArduinoComm): Base class for communication with Arduinos.
    """
    def __init__(self):
        super().__init__(self.DEVICE_NAME)

    #region properties
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_GRIPPERS'

    @property
    def OPEN_COMMAND(self):
        return 'o'

    @property
    def CLOSE_COMMAND(self):
        return 'c'

    @property
    def IS_OPEN_RESPONSE(self):
        return '1'

    @property
    def IS_CLOSE_RESPONSE(self):
        return '0'

    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 10
    
    @property
    def GRIPPER(self):
        return 'g'
    
    @property
    def SWITCH(self):
        return 's'
    #endregion
    
    def move_gripper(self, leg_id, command):
        """Send command to move a gripper on selected leg.

        Args:
            leg_id (int): Leg id.
            command (str): Command to execute on gripper - 'o' for opening, 'c' for closing .
        """
        if command in (self.OPEN_COMMAND, self.CLOSE_COMMAND):
            msg = command + str(leg_id) + "\n"
            self._send_data(msg)
    
    def get_tools_states(self, tool):
        """Get states of desired tool (grippers or switches).

        Args:
            tool (str): Tool identification.

        Raises:
            ValueError: If selected tool is not valid (not gripper or switch).

        Returns:
            string: String of five characters, each represeting tool's state - either '1' or '0'.
        """
        if tool not in (self.GRIPPER, self.SWITCH):
            raise ValueError("Wrong tool selected.")
        
        rec_msg = ''
        while len(rec_msg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                rec_msg = self.received_message

        if tool == self.GRIPPER:
            return rec_msg[:5]
        return rec_msg[5:]
    
    def get_ids_of_attached_legs(self):
        """Get ids of attached legs. Leg is attached if switch and gripper are both in closed state.

        Returns:
            list: Ids of attached legs, empty list if none of the legs is attached.
        """
        rec_msg = ''
        while len(rec_msg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                rec_msg = self.received_message
        grippers_states = rec_msg[:5]
        switches_state = rec_msg[5:]
        attachedlegs_ids = []
        for leg_id, gripper_state in enumerate(grippers_states):
            if gripper_state == self.IS_CLOSE_RESPONSE and switches_state[leg_id] == self.IS_CLOSE_RESPONSE:
                attachedlegs_ids.append(int(leg_id))

        return attachedlegs_ids

class WaterPumpsBnoArduino(ArduinoComm):
    """Class for controlling water pumps via serial communication with Arduino.

    Args:
        ArduinoComm (ArduinoComm): Base class for communication with Arduinos.
    """
    def __init__(self):
        super().__init__(self.DEVICE_NAME)
    
    #region properties
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_BNOWP'
    
    @property
    def PUMP_OFF_COMMAND(self):
        return '0'
    
    @property
    def PUMP_ON_COMMAND(self):
        return '1'
    
    @property
    def INIT_BNO(self):
        return '2'
    
    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 30
    #endregion

    def water_pump_controll(self, command, pump_id):
        """Controll water pump - turn it on or off.

        Args:
            command (str): Command to execute on water pump - '1' for on, '0' for off.
            pump_id (int): Water pump id.
        """
        if command in (self.PUMP_ON_COMMAND, self.PUMP_OFF_COMMAND):
            msg = command + str(pump_id) + '\n'
            self._send_data(msg)
    
    def reset_bno(self):
        """Send command to reset BNO.
        """
        self._send_data(self.INIT_BNO)
    
    def get_rpy(self):
        """Read rpy angles.

        Returns:
            numpy.ndarray: 1x3 array of roll, pitch and yaw values in radians.
        """
        rec_msg = ''
        start_time = time.perf_counter()
        elapsed_time = time.perf_counter() - start_time
        use_reading = True
        
        while len(rec_msg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                rec_msg = self.received_message
            elapsed_time = time.process_time() - start_time
            if elapsed_time > 1.0:
                use_reading = False
                break
            
        if use_reading:   
            roll = float(rec_msg[0 : 5])
            pitch = float(rec_msg[5 : 10])
            yaw = float(rec_msg[10 : 15])
        else:
            roll, pitch, yaw = [1000, 1000, 1000]

        return np.array([roll, pitch, yaw], dtype = np.float32)
    
    def get_gravity_vector(self):
        """Read gravity vector

        Returns:
            numpy.ndarray: 1x3 array of gravity vector.
        """
        rec_msg = ''
        while len(rec_msg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                rec_msg = self.received_message

        x_gravity = float(rec_msg[15 : 20])
        y_gravity = float(rec_msg[20 : 25])
        z_gravity = float(rec_msg[25 : 30])

        return np.array([x_gravity, y_gravity, z_gravity], dtype = np.float32)