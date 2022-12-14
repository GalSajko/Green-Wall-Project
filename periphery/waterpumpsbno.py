"""Module with water pumps controller class.
"""
import serial

class PumpsBnoArduino:
    """Class for controlling water pumps via serial communication with Arduino.
    """
    def __init__(self):
        """Init serial communication with Arduino.
        """
        self.ON_COMMAND = '1'
        self.OFF_COMMAND = '0'
        self.INIT_RESPONSE = "OK"
        self.INIT_MESSAGE = "init"

        self.comm = serial.Serial('/dev/ttyUSB_ARDUINO_WATER_PUMP', 115200, timeout = 0)
        self.comm.reset_input_buffer()
    
    #region public methods
    def pumpControll(self, command, pumpId):
        """Controll water pump - turn it on or off.

        Args:
            command (str): Command to execute on water pump - '1' for on, '0' for off.
            pumpId (int): Water pump id.
        """
        if command in (self.ON_COMMAND, self.OFF_COMMAND):
            msg = command + str(pumpId) + '\n'
            self.__sendData(msg)
    #endregion

    #region private methods
    def __sendData(self, msg):
        """Send data to Arduino.

        Args:
            msg (str): Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        self.comm.write(msg)
    #endregion