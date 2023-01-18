"""Module with water pumps controller class.
"""
import serial
import time
import threading
import numpy as np

class PumpsBnoArduino:
    """Class for controlling water pumps via serial communication with Arduino.
    """
    def __init__(self):
        """Init serial communication with Arduino.
        """
        self.INIT_RESPONSE = "OK"
        self.INIT_MESSAGE = "init"
        self.PUMP_OFF_COMMAND = '0'
        self.PUMP_ON_COMMAND = '1'
        self.INIT_BNO = '2'
        self.READ_BNO_RPY = '3'
        self.RECEIVED_MESSAGE_LENGTH = 30

        self.receivedMessage = ""

        self.comm = serial.Serial('/dev/ttyUSB2', 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.locker = threading.Lock()

        self.__initReadingThread()
        time.sleep(1)

        self.__handshake()
    
    #region public methods
    def pumpControll(self, command, pumpId):
        """Controll water pump - turn it on or off.

        Args:
            command (str): Command to execute on water pump - '1' for on, '0' for off.
            pumpId (int): Water pump id.
        """
        if command in (self.PUMP_ON_COMMAND, self.PUMP_OFF_COMMAND):
            msg = command + str(pumpId) + '\n'
            self.__sendData(msg)
    
    def resetBno(self):
        """Send command to reset BNO.
        """
        self.__sendData(self.INIT_BNO)
    
    def getRpy(self):
        """Read rpy angles.

        Returns:
            numpy.ndarray: 1x3 array of roll, pitch and yaw values in radians.
        """
        recMsg = ''
        while len(recMsg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                recMsg = self.receivedMessage
        roll = float(recMsg[0 : 5])
        pitch = float(recMsg[5 : 10])
        yaw = float(recMsg[10 : 15])

        return np.array([roll, pitch, yaw], dtype = np.float32)
    
    def getGravityVector(self):
        """Read gravity vector

        Returns:
            numpy.ndarray: 1x3 array of gravity vector.
        """
        recMsg = ''
        while len(recMsg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                recMsg = self.receivedMessage

        xGrav = float(recMsg[15 : 20])
        yGrav = float(recMsg[20 : 25])
        zGrav = float(recMsg[25 : 30])

        return np.array([xGrav, yGrav, zGrav], dtype = np.float32)
    #endregion

    #region private methods
    def __initReadingThread(self):
        """Initialize reading data thread.
        """
        readingThread = threading.Thread(target = self.__readData, name = "bno_serial_reading_thread", daemon = True)
        readingThread.start()

    def __sendData(self, msg):
        """Send data to Arduino.

        Args:
            msg (str): Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        with self.locker:
            self.comm.write(msg)
    
    def __readData(self):
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
            with self.locker:
                self.receivedMessage = msg.decode("utf-8", errors = "ignore").rstrip()

    def __handshake(self):
        """Handshake procedure with Arduino.
        """
        self.__sendData(self.INIT_MESSAGE)
        time.sleep(0.01)
        while self.receivedMessage != self.INIT_RESPONSE:
            time.sleep(0.01)
            self.__sendData(self.INIT_MESSAGE)
        print("Handshake with bno Arduino successfull.")
        
    #endregion