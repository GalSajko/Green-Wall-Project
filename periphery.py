import adafruit_bno055
import board
import numpy as np
import serial
import threading
import time

import mappers


class BNO055:
    """Class for communication with BNO055 sensor, connected with i2c protocol.
    """
    def __init__(self, isVertical = False):
        i2c = board.I2C()
        self.bno055 = adafruit_bno055.BNO055_I2C(i2c)
        # Remap axis to fix initial value of pitch, when spider is verticaly on the wall.
        if isVertical:
            self.bno055.axis_remap = (0, 2, 1, 0, 1, 0)
        print("BNO055 initializing...")
        time.sleep(2)
        self.initRpyOffsets = mappers.mapBno055ToSpiderDegrees(self.bno055.euler)
        self.prevGravityVector = np.zeros(3)
        print(f"Initial offsets are: {self.initRpyOffsets}")
    
    def readEulers(self):
        """Read, map and return spider's orientation given in rpy euler angles, considering initial offsets of sensor.

        Returns:
            numpy.ndarray: 1x3 array of spider's roll, pitch and yaw angles in radians.
        """
        rpy = mappers.mapBno055ToSpiderDegrees(self.bno055.euler)
        rpy -= self.initRpyOffsets

        return rpy
    
    def readGravity(self):
        """Read gravity vector.

        Returns:
            list: 1x3 gravity vector.
        """
        gravity = self.bno055.gravity
        try:
            gravity = mappers.mapGravityVectorToSpiderOrigin(gravity)
            self.prevGravityVector = gravity
            return gravity
        except TypeError:
            return self.prevGravityVector
        

class GripperController:
    """Class for controlling grippers via serial communication with Arduino.
    """
    def __init__(self):
        """Init serial communication with Arduino.
        """
        self.OPEN_COMMAND = "o"
        self.CLOSE_COMMAND = "c"
        self.INIT_MESSAGE = "init"
        self.GRIPPER_OPENED_RESPONSE = "1"
        self.GRIPPER_CLOSED_RESPONSE = "0"
        self.SWITCH_OPEN_RESPONSE = "1"
        self.SWITCH_CLOSE_RESPONSE = "0"
        self.INIT_RESPONSE = "OK"
        self.RECEIVED_MESSAGE_LENGTH = 10

        self.receivedMessage = ""

        self.comm = serial.Serial('/dev/ttyUSB_ARDUINO_GRIPPERS', 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.locker = threading.Lock()

        self.initReadingThread()

        self.handshake()

    def initReadingThread(self):
        """Initialize reading data thread.
        """
        readingThread = threading.Thread(target = self.readData, name = "gripper_serial_reading_thread", daemon = True)
        readingThread.start()

    def readData(self):
        """Constantly receiving data from Arduino. Runs in separate thread.
        """
        while True:
            time.sleep(0.001)
            with self.locker:
                msg = self.comm.readline()
            while not '\\n' in str(msg):
                time.sleep(0.001)
                with self.locker:
                    msg += self.comm.readline()

            self.receivedMessage = msg.decode("utf-8", errors = "ignore").rstrip()

    def sendData(self, msg):
        """Send data to Arduino.

        Args:
            msg (str): Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        with self.locker:
            self.comm.write(msg)

    def moveGripper(self, legId, command):
        """Send command to move a gripper on selected leg.

        Args:
            legId (int): Leg id.
            command (str): Command to execute on gripper - 'o' for opening, 'c' for closing .
        """
        if command in (self.OPEN_COMMAND, self.CLOSE_COMMAND):
            msg = command + str(legId) + "\n"
            self.sendData(msg)

    def openGrippersAndWait(self, legsIds):
        """Open grippers on given legs and wait for them to come in open state.

        Args:
            legsIds (list): Ids of used legs.

        Returns:
            bool: True, when all grippers reach open state.
        """
        for leg in legsIds:
            self.moveGripper(leg, self.OPEN_COMMAND)
        checkArray = np.array([False] * len(legsIds))
        while not checkArray.all():
            with self.locker:
                recMsg = self.receivedMessage
            if len(recMsg) == self.RECEIVED_MESSAGE_LENGTH:
                for id, leg in enumerate(legsIds):
                    if recMsg[leg] == self.GRIPPER_OPENED_RESPONSE:
                        checkArray[id] = True
        return True

    def getSwitchesStates(self):
        """Get switches states - 0 for closed switch (attached leg), 1 otherwise.

        Returns:
            string: String of five characters, each represeting switch state - either '1' or '0', depends on the state of the switch.
        """
        recMsg = ''
        while not len(recMsg) == self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                recMsg = self.receivedMessage

        return recMsg[5:]

    def getIdsOfAttachedLegs(self):
        """Get ids of attached legs. Leg is attached if switch and gripper are both in closed state.

        Returns:
            list: Ids of attached legs, empty list if none of the legs is attached.
        """
        recMsg = ''
        while not len(recMsg) == self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                recMsg = self.receivedMessage
        grippersStates = recMsg[:5]
        switchesStates = recMsg[5:]
        attachedLegsIds = []
        for id, gripper in enumerate(grippersStates):
            if gripper == self.GRIPPER_CLOSED_RESPONSE and switchesStates[id] == self.SWITCH_CLOSE_RESPONSE:
                attachedLegsIds.append(id)

        return attachedLegsIds

    def handshake(self):
        """Handshake procedure with Arduino.
        """
        self.sendData(self.INIT_MESSAGE)
        time.sleep(0.01)
        while not self.receivedMessage == self.INIT_RESPONSE:
            time.sleep(0.01)
            self.sendData(self.INIT_MESSAGE)
        print("Handshake with grippers Arduino successfull.")

class WaterPumpController:
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

    def sendData(self, msg):
        """Send data to Arduino.

        Args:
            msg (str): Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        self.comm.write(msg)
    
    def pumpControll(self, command, pumpId):
        """Controll water pump - turn it on of off.

        Args:
            command (str): Command to execute on water pump - '1' for on, '0' for off.
            pumpId (int): Water pump id.
        """
        if command in (self.ON_COMMAND, self.OFF_COMMAND):
            msg = command + str(pumpId) + '\n'
            self.sendData(msg)