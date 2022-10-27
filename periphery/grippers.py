"""Module with grippers-controller class.
"""
import serial
import threading
import time
import numpy as np

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

        self.comm = serial.Serial('/dev/ttyUSB2', 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.locker = threading.Lock()

        self.__initReadingThread()

        self.__handshake()

    #region public methods
    def moveGripper(self, legId, command):
        """Send command to move a gripper on selected leg.

        Args:
            legId (int): Leg id.
            command (str): Command to execute on gripper - 'o' for opening, 'c' for closing .
        """
        if command in (self.OPEN_COMMAND, self.CLOSE_COMMAND):
            msg = command + str(legId) + "\n"
            self.__sendData(msg)

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
                for legId, leg in enumerate(legsIds):
                    if recMsg[leg] == self.GRIPPER_OPENED_RESPONSE:
                        checkArray[legId] = True
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
        for legId, gripper in enumerate(grippersStates):
            if gripper == self.GRIPPER_CLOSED_RESPONSE and switchesStates[legId] == self.SWITCH_CLOSE_RESPONSE:
                attachedLegsIds.append(legId)

        return attachedLegsIds
    #endregion
    
    #region private methods
    def __initReadingThread(self):
        """Initialize reading data thread.
        """
        readingThread = threading.Thread(target = self.__readData, name = "gripper_serial_reading_thread", daemon = True)
        readingThread.start()

    def __readData(self):
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

    def __handshake(self):
        """Handshake procedure with Arduino.
        """
        self.__sendData(self.INIT_MESSAGE)
        time.sleep(0.01)
        while not self.receivedMessage == self.INIT_RESPONSE:
            time.sleep(0.01)
            self.__sendData(self.INIT_MESSAGE)
        print("Handshake with grippers Arduino successfull.")
    #endregion