"""Module with grippers-controller class.
"""
import serial
import threading
import time
import numpy as np

class GrippersArduino:
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

        self.comm = serial.Serial('/dev/ttyUSB_GRIPPERS', 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.locker = threading.Lock()

        self.__initReadingThread()

        self.__handshake()

    #region public methods
    def move_gripper(self, leg_id, command):
        """Send command to move a gripper on selected leg.

        Args:
            leg_id (int): Leg id.
            command (str): Command to execute on gripper - 'o' for opening, 'c' for closing .
        """
        if command in (self.OPEN_COMMAND, self.CLOSE_COMMAND):
            msg = command + str(leg_id) + "\n"
            self.__sendData(msg)

    def openGrippersAndWait(self, legs_ids):
        """Open grippers on given legs and wait for them to come in open state.

        Args:
            legs_ids (list): Ids of used legs.

        Returns:
            bool: True, when all grippers reach open state.
        """
        for leg in legs_ids:
            self.move_gripper(leg, self.OPEN_COMMAND)
        checkArray = np.array([False] * len(legs_ids))
        while not checkArray.all():
            with self.locker:
                recMsg = self.receivedMessage
            if len(recMsg) == self.RECEIVED_MESSAGE_LENGTH:
                for leg_id, leg in enumerate(legs_ids):
                    if recMsg[leg] == self.GRIPPER_OPENED_RESPONSE:
                        checkArray[leg_id] = True
            time.sleep(0.05)
        return True

    def getSwitchesStates(self):
        """Get switches states - 0 for closed switch (attached leg), 1 otherwise.

        Returns:
            string: String of five characters, each represeting switch state - either '1' or '0', depends on the state of the switch.
        """
        recMsg = ''
        while len(recMsg) != self.RECEIVED_MESSAGE_LENGTH:
            with self.locker:
                recMsg = self.receivedMessage

        return recMsg[5:]

    def get_ids_of_attached_legs(self):
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
        attachedlegs_ids = []
        for leg_id, gripperState in enumerate(grippersStates):
            if gripperState == self.GRIPPER_CLOSED_RESPONSE and switchesStates[leg_id] == self.SWITCH_CLOSE_RESPONSE:
                attachedlegs_ids.append(int(leg_id))

        return attachedlegs_ids
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
            while '\\n' not in str(msg):
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