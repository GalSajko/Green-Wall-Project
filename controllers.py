
import numpy as np
import time
import serial
import threading
import queue
import os
import inspect
import multiprocessing
import matplotlib.pyplot as plt

import calculations
import environment as env
import dynamixel as dmx
import planning
import config

class VelocityController:
    """ Class for velocity-control of spider's movement.
    """
    def __init__ (self):
        self.matrixCalculator = calculations.MatrixCalculator()
        self.kinematics = calculations.Kinematics()
        self.geometryTools = calculations.GeometryTools()
        self.spider = env.Spider()
        self.trajectoryPlanner = planning.TrajectoryPlanner()
        self.pathPlanner = planning.PathPlanner(0.05, 0.1, 'squared')
        self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.gripperController = GripperController()

        self.legsQueues = [queue.Queue() for i in range(self.spider.NUMBER_OF_LEGS)]
        self.sentinel = object()
        self.lastMotorsPositions = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        self.locker = threading.Lock()
        self.killControllerThread = False

        # self.Kp = np.array([[300, 300, 300]] * self.spider.NUMBER_OF_LEGS)
        self.Kp = np.array([[5, 5, 5]] * self.spider.NUMBER_OF_LEGS)
        self.Kd = np.ones([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG]) * 0
        self.period = 1.0 / config.CONTROLLER_FREQUENCY

        self.qA = []

        self.initControllerThread()

    def controller(self):
        """Velocity controller to controll all five legs contiuously.
        """
        lastErrors = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        init = True
        forces = np.empty([self.spider.NUMBER_OF_LEGS, 3])

        while 1:
            if self.killControllerThread:
                break

            startTime = time.perf_counter()

            # Get current data.
            with self.locker:
                self.qA, currents = self.motorDriver.syncReadPositionCurrentWrapper()

            # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
            if init:
                self.lastMotorsPositions = self.qA
                init = False

            for leg in self.spider.LEGS_IDS:
                forces[leg] = self.kinematics.getForceOnLegTip(leg, self.qA[leg], currents[leg])
            
            print(forces[1])

            qD, qDd = self.getQdQddFromQueues()

            # PD controller.
            errors = np.array(qD - self.qA, dtype = np.float32)
            dE = (errors - lastErrors) / self.period
            qCds = self.Kp * errors + self.Kd * dE + qDd
            lastErrors = errors

            # Send new commands to motors.
            if not self.motorDriver.syncWriteMotorsVelocitiesInLegs(self.spider.LEGS_IDS, qCds):
                return False

            elapsedTime = time.perf_counter() - startTime
            while elapsedTime < self.period:
                elapsedTime = time.perf_counter() - startTime
                time.sleep(0)

    def initControllerThread(self):
        """Start a thread with controller function.
        """
        thread = threading.Thread(target = self.controller, name = 'velocity_controller_thread', daemon = False)
        try:
            thread.start()
            print("Controller thread is running.")
        except RuntimeError as re:
            print(re)

    def endControllerThread(self):
        """Set flag to kill a controller thread.
        """
        self.killControllerThread = True

    def getQdQddFromQueues(self):
        """Read current qD and qDd from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            Two 5x3 ndarrays of current qDs and qDds.
        """
        qD = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)
        qDd = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)

        for leg in self.spider.LEGS_IDS:
            try:
                queueData = self.legsQueues[leg].get(False)
                self.lastMotorsPositions[leg] = self.qA[leg]
                if queueData is self.sentinel:
                    qD[leg] = self.lastMotorsPositions[leg]
                    qDd[leg] = ([0, 0, 0])
                else:
                    qD[leg] = queueData[0]
                    qDd[leg] = queueData[1]
            except queue.Empty:
                qD[leg] = self.lastMotorsPositions[leg]
                qDd[leg] = ([0, 0, 0])

        return qD, qDd

    def moveLegAsync(self, legId, goalPosition, origin, duration, trajectoryType, spiderPose = None):
        """Write reference positions and velocities into leg-queue.

        Args:
            legId: Leg id.
            goalPosition: Desired goal positon.
            origin: Origin that goal position is given in. Wheter 'l' for leg-local or 'g' for global.
            duration: Desired movement duration.
            trajectoryType: Type of movement trajectory (bezier or minJerk).
            spiderPose: Spider pose in global origin, used if goalPosition is given in global origin, defaults to None.
        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spiderPose is None.

        Returns:
            False if ValueError is catched during trajectory calculation, True otherwise.
        """
        if origin not in ('l', 'g'):
            raise ValueError("Unknown origin.")
        if origin == 'g' and spiderPose is None:
            raise TypeError("Parameter spiderPose should not be None.")

        self.legsQueues[legId] = queue.Queue()
        self.legsQueues[legId].put(self.sentinel)

        # If goal position is given in global origin, convert it into local.
        if origin == 'g':
            goalPosition = self.matrixCalculator.getLegInLocal(legId, goalPosition, spiderPose)

        with self.locker:
            legCurrentPosition = self.kinematics.legForwardKinematics(legId, self.qA[legId])[:,3][:3]
        try:
            positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, goalPosition, duration, trajectoryType)
        except ValueError as ve:
            print(f'{ve} - {inspect.stack()[0][3]}')
            return False

        qDs, qDds = self.getQdQddLegFF(legId, positionTrajectory, velocityTrajectory)

        for idx, qD in enumerate(qDs):
            self.legsQueues[legId].put([qD, qDds[idx]])
        self.legsQueues[legId].put(self.sentinel)

        return True

    def movePlatformAsync(self, goalPose, duration):
        """Write reference positions and velocities into leg-queues to achieve platform movement. Note that platform cannot be moved
        with less than 4 legs attached to the pins.

        Args:
            goalPose: Desired global goal pose of the spider on the wall.
            duration: Duration of platform movement.

        Raises:
            ValueError: If number of attached legs is less than 4.

        Returns:
            False if ValueError is catched during trajectory calculation, None otherwise.
        """

        attachedLegs = self.gripperController.getIdsOfAttachedLegs()
        if len(attachedLegs) < 4:
            raise ValueError("Number of attached legs is insufficient to move the platform. Should be at least 4.")
        localLegsPositions = np.array([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        for leg in self.spider.LEGS_IDS:
            with self.locker:
                localLegsPositions[leg] = self.kinematics.legForwardKinematics(leg, self.qA[leg])[:,3][:3]

        startPose = self.motorDriver.syncReadPlatformPose(attachedLegs, globalLegsPositions)
        globalLegsPositions = self.matrixCalculator.getLegsInGlobal(attachedLegs, localLegsPositions, startPose)
        try:
            positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(startPose, goalPose, duration, 'minJerk')
        except ValueError as ve:
            print(ve)
            return False
        qDs, qDds = self.getQdQddPlatformFF(attachedLegs, globalLegsPositions, positionTrajectory, velocityTrajectory)
        # Clear leg-queues of attached legs.
        _ = [self.legsQueues[leg].queue.clear() for leg in attachedLegs]
        # Write new values.
        for idx, qD in enumerate(qDs):
            for leg in attachedLegs:
                self.legsQueues[leg].put([qD[leg], qDds[idx][leg]])
        # Put a sentinel objects at the end of each leg-queue, to notify when trajectory ends.
        for leg in attachedLegs:
            self.legsQueues[leg].put(self.sentinel)

    def getQdQddLegFF(self, legId, xD, xDd):
        """Feed-forward calculate and write reference joints positions and velocities for single leg movement into leg-queue.

        :param legIdx: Leg id.
        :param xD: Leg tip reference positions.
        :param xDd: Leg tip reference velocities.
        :return: Matrices of reference joints positions and velocities.
        """
        qDs = np.empty([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDds = np.empty([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        for idx, pose in enumerate(xD):
            qDs[idx] = self.kinematics.legInverseKinematics(legId, pose[:3])
            J = self.kinematics.legJacobi(legId, qDs[idx])
            qDds[idx] = np.dot(np.linalg.inv(J), xDd[idx][:3])

        return qDs, qDds

    def getQdQddPlatformFF(self, legsIds, globalLegsPositions, xD, xDd):
        """Feed forward calculations of reference joints positions and velocities for parallel platform movement.

        :param xD: Platform referenece positions.
        :param xDd: Platform reference velocities.
        :param globalLegsPositions: Legs positions during parallel movement in global origin.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = np.empty([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDd = np.empty([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])

        for idx, pose in enumerate(xD):
            qDFf = self.kinematics.platformInverseKinematics(legsIds, globalLegsPositions, pose[:-1])
            referenceLegsVelocities = self.kinematics.getSpiderToLegReferenceVelocities(xDd[idx])
            qDdFf = []
            for idx, leg in enumerate(legsIds):
                J = self.kinematics.legJacobi(leg, qDFf[idx])
                qDdFf.append(np.dot(np.linalg.inv(J), referenceLegsVelocities[idx]))
            qD[idx] = qDFf
            qDd[idx] = qDdFf

        return qD, qDd

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

        self.comm = serial.Serial('/dev/ttyUSB1', 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.locker = threading.Lock()

        self.initReadingThread()

        self.handshake()

    def initReadingThread(self):
        """Initialize reading data thread.
        """
        readingThread = threading.Thread(target = self.readData, name = "serial_reading_thread", daemon = True)
        readingThread.start()

    def readData(self):
        """Constantly receiving data from Arduino.
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

        :param msg: Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        with self.locker:
            self.comm.write(msg)

    def moveGripper(self, legId, command):
        """Send command to move gripper on selected leg.

        :param legId: Leg id.
        :param command: Command to execute on gripper.
        """
        if (command == self.OPEN_COMMAND or command == self.CLOSE_COMMAND):
            msg = command + str(legId) + "\n"
            self.sendData(msg)

    def openGrippersAndWait(self, legsIds):
        """Open grippers on given legs and wait for them to come in open state.

        :param legsIds: Legs ids.
        :return: True when all grippers are opened.
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
        """Get switches states.

        :return: String of five elements, each representing state of the switch on the leg.
        """
        recMsg = ''
        while not len(recMsg) == self.RECEIVED_MESSAGE_LENGTH:
            with self.lock:
                recMsg = self.receivedMessage
        return recMsg[5:]

    def getIdsOfAttachedLegs(self):
        """Get ids of those legs, that are attached to pins. Leg is attached if switch and gripper are both closed.
        """
        recMsg = ''
        while not len(recMsg) == self.RECEIVED_MESSAGE_LENGTH:
            with self.lock:
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
        print("Handshake with Arduino successfull.")




