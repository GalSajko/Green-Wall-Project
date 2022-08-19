
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
import mappers

class VelocityController:
    """ Class for velocity-control of spider's movement. All legs are controlled with same controller, but can be moved separately and independently
    from other legs. Reference positions for each legs are writen in legs-queues. On each control-loop controller takes first values from all of the legs-queues.
    """
    def __init__ (self):
        self.matrixCalculator = calculations.MatrixCalculator()
        self.kinematics = calculations.Kinematics()
        self.dynamics = calculations.Dynamics()
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
        self.queuesReadyFlags = [False] * self.spider.NUMBER_OF_LEGS

        self.Kp = np.array([[20, 20, 20]] * self.spider.NUMBER_OF_LEGS)
        # self.Kp = np.array([[5, 5, 5]] * self.spider.NUMBER_OF_LEGS)
        self.Kd = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS)
        self.period = 1.0 / config.CONTROLLER_FREQUENCY

        self.qA = []
        self.fA = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        self.rgValues = np.zeros(self.spider.NUMBER_OF_LEGS)
        self.doOffload = False

        self.initControllerThread()

    def controller(self):
        """Velocity controller to controll all five legs continuously. Runs in separate thread.
        """
        lastQErrors = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        lastFErrors = np.zeros(3)
        init = True

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
                self.fA[leg] = self.dynamics.getForceOnLegTip(leg, self.qA[leg], currents[leg])
                # with self.locker:
                #     self.rgValues[leg] = self.dynamics.getForceEllipsoidLengthInGivenDirection(leg, self.qA[leg], self.spider.SPIDER_GRAVITY_VECTOR)

            qD, qDd = self.getQdQddFromQueues()            

            # Position PD controller.
            qErrors = np.array(qD - self.qA, dtype = np.float32)
            dQe = (qErrors - lastQErrors) / self.period
            qCds = self.Kp * qErrors + self.Kd * dQe + qDd
            lastQErrors = qErrors

            # Force PD controller.
            with self.locker:
                startForceController = self.doOffload
            if startForceController:
                legId = 0
                fD = np.zeros(3)
                forceErrors = np.array(fD - self.fA[legId], dtype = np.float32)
                # dFe = (forceErrors - lastFErrors) / self.period
                dXOff = 0.1 * forceErrors
                dQOff = np.dot(np.linalg.inv(self.kinematics.spiderBaseToLegTipJacobi(legId, self.qA[legId])), dXOff)
                qCds[legId] += dQOff

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
            tuple: Two 5x3 numpy.ndarrays of current qDs and qDds.
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

    def moveLegAsync(self, legId, goalPosition, origin, duration, trajectoryType, spiderPose = None, offset = False):
        """Write reference positions and velocities into leg-queue.

        Args:
            legId (int): Leg id.
            goalPosition (list): 1x3 array of  desired x, y, and z goal positons.
            origin (str): Origin that goal position is given in. Wheter 'l' for leg-local or 'g' for global.
            duration (float): Desired movement duration.
            trajectoryType (str): Type of movement trajectory (bezier or minJerk).
            spiderPose (list, optional): Spider pose in global origin, used if goalPosition is given in global origin. Defaults to None.
            offset(bool, optional): If true, move leg relatively on current position, goalPosition should be given as desired offset. Defaults to False.

        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spiderPose is None.

        Returns:
            bool: False if ValueError is catched during trajectory calculation, True otherwise.
        """
        if origin not in ('l', 'g'):
            raise ValueError("Unknown origin.")
        if origin == 'g' and spiderPose is None:
            raise TypeError("Parameter spiderPose should not be None.")

        self.legsQueues[legId] = queue.Queue()
        self.legsQueues[legId].put(self.sentinel)

        # If goal position is given in global origin, convert it into local.
        if origin == 'g' and offset is False:
            goalPosition = self.matrixCalculator.getLegInLocal(legId, goalPosition, spiderPose)

        with self.locker:
            legCurrentPosition = self.kinematics.legForwardKinematics(legId, self.qA[legId])[:,3][:3]

        # If goal position is given as offset in global direction, calculate global goal position by adding offset to global current position and than convert it into local.
        if offset:
            legCurrentGlobalPosition = self.matrixCalculator.getLegsInGlobal([legId], [legCurrentPosition], spiderPose)
            globalGoalPosition = legCurrentGlobalPosition + np.array(goalPosition)
            goalPosition = self.matrixCalculator.getLegInLocal(legId, globalGoalPosition, spiderPose)

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
    
    def moveLegsSync(self, legsIds, goalPositions, origin, duration, trajectoryType, spiderPose = None, offset = False):
        """Write reference positions and velocities in any number (within 5) of leg-queues. Legs start to move at the same time. 
        Meant for moving a platform.

        Args:
            legsIds (list): Legs ids.
            goalPositions (list): nx3x1 array of goal positions, where n is number of legs.
            origin (str): Origin that goal positions are given in, 'g' for global or 'l' for local.
            duration (float): Desired duration of movements.
            trajectoryType (str): Type of movement trajectory (bezier or minJerk).
            spiderPose (list, optional): Spider pose in global origin, used if goalPositions are given in global. Defaults to None.
            offset (bool, optional): If true, move legs relatively to current positions, goalPositions should be given as desired offsets in global origin. Defaults to False.

        Raises:
            ValueError: If origin is unknown.
            TypeError: If origin is global and spider pose is not given.
            ValueError: If number of used legs and given goal positions are not the same.

        Returns:
            bool: False if ValueError is catched during trajectory calculations, True otherwise.
        """
        if origin not in ('l', 'g'):
            raise ValueError(f"Unknown origin {origin}.")
        if origin == 'g' and spiderPose is None:
            raise TypeError("If origin is global, spider pose should be given.")  
        if len(legsIds) != len(goalPositions):
            raise ValueError("Number of legs and given goal positions should be the same.")
        
        # Stop all of the given legs.
        for leg in legsIds:
            self.legsQueues[leg] = queue.Queue()
            self.legsQueues[leg].put(self.sentinel)
        
        qDs = []
        qDds = []
        for idx, leg in enumerate(legsIds):
            if origin == 'g' and offset is False:
                goalPositions[idx] = self.matrixCalculator.getLegInLocal(leg, goalPositions[idx], spiderPose)
            with self.locker:
                legCurrentPosition = self.kinematics.legForwardKinematics(leg, self.qA[leg])[:,3][:3]
            if offset:
                legCurrentGlobalPosition = self.matrixCalculator.getLegsInGlobal([leg], [legCurrentPosition], spiderPose)
                globalGoalPosition = legCurrentGlobalPosition + np.array(goalPositions[idx])
                goalPositions[idx] = self.matrixCalculator.getLegInLocal(leg, globalGoalPosition, spiderPose)
            try:
                positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, goalPositions[idx], duration, trajectoryType)
            except ValueError as ve:
                print(f'{ve} - {inspect.stack()[0][3]}')
                return False
            
            qD, qDd = self.getQdQddLegFF(leg, positionTrajectory, velocityTrajectory)
            qDs.append(qD)
            qDds.append(qDd)
        
        for i in range(len(qDs[0])):
            for idx, leg in enumerate(legsIds):
                self.legsQueues[leg].put([qDs[idx][i], qDds[idx][i]])
        for leg in legsIds:
            self.legsQueues[leg].put(self.sentinel)
        
        return True
    
    def offloadSelectedLeg(self, legId):
        with self.locker:
            self.doOffload = True
   
    def moveLegAndGripper(self, legId, goalPosition, duration, spiderPose):
        """Open gripper, move leg to the new pin and close the gripper.

        Args:
            legId (int): Leg id.
            goalPosition (list): 1x3 array of global x, y and z position of leg.
            duration (float): Duration of movement.
            spiderPose (list): 1x4 array of global x, y, z and rotZ pose of spider's body.
        """
        detachOffset = 0.02
        attachTime = 1
        detachTime = 1

        with self.locker:
            legPosition = self.motorDriver.syncReadMotorsPositionsInLegs(legId, True, 'g')

        detachPosition = np.copy(legPosition)
        detachPosition[2] += detachOffset
        attachPosition = self.matrixCalculator.getLegsApproachPositionsInGlobal([legId], spiderPose, [goalPosition])

        # If leg is attached, open a gripper.
        if legId in self.gripperController.getIdsOfAttachedLegs():
            self.gripperController.openGrippersAndWait(legId)

        # Move from pin to detach position, to attach position and finally to new pin.
        self.moveLegAsync(legId, detachPosition, 'g', detachTime, 'minJerk', spiderPose)
        time.sleep(detachTime + 0.01)
        self.moveLegAsync(legId, attachPosition, 'g', duration, 'bezier', spiderPose)
        time.sleep(duration + 0.01)
        self.moveLegAsync(legId, goalPosition, 'g', attachTime, 'minJerk', spiderPose)
        time.sleep(attachTime + 0.01)

        # Close gripper.
        self.gripperController.moveGripper(legId, self.gripperController.CLOSE_COMMAND)

    # TODO: write wrapper for moving legs so that spider will reach desired pose, instead of this function.
    def movePlatformAsync(self, goalPose, duration, globalLegsPositions):
        """Write reference positions and velocities into leg-queues to achieve platform movement. Note that platform cannot be moved
        with less than 4 legs attached to the pins.

        Args:
            goalPose (list): Desired global goal pose of the spider on the wall. Could be given as 1x4 array, representing xyzy values or 1x6 array, representing xyzrpy values.
            duration (float): Duration of platform movement.
            globalLegsPositions (list): nx3 array of global positions of legs, which are to be used for platform movement, where n should not be smaller than 4.

        Raises:
            ValueError: If number of attached legs is less than 4 or does not match the length of globalLegsPositions parameter.

        Returns:
            bool: False if ValueError is catched during trajectory calculation, True otherwise.
        """
        usedLegs = self.gripperController.getIdsOfAttachedLegs()
        if len(usedLegs) < 4:
            raise ValueError("Number of attached legs is insufficient to move the platform. Should be at least 4.")
        if len(usedLegs) != len(globalLegsPositions):
            raise ValueError("Number of given global legs positions is not the same as number of attached legs!")

        for leg in usedLegs:
            self.legsQueues[leg] = queue.Queue()
            self.legsQueues[leg].put(self.sentinel)

        localLegsPositions = np.empty([len(usedLegs), 3])
        for leg in usedLegs:
            with self.locker:
                localLegsPositions[leg] = self.kinematics.legForwardKinematics(leg, self.qA[leg])[:3][:,3]

        startPose = self.motorDriver.syncReadPlatformPose(usedLegs, globalLegsPositions)

        try:
            positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(startPose, goalPose, duration, 'minJerk')
        except ValueError as ve:
            print(ve)
            return False

        qDs, qDds = self.getQdQddPlatformFF(usedLegs, globalLegsPositions, positionTrajectory, velocityTrajectory)

        # Write new values.
        for idx, qD in enumerate(qDs):
            for leg in usedLegs:
                self.legsQueues[leg].put([qD[leg], qDds[idx][leg]])
        # Put a sentinel objects at the end of each leg-queue, to notify when trajectory ends.
        for leg in usedLegs:
            self.legsQueues[leg].put(self.sentinel)

        return True

    def getQdQddLegFF(self, legId, xD, xDd):
        """(Feed-forward) calculate and write reference joints positions and velocities for single leg movement into leg-queue.

        Args:
            legId (int): Leg id.
            xD (list): nx7 array of position (6 values for xyzrpy) trajectory and time stamps (7th value in a row), where n is number of steps on trajectory.
            xDd (list): nx6 array for xyzrpy values of velocity trajectory, where n is number of steps on trajectory.

        Returns:
            tuple: Two nx3 numpy.ndarrays of reference joints positions and velocities, where n is number of steps on trajectory.
        """

        qDs = np.zeros([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDds = np.zeros([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        for idx, pose in enumerate(xD):
            qDs[idx] = self.kinematics.legInverseKinematics(legId, pose[:3])
            J = self.kinematics.legJacobi(legId, qDs[idx])
            qDds[idx] = np.dot(np.linalg.inv(J), xDd[idx][:3])

        return qDs, qDds

    # TODO: with wrapper for moving separate legs, this function will not be needed.
    def getQdQddPlatformFF(self, legsIds, globalLegsPositions, xD, xDd):
        """Feed forward calculations of reference joints positions and velocities for parallel platform movement.

        :param xD: Platform referenece positions.
        :param xDd: Platform reference velocities.
        :param globalLegsPositions: Legs positions during parallel movement in global origin.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = np.zeros([len(xD), len(legsIds), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDd = np.zeros([len(xD), len(legsIds), self.spider.NUMBER_OF_MOTORS_IN_LEG])

        for idx, pose in enumerate(xD):
            qDFf = self.kinematics.platformInverseKinematics(legsIds, globalLegsPositions, pose[:-1])
            referenceLegsVelocities = self.kinematics.getSpiderToLegReferenceVelocities(legsIds, xDd[idx])
            qDdFf = np.zeros_like(qDFf)
            for idx, leg in enumerate(legsIds):
                J = self.kinematics.legJacobi(leg, qDFf[idx])
                qDdFf[idx] = np.dot(np.linalg.inv(J), referenceLegsVelocities[idx])
            qD[idx] = qDFf
            qDd[idx] = qDdFf

        return qD, qDd
    
    def moveGripperWrapper(self, legId, command):
        self.gripperController.moveGripper(legId, command)

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
        if (command == self.OPEN_COMMAND or command == self.CLOSE_COMMAND):
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

        self.comm = serial.Serial('/dev/ttyUSB0', 115200, timeout = 0)
        self.receivedMessage = ""

        self.handshake()

    def sendData(self, msg):
        """Send data to Arduino.

        Args:
            msg (str): Message to send.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        self.com.write(msg)
    
    def pumpControll(self, command, pumpId):
        """Controll water pump - turn it on of off.

        Args:
            command (str): Command to execute on water pump - '1' for on, '0' for off.
            pumpId (int): Water pump id.
        """
        if command in (self.ON_COMMAND, self.OFF_COMMAND):
            msg = command + str(pumpId) + '\n'
            self.sendData(msg)
    
    def handshake(self):
        """Handshake procedure with Arduino.
        """
        self.sendData(self.INIT_MESSAGE)
        time.sleep(0.01)
        while not self.receivedMessage == self.INIT_RESPONSE:
            self.sendData(self.INIT_MESSAGE)
        print("Handhsake with water-pump Arduino succesfull.")
    