
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

        self.Kp = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS) * 20.0
        self.Kd = np.array([[1.0, 1.0, 1.0]] * self.spider.NUMBER_OF_LEGS) * 0.25
        self.period = 1.0 / config.CONTROLLER_FREQUENCY

        self.qA = []
        self.fA = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        # self.fD = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        self.fD = np.array([5.0, 0.0, 0.0])

        self.KpForce = 0.01

        self.initControllerThread()

    def controller(self):
        """Velocity controller to controll all five legs continuously. Runs in separate thread.
        """
        lastQErrors = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        lastFErrors = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDd = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qD = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDdF = np.zeros([self.spider.NUMBER_OF_LEGS, 3])
        qDf = np.zeros([self.spider.NUMBER_OF_LEGS , 3])
        init = True

        fBuffer = np.zeros([10, 5, 3])
        fMean = np.zeros([5, 3])
        counter = 0
        while True:
            if self.killControllerThread:
                break
 
            startTime = time.perf_counter()

            # Get current data.
            with self.locker: 
                currentAngles, currents = self.motorDriver.syncReadAnglesAndCurrentsWrapper()
                self.qA = currentAngles  
                # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
                if init:
                    self.lastMotorsPositions = currentAngles
                    init = False
                    
            # Force-velocity P controller.
            self.fA = self.dynamics.getForcesOnLegsTips(currentAngles, currents)
            fBuffer[counter] = self.fA
            counter += 1

            if counter == len(fBuffer):
                fMean = np.mean(fBuffer, axis = 0)
                counter = 0

            fErrors = self.fD - fMean
            dXSpider = fErrors * self.KpForce
            offsets = dXSpider * self.period
        
            # Test only on first leg.
            leg = 4
            # Read leg's current pose in spider's origin.
            xSpider = self.kinematics.spiderBaseToLegTipForwardKinematics(leg, currentAngles[leg])
            # Add calculated offset.
            xSpider[:3][:,3] += offsets[leg]
            # Transform into leg's origin.
            xD = np.dot(np.linalg.inv(self.spider.T_ANCHORS[leg]), xSpider)[:3][:,3]
            # Calculate values in joints space.
            qDf[leg] = self.kinematics.legInverseKinematics(leg, xD)
            qDdF[leg] = np.dot(np.linalg.inv(self.kinematics.spiderBaseToLegTipJacobi(leg, currentAngles[leg])), dXSpider[leg])
            
            qD = qDf
            qDd = qDdF

            # Position-velocity PD controller.
            qErrors = np.array(qD - currentAngles, dtype = np.float32)
            dQe = (qErrors - lastQErrors) / self.period
            qCds = self.Kp * qErrors + self.Kd * dQe + qDd
            lastQErrors = qErrors

            for idx in range(len(qCds[leg])):
                if qCds[leg][idx] > 1.0:
                    qCds[leg][idx] = 1.0
                elif qCds[leg][idx] < -1.0:
                    qCds[leg][idx] = -1.0

            # Send new commands to motors.
            with self.locker:
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(self.spider.LEGS_IDS, qCds)

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
                with self.locker:
                    self.lastMotorsPositions[leg] = self.qA[leg]
                if queueData is self.sentinel:
                    qD[leg] = self.lastMotorsPositions[leg]
                    qDd[leg] = ([0, 0, 0])
                else:
                    qD[leg] = queueData[0]
                    qDd[leg] = queueData[1]
            except queue.Empty:
                with self.locker:
                    qD[leg] = self.lastMotorsPositions[leg]
                qDd[leg] = ([0, 0, 0])

        return qD, qDd

    def moveLegAsync(self, legId, goalPositionOrOffset, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
        """Write reference positions and velocities into leg-queue.

        Args:
            legId (int): Leg id.
            goalPositionOrOffset (list): 1x3 array of  desired x, y, and z goal positons or offsets.
            origin (str): Origin that goal position is given in. Wheter 'l' for leg-local or 'g' for global.
            duration (float): Desired movement duration.
            trajectoryType (str): Type of movement trajectory (bezier or minJerk).
            spiderPose (list, optional): Spider pose in global origin, used if goalPositionOrOffset is given in global origin. Defaults to None.
            offset(bool, optional): If true, move leg relatively on current position, goalPositionOrOffset should be given as desired offset. Defaults to False.

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

        with self.locker:
            currentAngles = self.qA[legId]
        legCurrentPosition = self.kinematics.legForwardKinematics(legId, currentAngles)[:,3][:3]

        if origin == 'l':
            localGoalPosition = goalPositionOrOffset
            if isOffset:
                localGoalPosition += legCurrentPosition
        elif origin == 'g':
            if not isOffset:
                localGoalPosition = self.matrixCalculator.getLegInLocal(legId, goalPositionOrOffset, spiderPose)
            else:
                localGoalPosition = legCurrentPosition + self.matrixCalculator.getGlobalDirectionInLocal(legId, spiderPose, goalPositionOrOffset)

        try:
            positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)
        except ValueError as ve:
            print(f'{ve} - {inspect.stack()[0][3]}')
            return False

        qDs, qDds = self.getQdQddLegFF(legId, positionTrajectory, velocityTrajectory)

        for idx, qD in enumerate(qDs):
            self.legsQueues[legId].put([qD, qDds[idx]])
        self.legsQueues[legId].put(self.sentinel)

        return True
    
    def moveLegsSync(self, legsIds, goalPositionsOrOffsets, origin, duration, trajectoryType, spiderPose = None, offset = False):
        """Write reference positions and velocities in any number (within 5) of leg-queues. Legs start to move at the same time. 
        Meant for moving a platform.

        Args:
            legsIds (list): Legs ids.
            goalPositionsOrOffsets (list): nx3x1 array of goal positions, where n is number of legs.
            origin (str): Origin that goal positions are given in, 'g' for global or 'l' for local.
            duration (float): Desired duration of movements.
            trajectoryType (str): Type of movement trajectory (bezier or minJerk).
            spiderPose (list, optional): Spider pose in global origin, used if goalPositionsOrOffsets are given in global. Defaults to None.
            offset (bool, optional): If true, move legs relatively to current positions, goalPositionsOrOffsets should be given as desired offsets in global origin. Defaults to False.

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
        if len(legsIds) != len(goalPositionsOrOffsets):
            raise ValueError("Number of legs and given goal positions should be the same.")
        
        # Stop all of the given legs.
        for leg in legsIds:
            self.legsQueues[leg] = queue.Queue()
            self.legsQueues[leg].put(self.sentinel)
        
        qDs = []
        qDds = []

        for idx, leg in enumerate(legsIds):
            with self.locker:
                currentAngles = self.qA[leg]
            # Get current leg position in local.
            legCurrentPosition = self.kinematics.legForwardKinematics(leg, currentAngles)[:,3][:3]

            if origin == 'l':
                localGoalPosition = goalPositionsOrOffsets[idx]
                if offset:
                    localGoalPosition += legCurrentPosition
            # If goal position is given in global origin, convert it into local.
            elif origin == 'g':
                globalGoalPosition = goalPositionsOrOffsets[idx]
                if not offset:
                    localGoalPosition = self.matrixCalculator.getLegInLocal(leg, globalGoalPosition, spiderPose)
                else:
                    localGoalPosition = legCurrentPosition + self.matrixCalculator.getGlobalDirectionInLocal(leg, spiderPose, globalGoalPosition)
            
            try:
                positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)
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
    
    def offloadSelectedLeg(self, legId, legsGlobalPositions = None):
        """Offloading selected legs by turning it off and than on again.

        Args:
            legId (int): Leg id.
            legsGlobalPositions (list, optional): 5x3 array of global legs positions. Defaults to None.

        Returns:
            list: 1x6 array of new xyzrpy spider's pose (after offloading).
        """
        otherLegs = self.spider.LEGS_IDS.copy()
        otherLegs.remove(legId)

        with self.locker:
            self.motorDriver.disableLegs(legId)
        
        time.sleep(1)
        with self.locker:
            self.lastMotorsPositions[legId] = self.qA[legId]
            self.motorDriver.enableLegs(legId)
            
        if legsGlobalPositions is not None:
            with self.locker:
                currentAngles = self.qA
            newPose = self.matrixCalculator.getSpiderPose([0, 1, 2, 3, 4], legsGlobalPositions, currentAngles)  

        return newPose
   
    def moveLegAndGripper(self, legId, goalPosition, duration, spiderPose, legsGlobalPositions):
        """Open gripper, move leg to the new pin and close the gripper.

        Args:
            legId (int): Leg id.
            goalPosition (list): 1x3 array of global x, y and z position of leg.
            duration (float): Duration of movement.
            spiderPose (list): 1x4 array of global x, y, z and rotZ pose of spider's body.
        """
        attachTime = 1
        detachTime = 1

        with self.locker:
            legPosition = self.motorDriver.syncReadMotorsPositionsInLegs([legId], True, 'l')
        globalLegPosition = self.matrixCalculator.getLegsInGlobal([legId], legPosition, spiderPose)[0]
        detachPosition = self.matrixCalculator.getLegsApproachPositionsInGlobal([legId], spiderPose, [globalLegPosition], offset = 0.1)

        # If leg is attached, open a gripper.
        if legId in self.gripperController.getIdsOfAttachedLegs():
            self.gripperController.openGrippersAndWait([legId])

        # Move leg on detach position.
        self.moveLegAsync(legId, detachPosition, 'g', detachTime, 'minJerk', spiderPose)
        time.sleep(detachTime + 0.5)

        # Measure new spider pose after detaching the leg.
        usedLegs = self.spider.LEGS_IDS.copy()
        usedLegs.remove(legId)
        legsGlobalPositions = np.delete(legsGlobalPositions, legId, 0)
        with self.locker:
            currentAngles = self.qA
        newPose = self.matrixCalculator.getSpiderPose(usedLegs, legsGlobalPositions, currentAngles)
        
        # Move leg on attach position.
        attachPosition = self.matrixCalculator.getLegsApproachPositionsInGlobal([legId], newPose, [goalPosition])
        self.moveLegAsync(legId, attachPosition, 'g', duration, 'bezier', newPose)
        time.sleep(duration + 0.5)

        self.moveLegAsync(legId, goalPosition, 'g', attachTime, 'minJerk', newPose)
        time.sleep(attachTime + 0.5)

        # Close gripper.
        self.gripperController.moveGripper(legId, self.gripperController.CLOSE_COMMAND)

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

    def startForceControl(self):
        """Start force controller inside main velocity controller loop.
        """
        with self.locker:
            self.startForceController = True

    def moveGrippersWrapper(self, legsIds, command):
        """Wrapper function for moving the grippers on selected legs.

        Args:
            legsIds (list): List of legs ids.
            command (str): Command to execute on grippers (same for all selected gripepers).
        """
        for leg in legsIds:
            self.gripperController.moveGripper(leg, command)
        
    def readLegsPositionsWrapper(self, legsIds, origin = 'l', spiderPose = None, returnAngles = False):
        """Wrapper function for reading legs positions.

        Args:
            legsIds (list): List of legs ids.
            origin (str, optional): Origin in which to read legs positions, 'l' for local, 's' for spider's and 'g' for global. Defaults to 'l'.
            spiderPose (list, optional): Spider's pose given as xyzr or xyzrpy in global origin. Defaults to None.

        Raises:
            ValueError: If given origin is global and spider's pose is not given.

        Returns:
            list: nx3 list of x, y, z positions of legs, where n is number of legs.
        """
        if origin == 'g' and spiderPose is None:
            raise ValueError("Spider pose should be given, if selected origin is global.")

        legsPositions = np.zeros([len(legsIds), 3])
        currentAngles = np.zeros([len(legsIds), 3])
        for idx, leg in enumerate(legsIds):
            with self.locker:
                currentAngles[idx] = self.qA[leg]
            if origin == 'l' or origin == 'g': 
                legsPositions[idx] = self.kinematics.legForwardKinematics(leg, currentAngles[idx])[:3][:,3]
            elif origin == 's':
                legsPositions[idx] = self.kinematics.spiderBaseToLegTipForwardKinematics(leg, currentAngles[idx])[:3][:,3]
        if origin == 'g':
            globalLegsPositions = self.matrixCalculator.getLegsInGlobal(legsIds, legsPositions, spiderPose)
            return globalLegsPositions

        if returnAngles:
            return legsPositions, currentAngles

        return legsPositions

    def readSpiderPoseWrapper(self, legsIds, legsGlobalPositions):
        """Wrapper function for reading spider's pose.

        Args:
            legsIds (list): List of legs ids, used for reading spider's pose.
            legsGlobalPositions (list): Global positions of used legs.

        Raises:
            ValueError: If number of given legs is less than 3.
            ValueError: If number of used legs and given legs positions is not the same.

        Returns:
            list: 1x6 list of spider's pose, given as xyzrpy in global origin.
        """
        if len(legsIds) < 3:
            raise ValueError("At least three legs are needed to read spider's pose.")

        with self.locker:
            currentAngles = self.qA
        pose = self.matrixCalculator.getSpiderPose(legsIds, legsGlobalPositions, currentAngles)
        return pose

    def disableEnableLegsWrapper(self, legId, action):
        if action == 'd':
            with self.locker:
                self.motorDriver.disableLegs(legId)
        elif action == 'e':
            with self.locker:
                self.motorDriver.enableLegs(legId)

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
    
    def handshake(self):
        """Handshake procedure with Arduino.
        """
        self.sendData(self.INIT_MESSAGE)
        time.sleep(0.01)
        while not self.receivedMessage == self.INIT_RESPONSE:
            self.sendData(self.INIT_MESSAGE)
            time.sleep(0.01)

        print("Handshake with water-pump Arduino successfull.")
    