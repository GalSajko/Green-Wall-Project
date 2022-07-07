import queue
import numpy as np
import time
import serial
import threading
from queue import Queue
import os
from numba import jit

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
        
        self.legsQueues = [Queue() for i in range(self.spider.NUMBER_OF_LEGS)]
        self.sentinel = object()
        self.lastMotorsPositions = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        self.locker = threading.Lock()
        self.killControllerThread = False
        
        self.Kp = np.array([[30, 30, 30]] * self.spider.NUMBER_OF_LEGS)
        self.Kd = np.ones([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        self.lastErrors = np.zeros([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        self.period = 1.0 / config.CONTROLLER_FREQUENCY
        self.init = True

        self.qA = []

        self.initControllerThread()

    def controller(self):
        """Velocity controller to controll all five legs contiuously.
        """
        while 1:
            if self.killControllerThread:
                break
            # Get current data.
            startTime = time.perf_counter()
            with self.locker:
                self.qA, _ = self.motorDriver.syncReadPositionCurrentWrapper()
                # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
                if self.init:
                    # with self.locker:
                    self.lastMotorsPositions = self.qA
                    self.init = False
              
            qD, qDd = self.getQdQddFromQueues()

            # PD controller.
            with self.locker:
                errors = np.array(qD - self.qA, dtype = np.float32)
            dE = (errors - self.lastErrors) / self.period
            qCds = self.Kp * errors + self.Kd * dE + qDd
            self.lastErrors = errors

            # Send new commands to motors.
            with self.locker:
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
        qD = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDd = np.empty([self.spider.NUMBER_OF_LEGS, self.spider.NUMBER_OF_MOTORS_IN_LEG])

        for leg in self.spider.LEGS_IDS:
            if self.legsQueues[leg].empty():
                with self.locker:
                    qD[leg] = self.lastMotorsPositions[leg]
                qDd[leg] = ([0, 0, 0])
                continue

            queueData = self.legsQueues[leg].get()
            if queueData is not self.sentinel:
                qD[leg] = queueData[0]
                qDd[leg] = queueData[1]
                continue

            with self.locker:
                self.lastMotorsPositions[leg] = self.qA[leg]
                qD[leg] = self.qA[leg]
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
        if origin != 'l' and origin != 'g':
            raise ValueError("Unknown origin.") 
        if origin == 'g' and spiderPose is None:
            raise TypeError("Parameter spiderPose should not be None.")

        # If goal position is given in global origin, convert it into local.
        if origin == 'g':
            goalPosition = self.matrixCalculator.getLegInLocal(legId, goalPosition, spiderPose)

        with self.locker:
            # legCurrentPosition = self.motorDriver.syncReadMotorsPositionsInLegs([legId], True, 'leg')[0]
            legCurrentPosition = self.kinematics.legForwardKinematics(legId, self.qA[legId])[:,3][:3]
        try:
            positionTrajectory, velocityTrajectory = self.trajectoryPlanner.calculateTrajectory(legCurrentPosition, goalPosition, duration, trajectoryType)
        except ValueError as ve:
            print(ve)
            return False

        # Clear current leg-queue.
        with legsQueues[legId].mutex:
            self.legsQueues[legId].queue.clear()
        qDs, qDds = self.getQdQddLegFF(legId, positionTrajectory, velocityTrajectory)

        # Write new values.
        for idx, qD in enumerate(qDs):
            self.legsQueues[legId].put([qD, qDds[idx]])
        # Put a sentinel object at the end, to notify when trajectory ends.
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
        with self.locker:
            localLegsPositions = self.motorDriver.syncReadMotorsPositionsInLegs(attachedLegs, True, 'leg')
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
            [self.legsQueues[leg].put([qD[leg], qDds[idx][leg]]) for leg in attachedLegs]
        # Put a sentinel objects at the end of each leg-queue, to notify when trajectory ends.
        _ = [self.legsQueues[leg].put(self.sentinel) for leg in attachedLegs]
    
    def getQdQddLegFF(self, legId, xD, xDd):
        """Feed forward calculations of reference joints positions and velocities for single leg movement.

        :param legIdx: Leg id.
        :param xD: Leg tip reference positions.
        :param xDd: Leg tip reference velocities.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = np.empty([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])
        qDd = np.empty([len(xD), self.spider.NUMBER_OF_MOTORS_IN_LEG])

        for idx, pose in enumerate(xD):
            qDFf = self.kinematics.legInverseKinematics(legId, pose[:3])
            J = self.kinematics.legJacobi(legId, qDFf)
            qDdFf = np.dot(np.linalg.inv(J), xDd[idx][:3])
            qD[idx] = qDFf
            qDd[idx] = qDdFf
            time.sleep(0)
        
        return qD, qDd
    
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

    def moveLegs(self, legsIds, trajectories, velocities, grippersCommands = None):
        """Move any number of legs (within number of spiders legs) along given trajectories.

        :param ledIds: Array of legs ids, if value is 5 than all legs are selected.
        :param trajectory: 2D array of legs tips trajectories.
        :param velocity: 2D array of legs tips velocities.
        :return: True if movements were successfull, false otherwise.
        """
        if legsIds == 5:
            legsIds = [0, 1, 2, 3, 4]
        if len(legsIds) != len(trajectories) and len(legsIds) != len(velocities):
            raise ValueError("Invalid parameters!")

        qDs = []
        qDds = []
        # Time steps of all trajectories should be the same (it is defined in trajectory planner).
        timeStep = trajectories[0][:,-1][1] - trajectories[0][:,-1][0]

        self.motorDriver.clearGroupSyncWriteParams()

        for idx, leg in enumerate(legsIds):
            qD, qDd = self.getQdQddLegFF(leg, trajectories[idx], velocities[idx])
            qDs.append(qD)
            qDds.append(qDd)

        # Index of longer trajectory:
        longerIdx = trajectories.index(max(trajectories, key = len))
        lastErrors = np.zeros([len(legsIds), 3])
        Kp = np.array([5, 15, 15])
        Kd = np.array([1, 1, 1])

        # Open grippers (if needed) and wait for them to open before moving legs.
        if grippersCommands == [self.gripperController.OPEN_COMMAND] * len(legsIds):
            self.gripperController.openGrippersAndWait(legsIds)

        # Use indexes of longest trajectory.
        for i, _ in enumerate(trajectories[longerIdx]):
            startTime = time.time()
            # Read motors positions in all legs.
            qA = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds)
            qCds = []
            for l, leg in enumerate(legsIds):
                # If index is still whithin boundaries of current trajectory.
                if i < len(trajectories[l]) - 1:
                    currentQd = qDs[l][i]
                    currentQdd = qDds[l][i]
                    currentQa = qA[l]
                    error = currentQd - currentQa
                    dE = (error - lastErrors[l]) / timeStep
                    qCd = Kp * error + Kd * dE + currentQdd
                    lastErrors[l] = error
                # If not, stop the leg.
                elif i >= len(trajectories[l]) - 1:
                    qCd = [0, 0, 0]
                    if grippersCommands is not None and grippersCommands[l] == self.gripperController.CLOSE_COMMAND: 
                        self.gripperController.moveGripper(leg, self.gripperController.CLOSE_COMMAND)
                qCds.append(qCd)
                
            if not self.motorDriver.syncWriteMotorsVelocitiesInLegs(legsIds, qCds, i == 0):
                return False
            
            try:
                time.sleep(timeStep - (time.time() - startTime))
            except:
                time.sleep(0)
   
        self.motorDriver.clearGroupSyncWriteParams()

        return True

    def moveLegsWrapper(self, legsIds, globalGoalPositions, spiderPose, durations, gripperCommands = None, readLegs = True, globalStartPositions = None, trajectoryType = "bezier"):
        """Wrapper function for moving any number of legs (within number of spiders legs) to desired pins on the wall. 
        Includes transformation from global to legs-local pins positions and computing trajectories.

        :param legsIds: Array of legs ids, if value is 5 than all legs are selected.
        :param globalGoalPositions: Array of global goal positions (represents desired pins positions).
        :param spiderPose: Array of xyzrpy global position of spider's body.
        :param durations: Array of durations for legs movements in seconds.
        :param readLegs: If true, read local leg position on the beginning of each leg movement. Otherwise take FF calculations of global legs positions.
        :param globalStartPositions: Array of FF calculations of starting global legs positions.
        :param trajectoryType: 'bezier' for bezier curve or 'minJerk' for minimal jerk trajectory.
        :raises ValueError: Exception is thrown if legsIds, globalGoalPositions and durations parameters dont have same length.
        :return: True if movements were successfull, false otherwise.
        """
        if legsIds == 5:
            legsIds = [0, 1, 2, 3, 4]
        if len(legsIds) != len(globalGoalPositions) or len(legsIds) != len(durations):
            raise ValueError("Invalid values of legsIds, goalGlobalPositions or durations parameters.")
        if not readLegs and not np.array(globalStartPositions).any():
            raise ValueError("Legs starting positions are not given!")
        if not readLegs and (len(globalGoalPositions) != len(globalStartPositions)):
            raise ValueError("Invalid values of globalStartPositions parameter.")

        # Transformation matrix of global spider's pose.
        T_GS = self.matrixCalculator.xyzRpyToMatrix(spiderPose)
        # Calculate trajectories for each leg movement from local positions.
        trajectories = []
        velocities = []

        if readLegs:
            startPosition = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds, True)
        for legIdx, leg in enumerate(legsIds):
            # Transform global positions into local.
            T_GA = np.dot(T_GS, self.spider.T_ANCHORS[leg])
            # if readLegs:
            #     startPosition = self.motorDriver.syncReadMotorsPositionsInLegs([leg], True)[0]
                # startPosition = self.motorDriver.readLegPosition(leg)
            if not readLegs:
                legGlobalStartPosition = np.append(globalStartPositions[legIdx], 1)
                startPosition = np.dot(np.linalg.inv(T_GA), legGlobalStartPosition)[:3]
            legGlobalGoalPosition = np.append(globalGoalPositions[legIdx], 1)
            localGoalPosition = np.dot(np.linalg.inv(T_GA), legGlobalGoalPosition)[:3]
            # Calculate trajectory and velocity.
            if trajectoryType == 'bezier':
                traj, vel = self.trajectoryPlanner.bezierTrajectory(startPosition[legIdx], localGoalPosition, durations[legIdx])
            elif trajectoryType == 'minJerk':
                traj, vel = self.trajectoryPlanner.minJerkTrajectory(startPosition[legIdx], localGoalPosition, durations[legIdx])
            else:
                print("Invalid trajectory type.")
                return False
            trajectories.append(traj)
            velocities.append(vel)
        
        result = self.moveLegs(legsIds, trajectories, velocities, gripperCommands)

        return result
    
    def moveLegsAndGrabPins(self, legsIds, globalGoalPositions, spiderPose, durations, readLegs = True, globalStartPositions = None, correctAfterDetach = True, globalLegsPositions = None):
        """Open grippers, detach legs from pin, move them on approach positions above goal pins and than lower them on pins. Finally, close the grippers.

        :param legsIds: Legs ids.
        :param globalGoalPositions: Global goal positions of legs (pins).
        :param spiderPose: Global pose of spider during legs movements.
        :param durations: Array of durations for each leg movement.
        :param readLegs: If true, read starting legs positions before movements, otherwise use FF calculations, defaults to True
        :param globalStartPositions: Global start positions of legs, defaults to None.
        :param correctAfterDetach: If true, read platform pose after leg detaches itself from the pin and use this new pose as a base for further legs movements.
        :return: True if movements were successfull, false otherwise.
        """
        # if correctAfterDetach and len(legsIds) > 2:
        #     raise ValueError("Cannot calculate platform pose with less than three legs attached to the pins.")
        # if correctAfterDetach and globalLegsPositions is None:
        #     raise ValueError("Need global legs positions for calculating platform pose.")

        if legsIds == 5:
            legsIds = [0, 1, 2, 3, 4]
            
        approachTime = 1.5
        detachTime = 1
        durations = np.array(durations) - approachTime - detachTime
        approachPoints = self.matrixCalculator.getLegsApproachPositionsInGlobal(legsIds, spiderPose, globalGoalPositions)
        if globalStartPositions:
            detachPoints = np.copy(globalStartPositions)
        else:
            detachPoints = []
            localLegsPositions = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds, True)
            detachPoints = self.matrixCalculator.getLegsInGlobal(legsIds, localLegsPositions, spiderPose)
        detachPoints[:, 2] += 0.02
 
        if not self.moveLegsWrapper(legsIds, detachPoints, spiderPose, np.ones(len(legsIds)) * detachTime, [self.gripperController.OPEN_COMMAND] * len(legsIds), readLegs, globalStartPositions, 'minJerk'):
            print("Legs movement error!")
            return False
        if not self.moveLegsWrapper(legsIds, approachPoints, spiderPose, durations, readLegs = True, globalStartPositions = detachPoints):
            print("Legs movement error!")
            return False
        if not self.moveLegsWrapper(legsIds, globalGoalPositions, spiderPose, np.ones(len(legsIds)) * approachTime, [self.gripperController.CLOSE_COMMAND] * len(legsIds), True, approachPoints, 'minJerk'):
            print("Legs movement error!")
            return False

        return True

    def movePlatform(self, trajectory, velocity, globalLegsPositions):
        """Move spider body as a parallel platform along a given trajectory.

        :param trajectory: Spider's body trajectory.
        :param velocity: Spider's body velocity.
        :param globalStartPose: Global pose of spider at the beginning of the platform movement.
        :param globalLegsPositions: Global positions of legs during platform movement.
        :return: True if movement was successfull, false otherwise.
        """

        legsIds = [leg for leg in range(self.spider.NUMBER_OF_LEGS)]
        qDs, qDds = self.getQdQddPlatformFF(trajectory, velocity, globalLegsPositions)

        self.motorDriver.clearGroupSyncWriteParams()

        lastErrors = np.zeros([len(legsIds), 3])
        Kp = np.array([[5, 15, 15]] * len(legsIds))
        Kd = np.array([[1, 1, 1]] * len(legsIds))
        timeStep = trajectory[1][-1] - trajectory[0][-1]

        for idx, qD in enumerate(qDs):
            startTime = time.time()
            qA = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds)
            errors = np.array(qD - qA, dtype = object)
            dE = (errors - lastErrors) / timeStep
            qCds = Kp * errors + Kd * dE + qDds[idx]
            lastErrors = errors

            if idx == len(trajectory) - 1:
                qCds = np.zeros([len(legsIds), 3])

            if not self.motorDriver.syncWriteMotorsVelocitiesInLegs(legsIds, qCds, idx == 0):
                return False

            try:
                time.sleep(timeStep - (time.time() - startTime))
            except:
                time.sleep(0)

        self.motorDriver.clearGroupSyncWriteParams()

        return True

    def movePlatformWrapper(self, legsIds, globalGoalPose, globalLegsPositions, duration):
        """Wrapper function for moving a platform. Includes trajectory calculations.

        :param legsIds: Used legs for moving a platform (at least 4).
        :param globalStartPose: Starting pose in global origin.
        :param globalGoalPose: Goal pose in global origin.
        :param globalLegsPositions: Global positions of legs during platform movement.
        :param duration: Desired duration of movement.
        :return: True if movement was successfull, false otherwise.
        """
        if len(legsIds) < 4:
            print("Cannot move platform with less than 4 legs.")
            return False
        attachedLegs = self.gripperController.getIdsOfAttachedLegs()
        startPose = self.motorDriver.syncReadPlatformPose(attachedLegs, np.array(globalLegsPositions)[(attachedLegs)])
        traj, vel = self.trajectoryPlanner.minJerkTrajectory(startPose, globalGoalPose, duration)
        result = self.movePlatform(traj, vel, globalLegsPositions)

        return result

    def walk(self, globalStartPose, globalGoalPose):
        """Walking procedure for spider to walk from start to goal point on the wall.

        :param globalStartPose: Spider's starting pose on the wall.
        :param globalGoalPose: Spider's goal pose on the wall.
        :return: True if walking procedure was successfull, false otherwise.
        """
        platformPoses, pins = self.pathPlanner.calculateWalkingMovesFF(globalStartPose, globalGoalPose)

        # Move through calculated poses.
        for poseIdx, pose in enumerate(platformPoses):
            if poseIdx == 0:
                continue

            # Move platform.
            linDist = np.linalg.norm(platformPoses[poseIdx - 1][:3] - pose[:3])
            if linDist != 0:
                parallelMovementDuration = 2.5 * linDist / 0.05
            else:
                rotDist = abs(platformPoses[poseIdx - 1][3] - pose[3])
                parallelMovementDuration = 2 * rotDist / 0.1

            result = self.movePlatformWrapper([0, 1, 2, 3, 4], pose, pins[poseIdx - 1], parallelMovementDuration)
            if not result:
                print("Platform movement error!")
                return False
                
            # Select indexes of legs which have to move.
            legsToMoveIdxs = np.array(np.where(np.any(pins[poseIdx] - pins[poseIdx - 1] != 0, axis = 1))).flatten()
            # Move legs and grab pins.
            for legIdx in legsToMoveIdxs:
                result = self.moveLegsAndGrabPins([legIdx], [pins[poseIdx][legIdx]], pose, np.ones(1) * 6)
                if not result:
                    print("Legs movement error!")
                    return False

                

        return True

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




