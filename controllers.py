import numpy as np
import time
import threading
import queue

import config
from environment import spider
from calculations import mathtools as mathTools
from calculations import transformations as tf
from calculations import kinematics as kin
from calculations import dynamics as dyn
from planning import trajectoryplanner as trajPlanner
from planning import pathplanner 
from periphery import dynamixel as dmx
from periphery import grippers
from periphery import waterpumpsbno as wpb


class VelocityController:
    """ Class for velocity-control of spider's movement. All legs are controlled with same self, but can be moved separately and independently
    from other legs. Reference positions for each legs are writen in legs-queues. On each control-loop self takes first values from all of the legs-queues.
    """
    def __init__ (self, isVertical = False):
        self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.grippersArduino = grippers.GrippersArduino()
        self.pumpsBnoArduino = wpb.PumpsBnoArduino()

        self.legsQueues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()
        self.lastMotorsPositions = np.zeros([spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG])
        self.locker = threading.Lock()
        self.killControllerThread = False

        self.period = 1.0 / config.CONTROLLER_FREQUENCY
        self.Kp = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_P
        self.Kd = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_D

        self.qA = []
        self.fAMean = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tauAMean = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.fD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.isForceMode = False
        self.forceModeLegsIds = None

        self.spiderGravityVector = np.array([0.0, -9.81, 0.0], dtype = np.float32) if isVertical else np.array([0.0, 0.0, -9.81], dtype = np.float32)

        self.currents = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG))

        self.__initControllerThread()
        time.sleep(1)

    #region public methods
    def controller(self):
        """Velocity self to controll all five legs continuously. Runs in separate thread.
        """
        lastQErrors = np.zeros([spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG])
        fErrors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        qDd = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        qD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        init = True

        fBuffer = np.zeros((10, 5, 3), dtype = np.float32)
        tauBuffer = np.zeros((10, 5, 3), dtype = np.float32)
        fCounter = 0
        tauCounter = 0

        currentLimit = 4.0

        while True:
            if self.killControllerThread:
                break
            
            startTime = time.perf_counter()

            # Get current data.
            with self.locker:
                try:
                    currentAngles, self.currents = self.motorDriver.syncReadAnglesAndCurrentsWrapper()
                    currents = self.currents
                except KeyError:
                    print("COMM READING ERROR")
                    continue
                self.qA = currentAngles
                forceMode = self.isForceMode
                forceModeLegs = self.forceModeLegsIds
                # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
                if init:
                    self.lastMotorsPositions = currentAngles
                    init = False

            # if currents[currents > currentLimit].any():
            #     print(self.motorDriver.motorsIds[np.where(currents > currentLimit)])

            qD, qDd = self.__getQdQddFromQueues()

            tauA, fA, Jhash = dyn.getTorquesAndForcesOnLegsTips(currentAngles, currents, self.pumpsBnoArduino.getGravityVector())

            # Filter over measured values.
            self.fAMean, fBuffer, fCounter = mathTools.runningAverage(fBuffer, fCounter, fA)
            self.tauAMean, tauBuffer, tauCounter = mathTools.runningAverage(tauBuffer, tauCounter, tauA)

            if forceMode:
                offsets = self.__forcePositionP(self.fD, self.fAMean)
                fErrors = self.fD - self.fAMean
                qD[forceModeLegs], qDd[forceModeLegs] = dyn.getQdQddFromOffsetsAndForceErrors(forceModeLegs, offsets, currentAngles, fErrors, Jhash)

                with self.locker:
                    self.lastMotorsPositions[forceModeLegs] = currentAngles[forceModeLegs]

            qCds, lastQErrors = self.__positionVelocityPD(qD, currentAngles, qDd, lastQErrors)

            # Limit joints velocities, before sending them to motors.
            if forceMode:
                qCds[qCds > 1.0] = 1.0
                qCds[qCds < -1.0] = -1.0

            # Send new commands to motors.
            with self.locker:
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(spider.LEGS_IDS, qCds)

            elapsedTime = time.perf_counter() - startTime
            while elapsedTime < self.period:
                elapsedTime = time.perf_counter() - startTime
                time.sleep(0)

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

        legCurrentPosition = kin.legForwardKinematics(currentAngles)[:,3][:3]
        localGoalPosition = self.__convertIntoLocalGoalPosition(legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose)

        positionTrajectory, velocityTrajectory = self.__getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)

        qDs, qDds = self.__getQdQddLegFF(positionTrajectory, velocityTrajectory)

        for idx, qD in enumerate(qDs):
            self.legsQueues[legId].put([qD, qDds[idx]])
        self.legsQueues[legId].put(self.sentinel)

        return True
            
    def moveLegsSync(self, legsIds, goalPositionsOrOffsets, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
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

        with self.locker:
            currentAngles = self.qA

        for idx, leg in enumerate(legsIds):
            # Get current leg position in local.
            legCurrentPosition = kin.legForwardKinematics(currentAngles[leg])[:,3][:3]            
            localGoalPosition = self.__convertIntoLocalGoalPosition(leg, legCurrentPosition, goalPositionsOrOffsets[idx], origin, isOffset, spiderPose)
            
            positionTrajectory, velocityTrajectory = self.__getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)
            
            qD, qDd = self.__getQdQddLegFF(positionTrajectory, velocityTrajectory)
            qDs.append(qD)
            qDds.append(qDd)
        
        for i in range(len(qDs[0])):
            for idx, leg in enumerate(legsIds):
                self.legsQueues[leg].put([qDs[idx][i], qDds[idx][i]])
        for leg in legsIds:
            self.legsQueues[leg].put(self.sentinel)
        
        return True
    
    def walk(self, startPose, endPose, doInitBno = False):
        """Walking procedure from start to goal pose on the wall.

        Args:
            startPose (list): 1x4 array of x, y, z and yaw values of starting spider's pose in global origin.
            endPose (_type_): 1x4 array of x, y, z and yaw values of goal spider's pose in global origin.
        """
        poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)
        for step, pose in enumerate(poses):
            currentPinsPositions = pinsInstructions[step, :, 1:]
            currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)

            if step == 0:
                self.moveLegsSync(currentLegsMovingOrder, currentPinsPositions, 'g', 3, 'minJerk', pose)
                time.sleep(3.5)
                if doInitBno:
                    self.pumpsBnoArduino.resetBno()
                time.sleep(1)
                self.distributeForces(spider.LEGS_IDS, 5)
                continue
            
            if step % 3 == 0 and step != 0:
                self.startForceMode([0, 1, 2, 3, 4], [[0.0, -1.0, 0.0]] * 5)
                time.sleep(5)
                self.stopForceMode()
                time.sleep(30)

            previousPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
            self.moveLegsSync(currentLegsMovingOrder, previousPinsPositions, 'g', 1.5, 'minJerk', pose)
            time.sleep(2)

            pinsOffsets = currentPinsPositions - previousPinsPositions
            for idx, leg in enumerate(currentLegsMovingOrder):
                if pinsOffsets[idx].any():
                    self.moveLegFromPinToPin(leg, currentPinsPositions[idx], previousPinsPositions[idx])          
            self.distributeForces(spider.LEGS_IDS, 2)

    def moveLegFromPinToPin(self, leg, goalPinPosition, currentPinPosition):
        """Move leg from one pin to another, including force-offloading and gripper movements.

        Args:
            leg (int): Leg id.
            goalPinPosition (list): 1x3 array goal-pin position in global origin.
            currentPinPosition (list): 1x3 array current pin position in global origin.
        """
        otherLegs = np.delete(spider.LEGS_IDS, leg)
        self.distributeForces(otherLegs, 1)
        time.sleep(1.5)
        self.grippersArduino.moveGripper(leg, self.grippersArduino.OPEN_COMMAND)
        time.sleep(2)
        
        # Get rpy from sensor.
        rpy = self.pumpsBnoArduino.getRpy()

        globalToSpiderRotation = tf.xyzRpyToMatrix(np.array([0, 0, 0, rpy[0], rpy[1], rpy[2]]), True)
        globalToLegRotation = np.linalg.inv(np.dot(globalToSpiderRotation, spider.T_ANCHORS[leg][:3, :3]))

        globalZDirectionInSpider = np.dot(np.linalg.inv(globalToSpiderRotation), np.array([0.0, 0.0, 1.0]))
        globalZDirectionInLocal = np.dot(globalToLegRotation, np.array([0.0, 0.0, 1.0]))

        pinToPinGlobal = goalPinPosition - currentPinPosition
        pinToPinLocal = np.dot(globalToLegRotation, pinToPinGlobal)

        detachOffsetZ = 0.02
        detachOffsetInLocal = globalZDirectionInLocal * detachOffsetZ

        with self.locker:
            currentKp = self.Kp[leg]
            currentKd = self.Kd[leg]
            self.Kp[leg] = np.ones(spider.NUMBER_OF_MOTORS_IN_LEG, dtype = np.float32) * config.K_P_LEG
            self.Kd[leg] = np.ones(spider.NUMBER_OF_MOTORS_IN_LEG, dtype = np.float32) * config.K_D_LEG

        self.moveLegAsync(leg, pinToPinLocal, 'l', 3, 'bezier', isOffset = True)
        time.sleep(3.5)

        with self.locker:
            self.Kp[leg] = currentKp
            self.Kd[leg] = currentKd

        self.startForceMode([leg], np.array([np.zeros(3, dtype = np.float32)]))
        self.grippersArduino.moveGripper(leg, self.grippersArduino.CLOSE_COMMAND)
        time.sleep(3)
        self.stopForceMode()

        self.motorDriver.readHardwareErrorRegister()

        # Check if leg successfully grabbed the pin.
        while not (leg in self.grippersArduino.getIdsOfAttachedLegs()):
            print(f"LEG {leg} NOT ATTACHED")
            self.grippersArduino.moveGripper(leg, self.grippersArduino.OPEN_COMMAND)
            time.sleep(1)
            self.moveLegAsync(leg, detachOffsetInLocal / 2.0, 'l', 1, 'minJerk', isOffset = True)
            time.sleep(1)
            
            self.startForceMode([leg], [globalZDirectionInSpider * (-6.0)])
            time.sleep(1)
            self.grippersArduino.moveGripper(leg, self.grippersArduino.CLOSE_COMMAND)
            self.stopForceMode()
            time.sleep(3)         

    def startForceMode(self, legsIds, desiredForces):
        """Start force self inside main velocity self loop.

        Args:
            legsIds (list): Ids of leg, which is to be force-controlled.
            desiredForce (list): 5x3 vector of x, y, z values of force, given in spider's origin, where n is number of used legs.
        """
        if len(legsIds) != len(desiredForces):
            raise ValueError("Number of legs used in force controll does not match with number of given desired forces vectors.")
        with self.locker:
            self.isForceMode = True
            self.forceModeLegsIds = legsIds
            self.fD[legsIds] = desiredForces
    
    def stopForceMode(self):
        """Stop force self inside main velocity self loop.
        """
        with self.locker:
            self.isForceMode = False

    def endControllerThread(self):
        """Set flag to kill a self thread.
        """
        self.killControllerThread = True

    def distributeForces(self, legsIds, duration):
        """Run force distribution process in a loop for a given duration.
        """
        offloadLegId = np.setdiff1d(spider.LEGS_IDS, legsIds)
        if len(offloadLegId) > 1:
            print("Cannot offload more than one leg at the same time.")
            return False

        # print("START DISTRIBUTION")

        startTime = time.perf_counter()
        elapsedTime = 0
        while elapsedTime < duration:
            with self.locker:
                currentTorques = self.tauAMean
                currentAngles = self.qA
            fDist = dyn.calculateDistributedForces(currentTorques, currentAngles, legsIds, offloadLegId)

            if len(offloadLegId):
                fDist = np.insert(fDist, offloadLegId[0], np.zeros(3, dtype = np.float32), axis = 0)
            self.startForceMode(spider.LEGS_IDS, fDist)

            time.sleep(self.period)
            elapsedTime = time.perf_counter() - startTime
        
        self.stopForceMode()

        # print("DISTRIBUTION FINISHED")
    #endregion

    #region private methods
    def __initControllerThread(self):
        """Start a thread with self loop.
        """
        thread = threading.Thread(target = self.controller, name = 'velocity_controller_thread', daemon = False)
        try:
            thread.start()
            print("self thread is running.")
        except RuntimeError as re:
            print(re)

    def __getQdQddFromQueues(self):
        """Read current qD and qDd from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            tuple: Two 5x3 numpy.ndarrays of current qDs and qDds.
        """
        qD = np.empty([spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)
        qDd = np.empty([spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG], dtype = np.float32)

        for leg in spider.LEGS_IDS:
            try:
                queueData = self.legsQueues[leg].get(False)
                with self.locker:
                    self.lastMotorsPositions[leg] = np.copy(self.qA[leg])
                if queueData is self.sentinel:
                    qD[leg] = np.copy(self.lastMotorsPositions[leg])
                    qDd[leg] = ([0, 0, 0])
                else:
                    qD[leg] = queueData[0]
                    qDd[leg] = queueData[1]
            except queue.Empty:
                with self.locker:
                    qD[leg] = np.copy(self.lastMotorsPositions[leg])
                qDd[leg] = ([0, 0, 0])

        return qD, qDd

    def __getQdQddLegFF(self, xD, xDd):
        """(Feed-forward) calculate and write reference joints positions and velocities for single leg movement into leg-queue.

        Args:
            legId (int): Leg id.
            xD (list): nx7 array of position (6 values for xyzrpy) trajectory and time stamps (7th value in a row), where n is number of steps in trajectory.
            xDd (list): nx6 array for xyzrpy values of velocity trajectory, where n is number of steps in trajectory.

        Returns:
            tuple: Two nx3 numpy.ndarrays of reference joints positions and velocities, where n is number of steps in trajectory.
        """

        qDs = np.zeros([len(xD), spider.NUMBER_OF_MOTORS_IN_LEG])
        qDds = np.zeros([len(xD), spider.NUMBER_OF_MOTORS_IN_LEG])
        for idx, pose in enumerate(xD):
            qDs[idx] = kin.legInverseKinematics(np.array(pose[:3], dtype = np.float32))
            J = kin.legJacobi(qDs[idx])
            qDds[idx] = np.dot(np.linalg.inv(J), xDd[idx][:3])

        return qDs, qDds
    
    def __positionVelocityPD(self, desiredAngles, currentAngles, ffVelocity, lastErrors):
        """Position-velocity PD self. Calculate commanded velocities from position input and add feed-forward calculated velocities.

        Args:
            desiredAngles (list): 5x3 array of desired angles in joints.
            currentAngles (list): 5x3 array of measured current angles in joints.
            ffVelocity (list): 5x3 array of feed forward calculated joints velocities.
            lastErrors (list): 5x3 array of last position errors.

        Returns:
            tuple: 5x3 numpy.ndarray of commanded joints velocities and updated errors.
        """
        qErrors = np.array(desiredAngles - currentAngles)
        dQe = (qErrors - lastErrors) / self.period
        with self.locker:
            qCds = self.Kp * qErrors + self.Kd * dQe + ffVelocity
        lastErrors = qErrors

        return qCds, lastErrors

    def __forcePositionP(self, desiredForces, currentForces):
        """Force-position P self. Calculate position offsets from desired forces.

        Args:
            desiredForces (list): 5x3 array of desired forces in spider's origin.
            currentForces (list): 5x3 array of measured current forces in spider's origin.

        Returns:
            numpy.ndarray: 5x3 array of of position offsets of leg-tips, given in spider's origin.
        """
        fErrors = desiredForces - currentForces
        dXSpider = fErrors * config.K_P_FORCE
        offsets = dXSpider * self.period

        return offsets
    
    def __convertIntoLocalGoalPosition(self, legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose):
        if origin == 'l':
            localGoalPosition = np.copy(goalPositionOrOffset)
            if isOffset:
                localGoalPosition += legCurrentPosition
            return localGoalPosition
        if not isOffset:
            return tf.getLegInLocal(legId, goalPositionOrOffset, spiderPose)
        return np.array(legCurrentPosition + tf.getGlobalDirectionInLocal(legId, spiderPose, goalPositionOrOffset), dtype = np.float32)
    
    def __getTrajectory(self, legCurrentPosition, legGoalPosition, duration, trajectoryType):
        # If movement goes over x = 0, add additional point at (0, 0, dz/2).
        xSigns = [np.sign(legCurrentPosition[0]), np.sign(legGoalPosition[0])]
        legCurrentPosition = np.array(legCurrentPosition, dtype = np.float32)
        legGoalPosition = np.array(legGoalPosition, dtype = np.float32)
        if xSigns[0] != xSigns[1] and 0 not in xSigns:
            interPoint = np.array([0.0, 0.0, (legCurrentPosition[2] + legGoalPosition[2]) / 2.0])
            firstPositionTrajectory, firstVelocityTrajectory = trajPlanner.calculateTrajectory(legCurrentPosition, interPoint, duration / 2, trajectoryType)
            secondPositionTrajectory, secondVelocityTrajectory = trajPlanner.calculateTrajectory(interPoint, legGoalPosition, duration / 2, trajectoryType)

            return np.append(firstPositionTrajectory, secondPositionTrajectory, axis = 0), np.append(firstVelocityTrajectory, secondVelocityTrajectory, axis = 0)

        return trajPlanner.calculateTrajectory(legCurrentPosition, legGoalPosition, duration, trajectoryType)
    #endregion