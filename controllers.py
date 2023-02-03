import numpy as np
import time
import threading
import multiprocessing
import queue
import matplotlib.pyplot as plt

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
    def __init__ (self):
        # self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.grippersArduino = grippers.GrippersArduino()
        self.pumpsBnoArduino = wpb.PumpsBnoArduino()

        self.legsQueues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()

        self.qA = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.xA = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.lastLegsPositions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.lastXErrors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.locker = threading.Lock()
        self.killControllerThreadEvent = threading.Event()
        self.killSafetyThreadEvent = threading.Event()
        self.killWalkingThreadEvent = threading.Event()

        self.period = 1.0 / config.CONTROLLER_FREQUENCY
        self.Kp = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_P
        self.Kd = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_D

        self.isForceMode = False
        self.forceModeLegsIds = None
        self.fD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tauAMean = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)

        self.isImpedanceMode = False
        self.impedanceDirection = np.zeros(3, dtype = np.float32)
        self.impedanceLegId = None

        self.hardwareErrors = None

        time.sleep(1)

    #region public methods
    # def controller(self):
    #     """Velocity controller to controll all five legs continuously. Runs in separate thread.
    #     """
    #     lastXErrors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

    #     fBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
    #     tauBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
    #     fCounter = 0
    #     tauCounter = 0

    #     hwErrorCounter = 0

    #     init = True
    #     # Enable watchdogs when controller starts.
    #     self.motorDriver.setBusWatchdog(10)

    #     while True:
    #         if self.killControllerThreadEvent.is_set():
    #             break
            
    #         startTime = time.perf_counter()
    #         # Get current data.
    #         readErrors = hwErrorCounter % (config.CONTROLLER_FREQUENCY / 2)
    #         if readErrors:
    #             hwErrorCounter = 0
    #         hwErrorCounter += 1
    #         with self.locker:
    #             try:
    #                 currentAngles, currents, _ = self.motorDriver.syncReadMotorsData(readErrors)
    #                 self.qA = currentAngles
    #                 self.xA = kin.allLegsPositions(currentAngles, config.LEG_ORIGIN)
    #                 xA = self.xA
    #                 fD = self.fD
    #             except KeyError:
    #                 print("COMM READING ERROR")
    #                 continue
    #             forceMode = self.isForceMode
    #             forceModeLegs = self.forceModeLegsIds
    #             impedanceMode = self.isImpedanceMode
    #             impedanceDirection = self.impedanceDirection
    #             impedanceLegId = self.impedanceLegId
    #             # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
    #             if init:
    #                 self.lastLegsPositions = xA
    #                 init = False

    #         xD, xDd, xDdd = self.__getXdXddXdddFromQueues()
            
    #         tauA, fA = dyn.getTorquesAndForcesOnLegsTips(currentAngles, currents, self.pumpsBnoArduino.getGravityVector())
    #         fAMean, fBuffer, fCounter = mathTools.runningAverage(fBuffer, fCounter, fA)
    #         self.tauAMean, tauBuffer, tauCounter = mathTools.runningAverage(tauBuffer, tauCounter, tauA)
            
    #         if forceMode:
    #             legOffsetsInSpider, legVelocitiesInSpider = self.__forcePositionP(fD, fAMean)
    #             localOffsets, localVelocities = kin.getXdXddFromOffsets(forceModeLegs, legOffsetsInSpider, legVelocitiesInSpider)
    #             xD[forceModeLegs] += localOffsets
    #             xDd[forceModeLegs] = localVelocities
    #             with self.locker:
    #                 self.lastLegsPositions[forceModeLegs] = xA[forceModeLegs]
            
    #         if impedanceMode:
    #             xDd[impedanceLegId] = 0.1 * impedanceDirection * int(np.linalg.norm(fAMean[impedanceLegId]) < 3.0)
    #             with self.locker:
    #                 self.lastLegsPositions[forceModeLegs] = xA[forceModeLegs]

    #         xCds, lastXErrors = self.__eePositionVelocityPd(xD, xA, xDd, xDdd, lastXErrors)
    #         qCds = kin.getJointsVelocities(currentAngles, xCds)

    #         # Send new commands to motors.
    #         with self.locker:
    #             self.motorDriver.syncWriteMotorsVelocitiesInLegs(spider.LEGS_IDS, qCds)

    #         elapsedTime = time.perf_counter() - startTime
    #         while elapsedTime < self.period:
    #             elapsedTime = time.perf_counter() - startTime
    #             time.sleep(0)
        
    #     print("Controller thread stopped.")
    
    def jointsVelocityController(self, qA, xA, init, mode = None):
        # If controller was just initialized, save current positions and keep legs on these positions until new command is given.
        if init:
            self.lastLegsPositions = xA
        xD, xDd, xDdd = self.__getXdXddXdddFromQueues()
        
        xCds, self.lastXErrors = self.__eePositionVelocityPd(xD, xA, xDd, xDdd)
        qCds = kin.getJointsVelocities(qA, xCds)

        return qCds
    
    def moveLegAsync(self, legId, legCurrentPosition, goalPositionOrOffset, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
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
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            raise ValueError("Unknown origin.")
        if origin == config.GLOBAL_ORIGIN and spiderPose is None:
            raise TypeError("Parameter spiderPose should not be None.")

        self.legsQueues[legId] = queue.Queue()

        localGoalPosition = tf.convertIntoLocalGoalPosition(legId, legCurrentPosition, goalPositionOrOffset, origin, isOffset, spiderPose)
        positionTrajectory, velocityTrajectory, accelerationTrajectory = trajPlanner.getTrajectory(legCurrentPosition, localGoalPosition, duration, trajectoryType)

        for idx, position in enumerate(positionTrajectory):
            self.legsQueues[legId].put([position[:3], velocityTrajectory[idx][:3], accelerationTrajectory[idx][:3]])
        self.legsQueues[legId].put(self.sentinel)

        return True
            
    def moveLegsSync(self, legsIds, legsCurrentPositions, goalPositionsOrOffsets, origin, duration, trajectoryType, spiderPose = None, isOffset = False):
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
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            raise ValueError(f"Unknown origin {origin}.")
        if origin == config.GLOBAL_ORIGIN and spiderPose is None:
            raise TypeError("If origin is global, spider pose should be given.")
        if len(legsIds) != len(goalPositionsOrOffsets):
            raise ValueError("Number of legs and given goal positions should be the same.")
        
        # Stop all of the given legs.
        for leg in legsIds:
            self.legsQueues[leg] = queue.Queue()

        xDs = np.zeros((len(legsIds), int(duration / self.period), 3), dtype = np.float32)
        xDds = np.zeros((len(legsIds), int(duration / self.period), 3), dtype = np.float32)
        xDdds = np.zeros((len(legsIds), int(duration / self.period), 3) , dtype = np.float32)

        for idx, leg in enumerate(legsIds):          
            localGoalPosition = tf.convertIntoLocalGoalPosition(leg, legsCurrentPositions[leg], goalPositionsOrOffsets[idx], origin, isOffset, spiderPose) 
            positionTrajectory, velocityTrajectory, accelerationTrajectory = trajPlanner.getTrajectory(legsCurrentPositions[leg], localGoalPosition, duration, trajectoryType)

            xDs[idx] = positionTrajectory[:, :3]
            xDds[idx] = velocityTrajectory[:, :3]
            xDdds[idx] = accelerationTrajectory[:, :3]
        
        for i in range(len(xDs[0])):
            for idx, leg in enumerate(legsIds):
                self.legsQueues[leg].put([xDs[idx][i], xDds[idx][i], xDdds[idx][i]])

        for leg in legsIds:
            self.legsQueues[leg].put(self.sentinel)
        
        return True
    
    def moveLegFromPinToPin(self, leg, goalPinPosition, currentPinPosition):
        """Move leg from one pin to another, including force-offloading and gripper movements.

        Args:
            leg (int): Leg id.
            goalPinPosition (list): 1x3 array goal-pin position in global origin.
            currentPinPosition (list): 1x3 array current pin position in global origin.
        """
        otherLegs = np.delete(spider.LEGS_IDS, leg)
        self.distributeForces(otherLegs, config.FORCE_DISTRIBUTION_DURATION)
        self.grippersArduino.moveGripper(leg, self.grippersArduino.OPEN_COMMAND)
        time.sleep(1.5)
        
        # Get rpy from sensor.
        rpy = self.pumpsBnoArduino.getRpy()
        globalToSpiderRotation = tf.xyzRpyToMatrix(np.concatenate((np.zeros(3, dtype = np.float32), rpy)), True)
        globalToLegRotation = np.linalg.inv(np.dot(globalToSpiderRotation, spider.T_ANCHORS[leg][:3, :3]))
        globalZDirectionInLocal = np.dot(globalToLegRotation, np.array([0.0, 0.0, 1.0], dtype = np.float32))

        pinToPinGlobal = goalPinPosition - currentPinPosition
        pinToPinLocal = np.dot(globalToLegRotation, pinToPinGlobal)

        self.moveLegAsync(leg, pinToPinLocal, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, isOffset = True)
        time.sleep(3.5)
        
        self.startForceMode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
        self.grippersArduino.moveGripper(leg, self.grippersArduino.CLOSE_COMMAND)
        time.sleep(3)
        self.stopForceMode()

        detachOffsetZ = 0.03
        detachOffsetInLocal = globalZDirectionInLocal * detachOffsetZ
        # Check if leg successfully grabbed the pin and correct if neccessary.
        while not (leg in self.grippersArduino.getIdsOfAttachedLegs()):
            print(f"LEG {leg} NOT ATTACHED")
            self.grippersArduino.moveGripper(leg, self.grippersArduino.OPEN_COMMAND)
            time.sleep(1)
            self.moveLegAsync(leg, detachOffsetInLocal, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY, isOffset = True)
            time.sleep(1)

            with self.locker:
                self.isImpedanceMode = True
                self.impedanceDirection = -globalZDirectionInLocal
                self.impedanceLegId = leg

            time.sleep(3)
            self.grippersArduino.moveGripper(leg, self.grippersArduino.CLOSE_COMMAND)
            time.sleep(1)
                
        with self.locker:
            self.isImpedanceMode = False     

    def startForceMode(self, legsIds, desiredForces):
        """Start force mode inside main velocity controller loop.

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
        """Stop force mode inside main velocity self loop.
        """
        with self.locker:
            self.isForceMode = False

    def distributeForces(self, legsIds, qA, tauA, duration):
        """Run force distribution process in a loop for a given duration.
        """
        offloadLegId = np.setdiff1d(spider.LEGS_IDS, legsIds)
        if len(offloadLegId) > 1:
            print("Cannot offload more than one leg at the same time.")
            return False

        startTime = time.perf_counter()
        elapsedTime = 0
        while elapsedTime < duration:
            fDist = dyn.calculateDistributedForces(tauA, qA, legsIds, offloadLegId)

            if len(offloadLegId):
                fDist = np.insert(fDist, offloadLegId[0], np.zeros(3, dtype = np.float32), axis = 0)
            self.startForceMode(spider.LEGS_IDS, fDist)

            time.sleep(self.period)
            elapsedTime = time.perf_counter() - startTime
        
        self.stopForceMode()
    #endregion

    #region private methods
    def __getXdXddXdddFromQueues(self):
        """Read current desired position, velocity and acceleration from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            tuple: Three 5x3 arrays of current desired positions, velocities and accelerations of legs.
        """
        xD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        xDd = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        xDdd = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        for leg in spider.LEGS_IDS:
            try:
                queueData = self.legsQueues[leg].get(False)
                with self.locker:
                    if queueData is not self.sentinel:
                        self.lastLegsPositions[leg] = queueData[0]
                if queueData is self.sentinel:
                    xD[leg] = np.copy(self.lastLegsPositions[leg])
                    xDd[leg] = np.zeros(3, dtype = np.float32)
                    xDdd[leg] = np.zeros(3, dtype = np.float32)
                else:
                    xD[leg] = queueData[0]
                    xDd[leg] = queueData[1]
                    xDdd[leg] = queueData[2]
            except queue.Empty:
                with self.locker:
                    xD[leg] = np.copy(self.lastLegsPositions[leg])
                xDd[leg] = np.zeros(3, dtype = np.float32)
                xDdd[leg] = np.zeros(3, dtype = np.float32)

        return xD, xDd, xDdd

    def __eePositionVelocityPd(self, xD, xA, xDd, xDdd):
        """PD controller. Feed-forward velocity is used only in force mode, otherwise its values are zeros.

        Args:
            xD (numpy.ndarray): 5x3 array of desired legs' positions.
            xA (numpy.ndarray): 5x3 array of actual legs' positions.
            xDd (numpy.ndarray): 5x3 array of feed-forward velocities.
            xDdd (numpy.ndarray): 5x3 array of feed-forward accelerations.
            lastErrors (numpy.ndarray): 5x3 array of position errors from previous loop.

        Returns:
            tuple: Two 5x3 arrays of commanded legs velocities and current position errors.
        """
        xErrors = np.array(xD - xA)
        dXe = (xErrors - self.lastXErrors) / self.period
        xCd = np.array(self.Kp * xErrors + self.Kd * dXe + xDd + config.K_ACC * xDdd, dtype = np.float32)

        return xCd, xErrors

    def __forcePositionP(self, desiredForces, currentForces):
        """Force-position P self. Calculate position offsets and desired legs velocities in spider's origin from desired forces.

        Args:
            desiredForces (list): 5x3 array of desired forces in spider's origin.
            currentForces (list): 5x3 array of measured current forces in spider's origin.

        Returns:
            tuple: Two 5x3 array of of position offsets of leg-tips and leg-tips velocities, given in spider's origin.
        """
        fErrors = desiredForces - currentForces
        dXSpider = fErrors * config.K_P_FORCE
        offsets = dXSpider * self.period

        return offsets, dXSpider
    
    def walkProcedure(self, startPose, endPose, initBno = False):
        """Walking procedure from start to goal pose on the wall.

        Args:
            startPose (list): 1x4 array of x, y, z and yaw values of starting spider's pose in global origin.
            endPose (list): 1x4 array of x, y, z and yaw values of goal spider's pose in global origin.
        """
        poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)
        for step, pose in enumerate(poses):
            # if self.killWalkingThreadEvent.is_set():
            #     break
            currentPinsPositions = pinsInstructions[step, :, 1:]
            currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)

            if step == 0:
                self.moveLegsSync(currentLegsMovingOrder, currentPinsPositions, config.GLOBAL_ORIGIN, 3, config.MINJERK_TRAJECTORY, pose)
                time.sleep(3.5)
                # if initBno:
                #     self.pumpsBnoArduino.resetBno()
                #     time.sleep(1)
                # self.distributeForces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION)
                continue
            
            # if step % 3 == 0 and step != 0:
            #     self.startForceMode(spider.LEGS_IDS, [[0.0, -1.0, 0.0]] * spider.NUMBER_OF_LEGS)
            #     time.sleep(5)
            #     self.stopForceMode()
            #     time.sleep(30)

            previousPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
            self.moveLegsSync(currentLegsMovingOrder, previousPinsPositions, config.GLOBAL_ORIGIN, 1.5, config.MINJERK_TRAJECTORY, pose)
            time.sleep(2)

            pinsOffsets = currentPinsPositions - previousPinsPositions
            for idx, leg in enumerate(currentLegsMovingOrder):
                if pinsOffsets[idx].any():
                    self.moveLegFromPinToPin(leg, currentPinsPositions[idx], previousPinsPositions[idx])        
            self.distributeForces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION)
        
        # self.isWalking = False
        # print("Walking thread stopped.")
    #endregion