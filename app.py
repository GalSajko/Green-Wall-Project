import numpy as np
import threading
import time

import config
import controllers
import threadmanager
from periphery import dynamixel as dmx
from periphery import waterpumpsbno
from environment import spider
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools
from calculations import transformations as tf
from planning import pathplanner

class App:
    def __init__(self):
        self.qA = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.xA = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.iA = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.fA = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tauA = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.hwErrors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.qCd = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)

        self.motorsVelocityController = controllers.VelocityController()
        self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.pumpsBnoArduino = waterpumpsbno.PumpsBnoArduino()
        self.threadManager = threadmanager.CustomThread()

        self.statesObjectsLocker = threading.Lock()
        self.safetyKillEvent = threading.Event()

        self.currentState = None

        self.__initThreads()

    def readingLayer(self):
        def reading(killEvent):
            while True:
                if killEvent.is_set():
                    break
                try:
                    qA, iA, hwErrors, temperatures = self.motorDriver.syncReadMotorsData()
                    with self.statesObjectsLocker:
                        self.qA = qA
                        self.iA = iA
                        self.hwErrors = hwErrors
                        self.temperatures = temperatures
                except KeyError:
                    print("Reading error.")
                time.sleep(0.005)
        self.readingThread, self.readingThreadKillEvent = self.threadManager.run(reading, config.DXL_READING_THREAD_NAME, False, True)

    def writingLayer(self):
        def writing(killEvent):
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    qCd = self.qCd
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(qCd)
                time.sleep(0.005)
        self.writingThread, self.writingThreadKillEvent = self.threadManager.run(writing, config.DXL_WIRITNG_THREAD_NAME, False, True)
    
    def convertingLayer(self):
        def converting(killEvent):
            fBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauCounter = 0
            fCounter = 0
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    qA = self.qA
                    iA = self.iA
                xA = kin.allLegsPositions(qA, config.LEG_ORIGIN)
                tau, f = dyn.getTorquesAndForcesOnLegsTips(qA, iA, self.pumpsBnoArduino.getGravityVector())
                tauMean, tauBuffer, tauCounter = mathtools.runningAverage(tauBuffer, tauCounter, tau)
                fMean, fBuffer, fCounter = mathtools.runningAverage(fBuffer, fCounter, f)
                with self.statesObjectsLocker:
                    self.xA = xA
                    self.tauA = tauMean
                    self.fA = fMean
                time.sleep(0)
        self.convertingCalcThread, self.convertingCalcThreadKillEvent = self.threadManager.run(converting, config.CONVERTING_THREAD_NAME, False, True)
    
    def safetyLayer(self):
        def safetyChecking(killEvent):
            self.safetyKillEvent.clear()
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    hwErrors = self.hwErrors
                if hwErrors.any() and self.currentState == config.WORKING_STATE:
                    """TODO:
                    1. set safety kill event to kill working thread.
                    2. wait for working thread to stop.
                    3. start transition thread.
                    4. wait for transition thread to finish.
                    5. start rest mode (disable  and reboot motors).
                    """
                    self.safetyKillEvent.set()
                    # Wait for working thread to stop.
                    if config.WORKING_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                    self.spiderStatesManager(config.TRANSITION_STATE)
                    # Wait for transition state to finish.
                    if config.TRANSITION_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                    # Start resting state and wait for it to finish.
                    self.spiderStatesManager(config.RESTING_STATE)
                    if config.RESTING_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                    self.safetyKillEvent.clear()
                time.sleep(0.1)
        self.safetyThread, self.safetyThreadKillEvent = self.threadManager.run(safetyChecking, config.SAFETY_THREAD_NAME, False, True)
    
    def motorControlLayer(self):
        def controlLoop(killEvent):
            init = True
            self.motorDriver.setBusWatchdog(10)
            while True:
                startTime = time.perf_counter()
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    qA = self.qA
                    xA = self.xA
                    fA = self.fA
                qCd = self.motorsVelocityController.jointsVelocityController(qA, xA, fA, init)
                if init:
                    init = False
                with self.statesObjectsLocker:
                    self.qCd = qCd
                elapsedTime = time.perf_counter() - startTime
                while elapsedTime < (1 / config.CONTROLLER_FREQUENCY):
                    elapsedTime = time.perf_counter() - startTime
                    time.sleep(0)
        self.motorControlThread, self.motorControlThreadKillEvent = self.threadManager.run(controlLoop, config.CONTROL_THREAD_NAME, False, True)

    
    def spiderStatesManager(self, state, workingArs = None):
        if state == config.WORKING_STATE:
            self.spiderStateThread = self.threadManager.run(self.working, config.WORKING_THREAD_NAME, False, True, False, workingArs)
        elif state == config.RESTING_STATE:
            self.spiderStatesThread, self.spiderStatesThreadKillEvenet = self.threadManager.run(self.rest, config.RESTING_THREAD_NAME, False, True, True)
        elif state == config.TRANSITION_STATE:
            self.spiderStateThread = self.threadManager.run(self.transitionToRestState, config.TRANSITION_THREAD_NAME, False, True, False)
    
    def working(self, startPose):
        """TODO:
        Add logic for receiving start and end pose instructions from web interface (data from plants)."""
        import random
        self.currentState = config.WORKING_STATE
        initBno = True
        print("WORKING...")
        while True:
            # Here comes received end pose.
            endPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)
            poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)
            for step, pose in enumerate(poses):
                currentPinsPositions = pinsInstructions[step, :, 1:]
                currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)

                with self.statesObjectsLocker:
                    xA = self.xA

                if step == 0:
                    self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, currentPinsPositions, config.GLOBAL_ORIGIN, 3, config.MINJERK_TRAJECTORY, pose)
                    if self.safetyKillEvent.wait(timeout = 3.5):
                        break
                    if initBno:
                        self.pumpsBnoArduino.resetBno()
                        initBno = False
                        if self.safetyKillEvent.wait(timeout = 1.0):
                            break
                        with self.statesObjectsLocker:
                            qA = self.qA
                            tauA = self.tauA
                    self.motorsVelocityController.distributeForces(spider.LEGS_IDS, qA, tauA, config.FORCE_DISTRIBUTION_DURATION)
                    continue
            
                previousPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
                self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, previousPinsPositions, config.GLOBAL_ORIGIN, 1.5, config.MINJERK_TRAJECTORY, pose)
                if self.safetyKillEvent.wait(timeout = 2.5):
                    break

                pinsOffsets = currentPinsPositions - previousPinsPositions
                for idx, leg in enumerate(currentLegsMovingOrder):
                    if pinsOffsets[idx].any():
                        with self.statesObjectsLocker:
                            qA = self.qA
                            tauA = self.tauA
                        self.motorsVelocityController.releaseOneLeg(leg, qA, tauA)
                        rpy = self.pumpsBnoArduino.getRpy()
                        pinToPinLocal = tf.getPinToPinVectorInLocal(leg, rpy, previousPinsPositions[idx], currentPinsPositions[idx])
                        with self.statesObjectsLocker:
                            xALeg = self.xA[leg]
                        self.motorsVelocityController.moveLegAsync(leg, xALeg, pinToPinLocal, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, isOffset = True)
                        if self.safetyKillEvent.wait(timeout = 3.5):
                            break
                        self.motorsVelocityController.startForceMode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
                        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
                        if self.safetyKillEvent.wait(timeout = 3.0):
                            self.motorsVelocityController.stopForceMode()
                            break
                        self.motorsVelocityController.stopForceMode()
            if self.safetyKillEvent.is_set():
                break
        print("WORKING STOPED")
        
    def rest(self, killEvent):
        print("RESTING...")
        self.currentState = config.RESTING_STATE
        # Disable all attached legs.
        with self.statesObjectsLocker:
            self.qCd = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            hwErrors = self.hwErrors
        # attachedLegs = [int(i) for i in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()]
        # self.motorDriver.disableLegs(attachedLegs)
        attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()
        self.motorDriver.disableLegs()
        print("LEGS DISABLED")

        # TODO: Put un-attached leg in force-mode (fD = 0) to allow manual movement on correct pin. 
        # When leg is attached, check if pin is correct and continue with resting procedure.
        # unattachedLeg = np.setdiff1d(spider.LEGS_IDS, attachedLegs)
        # self.motorsVelocityController.startForceMode(unattachedLeg, [np.zeros(3, dtype = np.float32)])
        while not len(attachedLegs) == 5:
            attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()
            time.sleep(0.2)
        # self.motorsVelocityController.stopForceMode()

        # Reboot all motors that are in error state.
        motorsInError = self.motorDriver.motorsIds[np.where(hwErrors != 0)]
        print("REBOOTING MOTORS WITH IDS: ", motorsInError)
        self.motorDriver.rebootMotor(motorsInError)
        # Rest until temperature drops below working temperature.
        while True:
            with self.statesObjectsLocker:
                temperatures = self.temperatures
            if killEvent.wait(timeout = 1) or (temperatures < self.motorDriver.MAX_WORKING_TEMPERATURE).all():
                break
        time.sleep(10)
        # Update last positions and enable torques in motors.
        with self.statesObjectsLocker:
            self.motorsVelocityController.updateLastLegsPositions(self.xA)
        self.motorDriver.enableLegs()

        print("RESTING FINISHED")
    
    def transitionToRestState(self):
        print("WAITING FOR TRANSITION...")
        self.currentState = config.TRANSITION_STATE
        self.motorsVelocityController.clearInstructionQueues()
        fD = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.motorsVelocityController.startForceMode(spider.LEGS_IDS, fD)
        time.sleep(6)
        self.motorsVelocityController.stopForceMode()
        print("TRANSITION FINISHED")

    def __initThreads(self):
        self.readingLayer()
        self.writingLayer()
        self.safetyLayer()
        self.convertingLayer()
        time.sleep(2)
        self.motorControlLayer()