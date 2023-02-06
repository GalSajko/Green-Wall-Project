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
                    qA, iA, hwErrors = self.motorDriver.syncReadMotorsData()
                    with self.statesObjectsLocker:
                        self.qA = qA
                        self.iA = iA
                        # self.hwErrors = hwErrors
                except KeyError:
                    print("Reading error.")
                time.sleep(0.001)
        self.readingThread, self.readingThreadKillEvent = self.threadManager.run(reading, config.DXL_READING_THREAD_NAME, False, True)

    def writingLayer(self):
        def writing(killEvent):
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    qCd = self.qCd
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(qCd)
                time.sleep(0.001)
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
                if hwErrors.any():
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
                        while self.spiderStateThread.is_alive():
                            time.sleep(0.001)
                    self.spiderStatesManager(config.TRANSITION_STATE)
                    # Wait for transition state to finish
                    if config.TRANSITION_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                    self.spiderStatesManager(config.RESTING_STATE)
                    # self.safetyKillEvent.clear()
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
            self.rest()
        elif state == config.TRANSITION_STATE:
            self.spiderStateThread = self.threadManager.run(self.transitionToRestState, config.TRANSITION_THREAD_NAME, False, True, False)
    
    def working(self, startPose, endPose, initBno):
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
        print("WORKING STOPED")
        
    def rest(self):
        print("RESTING")
        with self.statesObjectsLocker:
            self.qCd = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            self.hwErrors = np.zeros((5, 3), dtype = np.float32)
        self.motorDriver.disableLegs()
        print("LEGS DISABLED")
    
    def transitionToRestState(self):
        print("WAIT FOR TRANSITION")
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