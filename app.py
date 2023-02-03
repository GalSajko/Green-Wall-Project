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
        # self.pumpsBnoArduino = waterpumpsbno.PumpsBnoArduino()
        self.threadManager = threadmanager.CustomThread()

        self.statesObjectsLocker = threading.Lock()
        self.safetyKillEvent = threading.Event()

        self.__initThreads()

    def readingLayer(self):
        def reading(killEvent):
            while True:
                try:
                    qA, iA, hwErrors = self.motorDriver.syncReadMotorsData()
                    with self.statesObjectsLocker:
                        self.qA = qA
                        self.iA = iA
                        self.hwErrors = hwErrors
                except KeyError:
                    print("Reading error.")
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.readingThread, self.readingThreadKillEvent = self.threadManager.run(reading, 'dxl_reading_thread', False, True)

    def writingLayer(self):
        def writing(killEvent):
            while True:
                with self.statesObjectsLocker:
                    qCd = self.qCd
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(qCd)
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.writingThread, self.writingThreadKillEvent = self.threadManager.run(writing, 'dxl_writing_thread', False, True)
    
    def convertingLayer(self):
        def converting(killEvent):
            fBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauCounter = 0
            fCounter = 0
            while True:
                with self.statesObjectsLocker:
                    qA = self.qA
                    iA = self.iA
                xA = kin.allLegsPositions(qA, config.LEG_ORIGIN)
                tau, f = dyn.getTorquesAndForcesOnLegsTips(qA, iA, self.motorsVelocityController.pumpsBnoArduino.getGravityVector())
                tauMean, tauBuffer, tauCounter = mathtools.runningAverage(tauBuffer, tauCounter, tau)
                fMean, fBuffer, fCounter = mathtools.runningAverage(fBuffer, fCounter, f)
                with self.statesObjectsLocker:
                    self.xA = xA
                    self.tauA = tauMean
                    self.fA = fMean
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.convertingCalcThread, self.convertingCalcThreadKillEvent = self.threadManager.run(converting, 'converting_calc_thread', False, True)
    
    def safetyLayer(self):
        def safetyChecking(killEvent):
            while True:
                with self.statesObjectsLocker:
                    hwErrors = self.hwErrors
                if hwErrors.any():
                    self.safetyKillEvent.set()
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.safetyThread, self.safetyThreadKillEvent = self.threadManager.run(safetyChecking, 'safety_thread', False, True)
    
    def motorControlLayer(self):
        def controlLoop(killEvent):
            init = True
            self.motorDriver.setBusWatchdog(10)
            while True:
                startTime = time.perf_counter()
                with self.statesObjectsLocker:
                    qA = self.qA
                    xA = self.xA
                qCd = self.motorsVelocityController.jointsVelocityController(qA, xA, init)
                if init:
                    init = False
                with self.statesObjectsLocker:
                    self.qCd = qCd
                if killEvent.is_set():
                    break
                elapsedTime = time.perf_counter() - startTime
                while elapsedTime < (1 / config.CONTROLLER_FREQUENCY):
                    elapsedTime = time.perf_counter() - startTime
                    time.sleep(0)
        self.motorControlThread, self.motorControlThreadKillEvent = self.threadManager.run(controlLoop, 'control_thread', False, True)
    
    def walk(self, startPose, endPose):
        print("START WALKING")
        # self.motorsVelocityController.moveLegAsync(2, self.xA[2], [0.4, 0.0, 0.1], config.LEG_ORIGIN, 3, config.MINJERK_TRAJECTORY)
        # self.motorsVelocityController.moveLegsSync(spider.LEGS_IDS, self.xA, [[0.4, 0.0, 0.1]] * 5, config.LEG_ORIGIN, 3, config.MINJERK_TRAJECTORY)

        poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)
        for step, pose in enumerate(poses):
            # if self.killWalkingThreadEvent.is_set():
            #     break
            currentPinsPositions = pinsInstructions[step, :, 1:]
            currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)

            with self.statesObjectsLocker:
                xA = self.xA

            if step == 0:
                self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, currentPinsPositions, config.GLOBAL_ORIGIN, 3, config.MINJERK_TRAJECTORY, pose)
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
            self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, previousPinsPositions, config.GLOBAL_ORIGIN, 1.5, config.MINJERK_TRAJECTORY, pose)
            time.sleep(2)

            pinsOffsets = currentPinsPositions - previousPinsPositions
            for idx, leg in enumerate(currentLegsMovingOrder):
                if pinsOffsets[idx].any():
                    self.motorsVelocityController.moveLegFromPinToPin(leg, xA[leg], currentPinsPositions[idx], previousPinsPositions[idx])    

    def __initThreads(self):
        self.readingLayer()
        self.writingLayer()
        self.convertingLayer()
        time.sleep(2)
        self.motorControlLayer()