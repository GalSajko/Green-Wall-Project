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

        self.systemVariablesLocker = threading.Lock()

    def readingLayer(self):
        def reading(killEvent):
            while True:
                qA, iA, hwErrors = self.motorDriver.syncReadMotorsData()
                with self.systemVariablesLocker:
                    self.qA = qA
                    self.iA = iA
                    self.hwErrors = hwErrors
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.readingThread, self.readingThreadKillEvent = self.threadManager.run(reading, 'dxl_reading_thread', True, True)

    def writingLayer(self):
        def writing(killEvent):
            while True:
                with self.systemVariablesLocker:
                    qCd = self.qCd
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(qCd)
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.writingThread, self.writingThreadKillEvent = self.threadManager.run(writing, 'dxl_writing_thread', True, True)
    
    def convertingLayer(self):
        def converting(killEvent):
            fBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauCounter = 0
            fCounter = 0
            while True:
                with self.systemVariablesLocker:
                    qA = self.qA
                    iA = self.iA
                xA = kin.allLegsPositions(qA, config.LEG_ORIGIN)
                tau, f = dyn.getTorquesAndForcesOnLegsTips(qA, iA, self.pumpsBnoArduino.getGravityVector())
                tauMean, tauBuffer, tauCounter = mathtools.runningAverage(tauBuffer, tauCounter, tau)
                fMean, fBuffer, fCounter = mathtools.runningAverage(fBuffer, fCounter, f)
                with self.systemVariablesLocker:
                    self.xA = xA
                    self.tauA = tauMean
                    self.fA = fMean
                if killEvent.is_set():
                    break
                time.sleep(0)
        self.convertingCalcThread, self.convertingCalcThreadKillEvent = self.threadManager.run(converting, 'converting_calc_thread', True, True)
    
    def motorsCotrolLayer(self):

                
