import numpy as np
import threading
import time

import config
import controllers
import threadmanager
import jsonfilemanager
from periphery import dynamixel as dmx
from periphery import waterpumpsbno
from environment.comunication import comunication 
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
        self.jsonFileManager = jsonfilemanager.JsonFileManager()
        self.comunicationManager = comunication()
        self.statesObjectsLocker = threading.Lock()
        self.safetyKillEvent = threading.Event()

        self.currentState = None
        self.stateDict = {}

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
                time.sleep(0)
        self.readingThread, self.readingThreadKillEvent = self.threadManager.run(reading, config.DXL_READING_THREAD_NAME, False, True)

    def writingLayer(self):
        def writing(killEvent):
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    qCd = self.qCd
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(qCd)
                time.sleep(0)
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

    def updatePositionData(self):
        def updatingPositionData(killEvent):
            while 1:
                if killEvent.is_set():
                    break
                self.comunicationManager.update_values()
                time.sleep(1)
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.threadManager.run(updatingPositionData, config.UPDATE_DATA_THREAD_NAME, False, True)
        

    def safetyLayer(self):
        def safetyChecking(killEvent):
            self.safetyKillEvent.clear()
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    hwErrors = self.hwErrors
                if hwErrors.any() and self.currentState == config.WORKING_STATE:
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
        """Working procedure, includes walking and watering the plants.

        Args:
            startPose (list): Initial starting pose.
        """
        import random
        self.currentState = config.WORKING_STATE
        initBno = True
        print("WORKING...")
        while True:

            # TODO: Here comes received goal point.
            try:
                sensor = self.comunicationManager.data[2]
                line = self.comunicationManager.data[1]
            except:
                continue
                # TODO: logic for errors
            endPose = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.3, 0.0], dtype = np.float32)

            poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)
            for step, pose in enumerate(poses):
                currentPinsPositions = pinsInstructions[step, :, 1:]
                currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)

                with self.statesObjectsLocker:
                    xA = self.xA

                if step == 0:
                    self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, currentPinsPositions, config.GLOBAL_ORIGIN, 3, config.MINJERK_TRAJECTORY, pose)
                    updateDictThread = self.threadManager.run(self.jsonFileManager.updateWholeDict, config.UPDATE_DICT_THREAD_NAME, True, True, False, (pose, currentPinsPositions, currentLegsMovingOrder, ))
                    if self.safetyKillEvent.wait(timeout = 3.5):
                        break
                    updateDictThread.join()

                    if initBno:
                        self.pumpsBnoArduino.resetBno()
                        initBno = False
                        if self.safetyKillEvent.wait(timeout = 1.0):
                            break
                    self.__distributeForces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION)
                    continue
            
                previousPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
                self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, previousPinsPositions, config.GLOBAL_ORIGIN, 1.5, config.MINJERK_TRAJECTORY, pose)
                updateDictThread = self.threadManager.run(self.jsonFileManager.updateWholeDict, config.UPDATE_DICT_THREAD_NAME, True, True, False, (pose, previousPinsPositions, currentLegsMovingOrder, ))
                if self.safetyKillEvent.wait(timeout = 2.5):
                    break
                updateDictThread.join()

                pinsOffsets = currentPinsPositions - previousPinsPositions
                for idx, leg in enumerate(currentLegsMovingOrder):
                    if pinsOffsets[idx].any():
                        # Distribute forces among other legs and release selected leg.
                        otherLegs = np.delete(spider.LEGS_IDS, leg)
                        self.__distributeForces(otherLegs, config.FORCE_DISTRIBUTION_DURATION)
                        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
                        if self.safetyKillEvent.wait(timeout = 1.5):
                            break

                        # Read spider's orientation to use it for adjustment of pin-to-pin leg's movement direction.
                        rpy = self.pumpsBnoArduino.getRpy()
                        pinToPinLocal = tf.getPinToPinVectorInLocal(leg, rpy, previousPinsPositions[idx], currentPinsPositions[idx])

                        # Move leg from current to next pin.
                        with self.statesObjectsLocker:
                            xALeg = self.xA[leg]
                        self.motorsVelocityController.moveLegAsync(leg, xALeg, pinToPinLocal, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, isOffset = True)

                        updateDictThread = self.threadManager.run(self.jsonFileManager.updatePins, config.UPDATE_DICT_THREAD_NAME, True, True, False, (leg, currentPinsPositions[idx], ))
                        if self.safetyKillEvent.wait(timeout = 3.5):
                            break
                        updateDictThread.join()

                        self.motorsVelocityController.startForceMode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
                        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
                        if self.safetyKillEvent.wait(timeout = 3.0):
                            self.motorsVelocityController.stopForceMode()
                            break
                        self.motorsVelocityController.stopForceMode()
                        # TODO: Add correction logic in case of leg missed the pin.

            if self.safetyKillEvent.is_set():
                break
        print("WORKING STOPED")
        
    def rest(self, killEvent):
        """Resting procedure, includes option for manually correcting non-attached leg. Resting lasts until temperatures of all motors
        drop below working temperature.

        Args:
            killEvent (event): Event for killing a procedure.
        """
        print("RESTING...")
        self.currentState = config.RESTING_STATE

        with self.statesObjectsLocker:
            self.qCd = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            hwErrors = self.hwErrors
        attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()

        unattachedLeg = np.setdiff1d(spider.LEGS_IDS, attachedLegs)
        if unattachedLeg.size == 0:
            self.motorDriver.disableLegs()
            print("LEGS DISABLED")
        else:
            _, usedPinsIndexes, legsGlobalPositions = self.jsonFileManager.readPosePins()
            unattachedLegGoalPin = usedPinsIndexes[unattachedLeg[0]]
            print(f"MANUALLY CORRECT LEG {unattachedLeg[0]} ON PIN {unattachedLegGoalPin}")

            with self.statesObjectsLocker:
                qA = self.qA
                xAUnattachedLeg = self.xA[unattachedLeg[0]]

            spiderPose = tf.xyzRpyToMatrix(kin.getSpiderPose(attachedLegs, legsGlobalPositions, qA))
            goalPinInSpider = np.dot(np.linalg.inv(spiderPose), np.append(unattachedLegGoalPin, 1))[:3]

            self.motorsVelocityController.grippersArduino.moveGripper(unattachedLeg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
            self.motorsVelocityController.startForceMode(unattachedLeg, [np.zeros(3, dtype = np.float32)])
            distance = np.linalg.norm(goalPinInSpider - xAUnattachedLeg)
            while not len(attachedLegs) == 5 and distance > 0.08:
                with self.statesObjectsLocker:
                    xAUnattachedLeg = self.xA[unattachedLeg[0]]
                distance = np.linalg.norm(goalPinInSpider - xAUnattachedLeg)
                attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()
                time.sleep(0.2)
            self.motorsVelocityController.stopForceMode()
            self.motorDriver.disableLegs()

        # Reboot all motors that are in error state.
        motorsInError = self.motorDriver.motorsIds[np.where(hwErrors != 0)]
        print("REBOOTING MOTORS WITH IDS: ", motorsInError, "...")
        self.motorDriver.rebootMotor(motorsInError)
        # Rest until temperature drops below working temperature.
        print("WAITING ON TEMPERATURES TO DROP BELOW WORKING TEMPERATURE...")
        while True:
            with self.statesObjectsLocker:
                temperatures = self.temperatures
            if killEvent.wait(timeout = 1) or (temperatures < self.motorDriver.MAX_WORKING_TEMPERATURE).all():
                break
        time.sleep(10)

        # Update last positions and enable torques in motors.
        with self.statesObjectsLocker:
            xA = self.xA
        self.motorsVelocityController.updateLastLegsPositions(xA)
        self.motorDriver.enableLegs()
        print("LEGS ENABLED, RESTING FINISHED")
    
    def transitionToRestState(self):
        """Procedure for transition between working and resting state, by using force mode. 
        """
        print("TRANSITION INTO RESTING STATE...")
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
    
    def __distributeForces(self, legsIds, duration):
        """Run force distribution process in a loop for a given duration.
        """
        offloadLegId = np.setdiff1d(spider.LEGS_IDS, legsIds)
        if len(offloadLegId) > 1:
            print("Cannot offload more than one leg at the same time.")
            return False

        startTime = time.perf_counter()
        elapsedTime = 0
        while elapsedTime < duration:
            with self.statesObjectsLocker:
                tauA = self.tauA
                qA = self.qA
            fDist = dyn.calculateDistributedForces(tauA, qA, legsIds, offloadLegId)

            if len(offloadLegId):
                fDist = np.insert(fDist, offloadLegId[0], np.zeros(3, dtype = np.float32), axis = 0)
            self.motorsVelocityController.startForceMode(spider.LEGS_IDS, fDist)

            elapsedTime = time.perf_counter() - startTime
            time.sleep(self.motorsVelocityController.period)
        
        self.motorsVelocityController.stopForceMode()