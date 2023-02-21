import numpy as np
import threading
import time

import config
import controllers
import threadmanager
import jsonfilemanager
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
        self.tauA = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.hwErrors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        self.qCd = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)

        self.motorsVelocityController = controllers.VelocityController()
        self.motorDriver = dmx.MotorDriver([[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]])
        self.pumpsBnoArduino = waterpumpsbno.PumpsBnoArduino()
        self.threadManager = threadmanager.CustomThread()
        self.jsonFileManager = jsonfilemanager.JsonFileManager()
        
        self.statesObjectsLocker = threading.Lock()
        self.safetyKillEvent = threading.Event()
        self.unsuccessfullLegMovementKillEvent = threading.Event()

        self.currentState = None

        self.__initLayers()

    def safetyLayer(self):
        """Continuously checking for hardware errors on the motors. If error occurs, inititate resting procedure. 
        """
        def safetyChecking(killEvent):
            self.safetyKillEvent.clear()
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    hwErrors = self.hwErrors

                isHwError = hwErrors.any() and self.currentState == config.WORKING_STATE
                isLegMovementError = self.unsuccessfullLegMovementKillEvent.is_set()

                if isHwError or isLegMovementError:
                    print("SAFETY")
                    if isHwError:
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
                    self.unsuccessfullLegMovementKillEvent.clear()
                time.sleep(0.1)
        self.safetyThread, self.safetyThreadKillEvent = self.threadManager.run(safetyChecking, config.SAFETY_THREAD_NAME, False, True)
    
    def motorControlLayer(self):
        """Motors control layer with reading, recalculating and sending data to the motors.
        """
        def controlLoop(killEvent):
            fBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauCounter = 0
            fCounter = 0
            init = True
            self.motorDriver.setBusWatchdog(10)
            while True:
                startTime = time.perf_counter()

                if killEvent.is_set():
                    break
                
                # Reading data from motors.
                try:
                    qA, iA, hwErrors, temperatures = self.motorDriver.syncReadMotorsData()
                except KeyError:
                    print("Reading error.")
                    continue

                # Calculating input data for controller.
                xA = kin.allLegsPositions(qA, config.LEG_ORIGIN)
                tau, f = dyn.getTorquesAndForcesOnLegsTips(qA, iA, self.pumpsBnoArduino.getGravityVector())
                tauMean, tauBuffer, tauCounter = mathtools.runningAverage(tauBuffer, tauCounter, tau)
                fMean, fBuffer, fCounter = mathtools.runningAverage(fBuffer, fCounter, f)

                with self.statesObjectsLocker:
                    self.qA = qA
                    self.hwErrors = hwErrors
                    self.temperatures = temperatures
                    self.xA = xA
                    self.tauA = tauMean

                # Controller.
                qCd = self.motorsVelocityController.jointsVelocityController(qA, xA, fMean, init)           
                if init:
                    init = False

                # Sending velocities to motors.
                self.motorDriver.syncWriteMotorsVelocitiesInLegs(qCd)

                # Enforce desired frequency.
                elapsedTime = time.perf_counter() - startTime
                while elapsedTime < (1 / config.CONTROLLER_FREQUENCY):
                    elapsedTime = time.perf_counter() - startTime
                    time.sleep(0)

        self.motorControlThread, self.motorControlThreadKillEvent = self.threadManager.run(controlLoop, config.CONTROL_THREAD_NAME, False, True)

    def spiderStatesManager(self, state, workingArs = None):
        """Managing spider's state.

        Args:
            state (string): Name of desired state.
            workingArs (tuple, optional): Neede parameters for desired state. Defaults to None.
        """
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
                # Tukaj je potrebno lock-at branje te spremenljivke in sicer z istim lockerjem, kot ga uporabis za pisanje v to spremenljivko. 
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
                self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, previousPinsPositions, config.GLOBAL_ORIGIN, 2.5, config.MINJERK_TRAJECTORY, pose)
                updateDictThread = self.threadManager.run(self.jsonFileManager.updateWholeDict, config.UPDATE_DICT_THREAD_NAME, True, True, False, (pose, previousPinsPositions, currentLegsMovingOrder, ))
                if self.safetyKillEvent.wait(timeout = 3.5):
                    break
                updateDictThread.join()

                pinsOffsets = currentPinsPositions - previousPinsPositions
                for idx, leg in enumerate(currentLegsMovingOrder):
                    if pinsOffsets[idx].any():
                        pinToPinMovementSuccess = self.__pinToPinMovement(leg, previousPinsPositions[idx], currentPinsPositions[idx])
                        if not pinToPinMovementSuccess:
                            break
            
                if not pinToPinMovementSuccess:
                    break

            if self.safetyKillEvent.is_set() or not pinToPinMovementSuccess:
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

        self.motorsVelocityController.clearInstructionQueues()

        with self.statesObjectsLocker:
            hwErrors = self.hwErrors

        attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()
        unattachedLeg = np.setdiff1d(spider.LEGS_IDS, attachedLegs)

        if unattachedLeg.size != 0:
            _, usedPinsIndexes, legsGlobalPositions = self.jsonFileManager.readPosePins()
            unattachedLegGoalPin = usedPinsIndexes[unattachedLeg[0]]
            print(f"MANUALLY CORRECT LEG {unattachedLeg[0]} ON PIN {unattachedLegGoalPin}")

            with self.statesObjectsLocker:
                qA = self.qA
                # xAUnattachedLeg = self.xA[unattachedLeg[0]]

            spiderPose = tf.xyzRpyToMatrix(kin.getSpiderPose(attachedLegs, legsGlobalPositions, qA))

            self.__manualCorrection(unattachedLeg[0], spiderPose, useSafety = False)
            # goalPinInSpider = np.dot(np.linalg.inv(spiderPose), np.append(legsGlobalPositions[unattachedLeg[0]], 1))
            # goalPinInLocal = np.dot(np.linalg.inv(spider.T_ANCHORS[unattachedLeg[0]]), goalPinInSpider)[:3]

            # self.motorsVelocityController.startForceMode(unattachedLeg, [np.zeros(3, dtype = np.float32)])
            # distance = np.linalg.norm(goalPinInLocal - xAUnattachedLeg)
            # while not len(attachedLegs) == 5 and distance > 0.08:
            #     with self.statesObjectsLocker:
            #         xAUnattachedLeg = self.xA[unattachedLeg[0]]
            #     distance = np.linalg.norm(goalPinInLocal - xAUnattachedLeg)
            #     attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()
            #     time.sleep(0.2)
            # self.motorsVelocityController.stopForceMode()

        print("LEGS DISABLED")
        self.motorDriver.disableLegs()

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
        time.sleep(10)
        self.motorsVelocityController.stopForceMode()
        print("TRANSITION FINISHED")

    # region private methods
    def __initLayers(self):
        self.safetyLayer()
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
    
    def __pinToPinMovement(self, leg, currentPinPosition, goalPinPosition, pose):
        otherLegs = np.delete(spider.LEGS_IDS, leg)
        self.__distributeForces(otherLegs, config.FORCE_DISTRIBUTION_DURATION)

        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        if self.safetyKillEvent.wait(timeout = 1.5):
            return False

        rpy = self.pumpsBnoArduino.getRpy()
        pinToPinLocal, legOriginOrientationInGlobal = tf.getPinToPinVectorInLocal(leg, rpy, currentPinPosition, goalPinPosition)
        globalZDirectionInLegOrigin = np.dot(legOriginOrientationInGlobal, np.array([0.0, 0.0, 1.0], dtype = np.float32))

        with self.statesObjectsLocker:
            xALeg = self.xA[leg]
        self.motorsVelocityController.moveLegAsync(leg, xALeg, pinToPinLocal, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, isOffset = True)
        updateDictThread = self.threadManager.run(self.jsonFileManager.updatePins, config.UPDATE_DICT_THREAD_NAME, True, True, False, (leg, goalPinPosition, ))
        if self.safetyKillEvent.wait(timeout = 3.5):
            return False
        updateDictThread.join()

        self.motorsVelocityController.startForceMode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
        if self.safetyKillEvent.wait(timeout = 3.0):
            self.motorsVelocityController.stopForceMode()
            return False
        self.motorsVelocityController.stopForceMode()

        detachOffsetZ = 0.03
        detachOffsetInLocal = globalZDirectionInLegOrigin * detachOffsetZ
        numberOfTries = 0
        # while not (leg in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()):
        #     print(f"LEG {leg} NOT ATTACHED")
        #     if numberOfTries <= 2:
        #         self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        #         if self.safetyKillEvent.wait(timeout = 1.0):
        #             return False      

        #         with self.statesObjectsLocker:
        #             xALeg = self.xA[leg]

        #         self.motorsVelocityController.moveLegAsync(leg, xALeg, detachOffsetInLocal, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY, isOffset = True)
        #         if self.safetyKillEvent.wait(timeout = 1.0):
        #             return False
                
        #         self.motorsVelocityController.startImpedanceMode(leg, -globalZDirectionInLegOrigin)
        #         if self.safetyKillEvent.wait(timeout = 3.0):
        #             return False
        #         self.motorsVelocityController.stopImpedanceMode()

        #         self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
        #         if self.safetyKillEvent.wait(timeout = 1.0):
        #             return False  
            
        #         numberOfTries += 1
        
        if leg not in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            print(f"LEG {leg} NOT ATTACHED")
            # First try to correct with impedance mode.
            while numberOfTries <= 2:
                self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
                if self.safetyKillEvent.wait(timeout = 1.0):
                    return False      

                with self.statesObjectsLocker:
                    xALeg = self.xA[leg]

                self.motorsVelocityController.moveLegAsync(leg, xALeg, detachOffsetInLocal, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY, isOffset = True)
                if self.safetyKillEvent.wait(timeout = 1.0):
                    return False
                
                self.motorsVelocityController.startImpedanceMode(leg, -globalZDirectionInLegOrigin)
                if self.safetyKillEvent.wait(timeout = 3.0):
                    return False
                self.motorsVelocityController.stopImpedanceMode()

                self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
                if self.safetyKillEvent.wait(timeout = 1.0):
                    return False 

                if leg in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
                    return True 
            
                numberOfTries += 1
            
            # If impedance mode doesn't do the trick, manually correct leg.
            return self.__manualCorrection(leg, pose)
        
        return True
    
    def __manualCorrection(self, legId, spiderPose, useSafety = True):
        _, usedPinsIndexes, legsGlobalPositions = self.jsonFileManager.readPosePins()
        goalPin = usedPinsIndexes[legId]

        print(f"MANUALLY CORRECT LEG {legId} ON PIN {goalPin}")

        goalPinInSpider = np.dot(np.linalg.inv(spiderPose), np.append(legsGlobalPositions[legId], 1))
        goalPinInLocal = np.dot(np.linalg.inv(spider.T_ANCHORS[legId]), goalPinInSpider)[:3]

        with self.statesObjectsLocker:
            xALeg = self.xA[legId]
        distance = np.linalg.norm(goalPinInLocal - xALeg)
        self.motorsVelocityController.startForceMode(np.array([legId]), np.array([np.zeros(3, dtype = np.float32)]))
        while True:
            with self.statesObjectsLocker:
                xALeg = self.xA[legId]
            distance = np.linalg.norm(goalPinInLocal - xALeg)
            if legId in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs() and distance < 0.08:
                break
            if useSafety and self.safetyKillEvent.wait(timeout = 0.01):
                self.motorsVelocityController.stopForceMode()
                return False
        self.motorsVelocityController.stopForceMode()

        return True      
    #endregion