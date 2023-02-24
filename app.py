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

        self.currentState = None
        self.wateringCounter = 0

        self.__initLayers()

    def safetyLayer(self):
        """Continuously checking for hardware errors on the motors. If error occurs, inititate resting procedure and then continue with working. 
        """
        def safetyChecking(killEvent):
            self.safetyKillEvent.clear()
            while True:
                if killEvent.is_set():
                    break
                with self.statesObjectsLocker:
                    hwErrors = self.hwErrors

                isHwError = hwErrors.any() and self.currentState == config.WORKING_STATE

                if isHwError:
                    self.safetyKillEvent.set()
                    # Wait for working thread to stop.
                    if config.WORKING_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                        print("WORKING STOPED.")

                    # Start transition state and wait for it to finish.
                    self.spiderStatesManager(config.TRANSITION_STATE)
                    if config.TRANSITION_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                        print("TRANSITION FINISHED")

                    # Start resting state and wait for it to finish.
                    self.spiderStatesManager(config.RESTING_STATE)
                    if config.RESTING_THREAD_NAME in self.spiderStateThread.name:
                        self.spiderStateThread.join()
                        print("RESTING FINISHED")

                    self.safetyKillEvent.clear()
                    time.sleep(1)

                    # Continue with working.
                    self.spiderStatesManager(config.WORKING_STATE)

                time.sleep(0.1)
        self.safetyThread, self.safetyThreadKillEvent = self.threadManager.run(safetyChecking, config.SAFETY_THREAD_NAME, False, True, doPrint = True)
    
    def motorControlLayer(self):
        """Motors control layer with reading, recalculating and sending data to the motors.
        """
        def controlLoop(killEvent):
            fBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauBuffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
            tauCounter = 0
            fCounter = 0
            init = True
            self.motorDriver.setBusWatchdog(15)
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

        self.motorControlThread, self.motorControlThreadKillEvent = self.threadManager.run(controlLoop, config.CONTROL_THREAD_NAME, False, True, doPrint = True)

    def spiderStatesManager(self, state, workingArs = None):
        """Managing spider's state.

        Args:
            state (string): Name of desired state.
            workingArs (tuple, optional): Neede parameters for desired state. Defaults to None.
        """
        if state == config.WORKING_STATE:
            self.spiderStateThread = self.threadManager.run(self.working, config.WORKING_THREAD_NAME, False, True, False, workingArs)
        elif state == config.RESTING_STATE:
            self.spiderStateThread, self.spiderStatesThreadKillEvenet = self.threadManager.run(self.rest, config.RESTING_THREAD_NAME, False, True, True)
        elif state == config.TRANSITION_STATE:
            self.spiderStateThread = self.threadManager.run(self.transitionToRestState, config.TRANSITION_THREAD_NAME, False, True, False)
    
    def working(self):
        """Working procedure, includes walking and watering the plants.

        Args:
            startPose (list): Initial starting pose.
        """
        import random
        self.currentState = config.WORKING_STATE
        isInit = True
        print("WORKING...") 
        
        while True:
            startPose, _, startLegsPositions = self.jsonFileManager.readSpiderState()
            doRefillWaterTank = self.wateringCounter >= config.NUMBER_OF_WATERING_BEFORE_REFILL
            while True:
                try:
                    if doRefillWaterTank:
                        self.wateringCounter = 0
                        wateringLegId, endPose = tf.getWateringLegAndPose(startPose, doRefill = doRefillWaterTank)
                        wateringPosition = endPose[:3] + spider.REFILLING_LEG_OFFSET
                    else:
                        # TODO: Here comes received plant position.
                        wateringPosition = np.array([random.uniform(0.2, 1.0), random.uniform(0.4, 0.8), 0.0], dtype = np.float32)
                        wateringLegId, endPose = tf.getWateringLegAndPose(startPose, wateringPosition)
                    if isInit:
                        poses, pinsInstructions = pathplanner.modifiedWalkingInstructions(startLegsPositions, endPose)
                        break
                    poses, pinsInstructions = pathplanner.createWalkingInstructions(startPose, endPose)
                    break
                except:
                    print("Error in path planning. Trying again... ")

            print("NEW GOAL POINT")
            for step, pose in enumerate(poses):
                currentPinsPositions = pinsInstructions[step, :, 1:]
                currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)
                with self.statesObjectsLocker:
                    xA = self.xA

                if step == 0:
                    self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, currentPinsPositions, config.GLOBAL_ORIGIN, 3, config.MINJERK_TRAJECTORY, pose)
                    updateDictThread = self.threadManager.run(self.jsonFileManager.updateWholeDict, config.UPDATE_DICT_THREAD_NAME, True, True, False, False, (pose, currentPinsPositions, currentLegsMovingOrder, ))
                    if self.safetyKillEvent.wait(timeout = 3.5):
                        return
                    updateDictThread.join()

                    if isInit:
                        self.pumpsBnoArduino.resetBno()
                        isInit = False
                        if self.safetyKillEvent.wait(timeout = 1.0):
                            return
                    self.__distributeForces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION)
                    continue

                previousPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
                self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, previousPinsPositions, config.GLOBAL_ORIGIN, 1.5, config.MINJERK_TRAJECTORY, pose)
                updateDictThread = self.threadManager.run(self.jsonFileManager.updateWholeDict, config.UPDATE_DICT_THREAD_NAME, True, True, False, False, (pose, previousPinsPositions, currentLegsMovingOrder, ))
                if self.safetyKillEvent.wait(timeout = 2.5):
                    print("UNSUCCESSFULL BODY MOVEMENT")
                    return
                updateDictThread.join()

                pinsOffsets = currentPinsPositions - previousPinsPositions
                for idx, leg in enumerate(currentLegsMovingOrder):
                    if pinsOffsets[idx].any():
                        movementSuccess = self.__pinToPinMovement(leg, previousPinsPositions[idx], currentPinsPositions[idx])
                        if not movementSuccess:
                            print("WORKING STOPED - UNSUCCESSFULL PIN TO PIN MOVEMENT")
                            return
            
            wateringSuccess = self.__watering(wateringLegId, wateringPosition, pose, doRefillWaterTank)
            if not wateringSuccess:
                return
            self.wateringCounter += 1
        
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
        motorsInError = self.motorDriver.motorsIds[np.where(hwErrors != 0)]
        print("REBOOTING MOTORS WITH IDS: ", motorsInError, "...")
        self.motorDriver.rebootMotor(motorsInError)
        while hwErrors.any():
            with self.statesObjectsLocker:
                hwErrors = self.hwErrors
            if killEvent.wait(timeout = 0.1):
                break
        if killEvent.is_set():
            return

        if unattachedLeg.size != 0:
            self.__manualCorrection(unattachedLeg[0], useSafety = False)

        # Rest until temperature drops below working temperature.
        print("WAITING ON TEMPERATURES TO DROP BELOW WORKING TEMPERATURE...")
        while True:
            with self.statesObjectsLocker:
                temperatures = self.temperatures
            if killEvent.wait(timeout = 1.0) or (temperatures < self.motorDriver.MAX_WORKING_TEMPERATURE).all():
                break
        if killEvent.is_set():
            return

        # Update last positions and enable torques in motors.
        with self.statesObjectsLocker:
            xA = self.xA
        self.motorsVelocityController.updateLastLegsPositions(xA)
        self.motorDriver.enableLegs()
    
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
            if self.safetyKillEvent.wait(timeout = self.motorsVelocityController.period):
                break
        
        self.motorsVelocityController.stopForceMode()
    
    def __pinToPinMovement(self, leg, currentPinPosition, goalPinPosition):
        # Distribute forces among other legs.
        otherLegs = np.delete(spider.LEGS_IDS, leg)
        self.__distributeForces(otherLegs, config.FORCE_DISTRIBUTION_DURATION)

        print("OPEN GRIPPER ", leg)
        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        if self.safetyKillEvent.wait(timeout = 3.5):
            return False
        if leg in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            print("GRIPPER DID NOT OPEN.")
            return

        # Read spider's rpy after releasing the leg.
        rpy = self.pumpsBnoArduino.getRpy()
        pinToPinLocal, legOriginOrientationInGlobal = tf.getPinToPinVectorInLocal(leg, rpy, currentPinPosition, goalPinPosition)
        print(pinToPinLocal)
        input("ENTER")
        globalZDirectionInLegOrigin = np.dot(legOriginOrientationInGlobal, np.array([0.0, 0.0, 1.0], dtype = np.float32))

        with self.statesObjectsLocker:
            xALeg = self.xA[leg]
        # Move leg and update spider state.
        self.motorsVelocityController.moveLegAsync(leg, xALeg, pinToPinLocal, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, isOffset = True)
        updateDictThread = self.threadManager.run(self.jsonFileManager.updatePins, config.UPDATE_DICT_THREAD_NAME, True, True, False, False, (leg, goalPinPosition, ))
        if self.safetyKillEvent.wait(timeout = 4.0):
            return False
        updateDictThread.join()

        # Before closing the gripper, put leg in force mode to avoid pulling the spider with gripper.
        self.motorsVelocityController.startForceMode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
        if self.safetyKillEvent.wait(timeout = 3.0):
            self.motorsVelocityController.stopForceMode()
            return False
        self.motorsVelocityController.stopForceMode()

        # Correction in case of missed pin.
        if leg not in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            correctionSuccess = self.__correction(leg, globalZDirectionInLegOrigin)
            print("CORRECTION SUCCESS: ", correctionSuccess)
            if not correctionSuccess:
                return False
        
        return True
    
    def __correction(self, legId, globalZDirectionInLocal):
        print(f"LEG {legId} IS NOT ATTACHED")
        autoSuccess = self.__automaticCorrection(legId, globalZDirectionInLocal)
        print("AUTO CORRECTION SUCCESS: ", autoSuccess)
        if autoSuccess:
            return True
        manualSuccess = self.__manualCorrection(legId)
        print("MANUAL CORRECTION SUCCESS: ", manualSuccess)
        return manualSuccess

    def __automaticCorrection(self, legId, globalZDirectionInLocal):
        detachOffsetZ = 0.03
        detachOffsetInLocal = globalZDirectionInLocal * detachOffsetZ
        numberOfTries = 0
        while numberOfTries < 2:
            self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
            if self.safetyKillEvent.wait(timeout = 1.0):
                return False      

            with self.statesObjectsLocker:
                xALeg = self.xA[legId]

            self.motorsVelocityController.moveLegAsync(legId, xALeg, detachOffsetInLocal, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY, isOffset = True)
            if self.safetyKillEvent.wait(timeout = 1.5):
                return False
            
            self.motorsVelocityController.startImpedanceMode(legId, -globalZDirectionInLocal)
            if self.safetyKillEvent.wait(timeout = 3.0):
                return False
            self.motorsVelocityController.stopImpedanceMode()

            self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
            if self.safetyKillEvent.wait(timeout = 1.0):
                return False 

            if legId in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
                return True 
        
            numberOfTries += 1

        return False

    def __manualCorrection(self, legId, useSafety = True):
        _, usedPinsIndexes, legsGlobalPositions = self.jsonFileManager.readSpiderState()
        goalPin = usedPinsIndexes[legId]

        print(f"MANUALLY CORRECT LEG {legId} ON PIN {goalPin}")
        self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        with self.statesObjectsLocker:
            xALeg = self.xA[legId]
            qA = self.qA

        attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()

        goalPinInLocal = kin.getGoalPinInLocal(legId, attachedLegs, legsGlobalPositions, qA, self.pumpsBnoArduino.getRpy())

        distance = np.linalg.norm(goalPinInLocal - xALeg)
        disableLegs = not (spider.LEG_LENGTH_MIN_LIMIT < np.linalg.norm(goalPinInLocal) < spider.LEG_LENGTH_MAX_LIMIT)
        if disableLegs:
            self.motorDriver.disableLegs(attachedLegs)

        self.motorsVelocityController.startForceMode([legId], [np.zeros(3, dtype = np.float32)])
        while True:
            with self.statesObjectsLocker:
                xALeg = self.xA[legId]
                if disableLegs:
                    qA = self.qA
            
            if disableLegs:
                goalPinInLocal = kin.getGoalPinInLocal(legId, attachedLegs, legsGlobalPositions, qA, self.pumpsBnoArduino.getRpy())

            distance = np.linalg.norm(goalPinInLocal - xALeg)
            switchState = int(self.motorsVelocityController.grippersArduino.getSwitchesStates()[legId])
            if (switchState == 0) and (distance < 0.1):
                self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
                time.sleep(1)
                break   

            if useSafety:
                if self.safetyKillEvent.wait(timeout = 0.1):
                    self.motorsVelocityController.stopForceMode()
                    return False
            else:
                time.sleep(0.1)
        self.motorsVelocityController.stopForceMode()

        return True    

    def __watering(self, wateringLegId, plantPosition, spiderPose, doRefill): 
        self.__distributeForces(np.delete(spider.LEGS_IDS, wateringLegId), config.FORCE_DISTRIBUTION_DURATION)
        with self.statesObjectsLocker:
            xALegBeforeWatering = self.xA[wateringLegId]

        # Move leg on watering position.
        _, _, legsGlobalPositions = self.jsonFileManager.readSpiderState()
        with self.statesObjectsLocker:
            qA = self.qA
        otherLegs = list(np.delete(spider.LEGS_IDS, wateringLegId))
        spiderPose = kin.getSpiderPose(otherLegs, legsGlobalPositions[otherLegs], qA)
        self.motorsVelocityController.grippersArduino.moveGripper(wateringLegId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        if self.safetyKillEvent.wait(timeout = 1.0):
            return False
        self.motorsVelocityController.moveLegAsync(wateringLegId, xALegBeforeWatering, plantPosition, config.GLOBAL_ORIGIN, 3, config.BEZIER_TRAJECTORY, spiderPose)
        if self.safetyKillEvent.wait(timeout = 3.5):
            return False
        
        # Turn on water pump for 3 seconds.
        pumpId = 1 if wateringLegId == 1 else 0
        if wateringLegId == 1:
            pumpId = 1
        elif wateringLegId == 4:
            pumpId = 0
        else:
            pumpId = 2
        pumpTime = config.NUMBER_OF_WATERING_BEFORE_REFILL * config.WATERING_TIME if doRefill else config.WATERING_TIME

        print("PUMP ON")
        startTime = time.perf_counter()
        while True:
            elapsedTime = time.perf_counter() - startTime
            self.pumpsBnoArduino.pumpControll(self.pumpsBnoArduino.PUMP_ON_COMMAND, pumpId)
            if elapsedTime > pumpTime:
                self.pumpsBnoArduino.pumpControll(self.pumpsBnoArduino.PUMP_OFF_COMMAND, pumpId)
                print("PUMP OFF")
                break
            if self.safetyKillEvent.wait(timeout = 0.05):
                return False
        
        # Move leg back on starting position.
        with self.statesObjectsLocker:
            xALegAfterWatering = self.xA[wateringLegId]

        self.motorsVelocityController.moveLegAsync(wateringLegId, xALegAfterWatering, xALegBeforeWatering, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY)
        if self.safetyKillEvent.wait(timeout = 3.5):
            return False
        self.motorsVelocityController.grippersArduino.moveGripper(wateringLegId, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
        if self.safetyKillEvent.wait(1.0):
            return False
        # Correct if leg does not reach starting pin successfully.
        if wateringLegId not in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            rpy = self.pumpsBnoArduino.getRpy()
            spiderRotationInGlobal = tf.xyzRpyToMatrix(rpy, True)
            legOriginOrientationInGlobal = np.linalg.inv(np.dot(spiderRotationInGlobal, spider.T_ANCHORS[wateringLegId][:3, :3]))
            globalZDirectionInLegOrigin = np.dot(legOriginOrientationInGlobal, np.array([0.0, 0.0, 1.0], dtype = np.float32))

            return self.__correction(wateringLegId, globalZDirectionInLegOrigin)
        
        return True
    #endregion

if __name__ == '__main__':
    app = App()
    time.sleep(1)
    app.spiderStatesManager(config.WORKING_STATE)