import numpy as np
import threading
import time
import os
import random

import config
import controllers
import threadmanager
import jsonfilemanager
from periphery import dynamixel as dmx
from periphery import waterpumpsbno
from environment.comunication import CommunicationWithServer
from environment import spider
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools
from calculations import transformations as tf
from planning import pathplanner

from environment import wall

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
        self.comunicationWithServer = CommunicationWithServer(self.jsonFileManager.FILENAME)

        self.statesObjectsLocker = threading.Lock()
        self.safetyKillEvent = threading.Event()

        self.currentState = None
        self.wateringCounter = 0

        self.wasInRestingState = False
        self.lastPlantOrRefillPosition = None

        self.initBno = True

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
                    
                    self.wasInRestingState = True

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

    def spiderStatesManager(self, state):
        """Managing spider's state.

        Args:
            state (string): Name of desired state.
            workingArs (tuple, optional): Neede parameters for desired state. Defaults to None.
        """
        if state == config.WORKING_STATE:
            self.spiderStateThread = self.threadManager.run(self.working, config.WORKING_THREAD_NAME, True, True, False, False)
        elif state == config.RESTING_STATE:
            self.spiderStateThread, self.spiderStatesThreadKillEvenet = self.threadManager.run(self.rest, config.RESTING_THREAD_NAME, True, True, True, False)
        elif state == config.TRANSITION_STATE:
            self.spiderStateThread = self.threadManager.run(self.transitionToRestState, config.TRANSITION_THREAD_NAME, True, True, False, False)
    
    def working(self):
        """Working procedure, includes walking and watering the plants.
        """
        self.currentState = config.WORKING_STATE
        isInit = True
        print("WORKING...") 
        
        while True:
            spiderPose, _, startLegsPositions = self.jsonFileManager.readSpiderState()
            doRefillWaterTank = self.wateringCounter >= config.NUMBER_OF_WATERING_BEFORE_REFILL
            while True:
                try:
                    if doRefillWaterTank:
                        wateringLegId, endPose = tf.getWateringLegAndPose(spiderPose, doRefill = True)
                        plantOrRefillPosition = endPose[:3] + spider.REFILLING_LEG_OFFSET
                        print("GOING TO REFILL POSITION.")
                    else:
                        if not self.wasInRestingState:
                            plantOrRefillPosition = self.comunicationWithServer.getGoalPos()
                            self.lastPlantOrRefillPosition = plantOrRefillPosition
                        else:
                            plantOrRefillPosition = self.lastPlantOrRefillPosition
                        print(f"PLANT POSITION {plantOrRefillPosition}.")
                        wateringLegId, endPose = tf.getWateringLegAndPose(spiderPose, plantOrRefillPosition)
                    if isInit:
                        pathPlannerReturnValue = pathplanner.modifiedWalkingInstructions(startLegsPositions, endPose)
                        if not pathPlannerReturnValue:
                            os._exit(0)
                        else:
                            poses, pinsInstructions = pathPlannerReturnValue
                        isInit = False
                        break
                        
                    poses, pinsInstructions = pathplanner.createWalkingInstructions(spiderPose, endPose)
                    break
                except Exception as e:
                    print("Error in path planning. Trying again... ")
                    print(f"EXCEPTION {e}")
                time.sleep(0.05)

            print(f"NEW GOAL POINT {endPose[:3]}.")
            for step, pose in enumerate(poses):
                currentPinsPositions = pinsInstructions[step, :, 1:]
                currentLegsMovingOrder = pinsInstructions[step, :, 0].astype(int)
                with self.statesObjectsLocker:
                    xA = self.xA

                if step == 0:
                    self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, currentPinsPositions, config.GLOBAL_ORIGIN, 5, config.MINJERK_TRAJECTORY, pose)
                    self.jsonFileManager.updateWholeDict(pose, currentPinsPositions, currentLegsMovingOrder)
                    if self.safetyKillEvent.wait(timeout = 5.5):
                        return

                    if self.initBno:
                        self.pumpsBnoArduino.resetBno()
                        self.initBno = False
                        if self.safetyKillEvent.wait(timeout = 1.0):
                            return
                    self.__distributeForces(spider.LEGS_IDS, config.FORCE_DISTRIBUTION_DURATION)
                    continue

                previousPinsPositions = np.array(pinsInstructions[step - 1, :, 1:])
                self.motorsVelocityController.moveLegsSync(currentLegsMovingOrder, xA, previousPinsPositions, config.GLOBAL_ORIGIN, 2.5, config.MINJERK_TRAJECTORY, pose)
                self.jsonFileManager.updateWholeDict(pose, previousPinsPositions, currentLegsMovingOrder)
                if self.safetyKillEvent.wait(timeout = 3.0):
                    print("UNSUCCESSFULL BODY MOVEMENT.")
                    return

                pinsOffsets = currentPinsPositions - previousPinsPositions
                for idx, leg in enumerate(currentLegsMovingOrder):
                    if pinsOffsets[idx].any():
                        movementSuccess = self.__pinToPinMovement(leg, previousPinsPositions[idx], currentPinsPositions[idx], pose)
                        if not movementSuccess:
                            print("UNSUCCESSFULL PIN TO PIN MOVEMENT.")
                            return
            
            wateringSuccess = self.__watering(wateringLegId, plantOrRefillPosition, pose, doRefillWaterTank)
            if not wateringSuccess:
                return
        
    def rest(self, killEvent):
        """Resting procedure, includes option for manually correcting non-attached leg. Resting lasts until temperatures of all motors
        drop below working temperature.

        Args:
            killEvent (event): Event for killing a procedure.
        """
        print("RESTING...")
        self.currentState = config.RESTING_STATE

        with self.statesObjectsLocker:
            hwErrors = self.hwErrors

        attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()
        motorsInError = self.motorDriver.motorsIds[np.where(hwErrors != 0)]
        print("REBOOTING MOTORS WITH IDS: ", motorsInError, "...")
        self.motorDriver.rebootMotors(motorsInError)
        while hwErrors.any():
            with self.statesObjectsLocker:
                hwErrors = self.hwErrors
            if killEvent.wait(timeout = 0.1):
                return

        unattachedLeg = np.setdiff1d(spider.LEGS_IDS, attachedLegs)
        while unattachedLeg.size > 1:
            print("MORE THAN ONE UNATTACHED LEG: ", unattachedLeg)
            unattachedLeg = np.setdiff1d(spider.LEGS_IDS, self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs())
            if killEvent.wait(timeout = 0.1):
                return

        if unattachedLeg.size == 1:
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
        
        # Check for possible singularity of lower legs.
        _, _, legsGlobalPositions = self.jsonFileManager.readSpiderState()
        if np.linalg.norm(legsGlobalPositions[2] - legsGlobalPositions[3]) <= 0.22:
            print("CAUTION! POSSIBLE SINGULARITY LOCK - CHECK BEFORE CONTINUING!!!")
            self.motorDriver.disableLegs()
            confirmInput = input("PRESS ENTER TO CONTINUE OR K + ENTER TO KILL A PROGRAM.")
            if confirmInput == config.PROGRAM_KILL_KEY:
                print("KILLING A PROGRAM...")
                os._exit(0)

        # Update last positions and enable torques in motors.
        with self.statesObjectsLocker:
            xA = self.xA
        self.motorsVelocityController.updateLastLegsPositions(xA)
        if killEvent.wait(timeout = 1.0):
            return
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
            if self.safetyKillEvent.wait(timeout = self.motorsVelocityController.period):
                break
        
        self.motorsVelocityController.stopForceMode()
        time.sleep(2.0)
    
    def __pinToPinMovement(self, leg, currentPinPosition, goalPinPosition, pose):
        # Distribute forces among other legs.
        otherLegs = np.delete(spider.LEGS_IDS, leg)
        self.__distributeForces(otherLegs, config.FORCE_DISTRIBUTION_DURATION)

        with self.statesObjectsLocker:
            qALeg = self.qA[leg]
        lastJointPositionInLocal = kin.legBaseToThirdJointForwardKinematics(qALeg)[:,3][:3]
        lastJointToGoalPinInSpiderUnit = tf.getLastJointToGoalPinVectorInSpider(leg, lastJointPositionInLocal, goalPinPosition, pose)
        
        self.motorsVelocityController.startForceMode([leg], [lastJointToGoalPinInSpiderUnit * 2.5])
        if self.safetyKillEvent.wait(timeout = 1.0):
            self.motorsVelocityController.stopForceMode()
            return False

        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        if self.safetyKillEvent.wait(timeout = 3.5):
            self.motorsVelocityController.stopForceMode()
            return False
        if leg in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            print(f"GRIPPER {leg} DID NOT OPEN.")
            self.motorsVelocityController.stopForceMode()
            return

        # Read spider's rpy after releasing the leg.
        _, _, legsGlobalPositions = self.jsonFileManager.readSpiderState()
        with self.statesObjectsLocker:
            qA = self.qA
        spiderPose = kin.getSpiderPose(otherLegs, legsGlobalPositions[otherLegs], qA)
        rpy = spiderPose[3:]

        pinToPinLocal, legOriginOrientationInGlobal = tf.getPinToPinVectorInLocal(leg, rpy, currentPinPosition, goalPinPosition)
        globalZDirectionInLegOrigin = np.dot(legOriginOrientationInGlobal, np.array([0.0, 0.0, 1.0], dtype = np.float32))

        self.motorsVelocityController.stopForceMode()
        if self.safetyKillEvent.wait(timeout = 1.0):
            return False
        
        # Move leg and update spider state.
        with self.statesObjectsLocker:
            xALeg = self.xA[leg]
        localGoalPosition = self.motorsVelocityController.moveLegAsync(leg, xALeg, pinToPinLocal, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY, isOffset = True)
        self.jsonFileManager.updatePins(leg, goalPinPosition)
        if self.safetyKillEvent.wait(timeout = 4.0):
            return False

        # Before closing the gripper, put leg in force mode to avoid pulling the spider with gripper.
        self.motorsVelocityController.startForceMode(np.array([leg]), np.array([np.zeros(3, dtype = np.float32)]))
        self.motorsVelocityController.grippersArduino.moveGripper(leg, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
        if self.safetyKillEvent.wait(timeout = 3.0):
            self.motorsVelocityController.stopForceMode()
            return False
        self.motorsVelocityController.stopForceMode()

        # Correction in case of missed pin.
        if leg not in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            correctionSuccess = self.__correction(leg, globalZDirectionInLegOrigin, localGoalPosition)
            print("CORRECTION SUCCESS: ", correctionSuccess)
            if not correctionSuccess:
                return False

        return True
    
    def __correction(self, legId, globalZDirectionInLocal, localGoalPosition):
        print(f"LEG {legId} IS NOT ATTACHED.")
        autoSuccess = self.__automaticCorrection(legId, globalZDirectionInLocal, localGoalPosition)
        print("AUTO CORRECTION SUCCESS: ", autoSuccess)
        if autoSuccess:
            return True
        manualSuccess = self.__manualCorrection(legId)
        print("MANUAL CORRECTION SUCCESS: ", manualSuccess)
        return manualSuccess

    def __automaticCorrection(self, legId, globalZDirectionInLocal, localGoalPosition):
        detachOffsetZ = 0.08
        offsetValue = 0.15
        offsets = [
            [0.0, 0.0, 0.0],
            [0.0, offsetValue, 0.0],
            [0.0, -offsetValue, 0.0],
            [offsetValue, 0.0, 0.0],
            [-offsetValue, 0.0, 0.0],
            [offsetValue, offsetValue, 0.0],
            [offsetValue, -offsetValue, 0.0],
            [-offsetValue, -offsetValue, 0.0],
            [-offsetValue, offsetValue, 0.0]
        ]
        
        detachPosition = np.copy(localGoalPosition)
        detachPosition[2] += detachOffsetZ
        for offset in offsets:
            self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
            if self.safetyKillEvent.wait(timeout = 1.5):
                return False
 
            with self.statesObjectsLocker:
                xALeg = self.xA[legId]
            self.motorsVelocityController.moveLegAsync(legId, xALeg, detachPosition, config.LEG_ORIGIN, 1, config.MINJERK_TRAJECTORY)
            if self.safetyKillEvent.wait(timeout = 1.5):
                return False
            
            velocityDirection = -(globalZDirectionInLocal + offset)
            self.motorsVelocityController.startImpedanceMode(legId, velocityDirection)
            if self.safetyKillEvent.wait(timeout = 2.0):
                self.motorsVelocityController.stopImpedanceMode()
                return False
            self.motorsVelocityController.stopImpedanceMode()      

            self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
            if self.safetyKillEvent.wait(timeout = 2.0):
                return False 

            if legId in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
                return True 

        return False

    def __manualCorrection(self, legId, useSafety = True):
        _, usedPinsIndexes, legsGlobalPositions = self.jsonFileManager.readSpiderState()
        goalPin = usedPinsIndexes[legId]

        print(f"MANUALLY CORRECT LEG {legId} ON PIN {goalPin}.")
        self.motorsVelocityController.grippersArduino.moveGripper(legId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        with self.statesObjectsLocker:
            xALeg = self.xA[legId]
            qA = self.qA

        attachedLegs = self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs()

        print("ATTACHED LEGS: ", attachedLegs)
        goalPinInLocal = kin.getGoalPinInLocal(legId, attachedLegs, legsGlobalPositions, qA, self.pumpsBnoArduino.getRpy())

        distance = np.linalg.norm(goalPinInLocal - xALeg)
        disableLegs = not (spider.LEG_LENGTH_MIN_LIMIT < np.linalg.norm(goalPinInLocal) < spider.LEG_LENGTH_MAX_LIMIT)
        if disableLegs:
            print(f"DISABLE LEGS: {attachedLegs}.")
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
                time.sleep(2)
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
        spiderPose = kin.getSpiderPose(spider.LEGS_IDS, legsGlobalPositions, qA)

        self.motorsVelocityController.grippersArduino.moveGripper(wateringLegId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
        if self.safetyKillEvent.wait(timeout = 1.0):
            if not doRefill:
                return False

        self.motorsVelocityController.moveLegAsync(wateringLegId, xALegBeforeWatering, plantPosition, config.GLOBAL_ORIGIN, 3, config.BEZIER_TRAJECTORY, spiderPose)
        if self.safetyKillEvent.wait(timeout = 3.5):
            if not doRefill:
                return False
        
        # Close gripper to allow refilling.
        if doRefill:
          self.motorsVelocityController.grippersArduino.moveGripper(wateringLegId, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)  

        # Turn on water pump.
        if wateringLegId == 1:
            pumpId = 1
        elif wateringLegId == 4:
            pumpId = 0
        else:
            pumpId = 2

        pumpTime = config.REFILL_TIME if doRefill else config.WATERING_TIME

        print(f"PUMP {pumpId} ON.")
        startTime = time.perf_counter()
        while True:
            elapsedTime = time.perf_counter() - startTime
            self.pumpsBnoArduino.pumpControll(self.pumpsBnoArduino.PUMP_ON_COMMAND, pumpId)
            if elapsedTime > pumpTime:
                if not doRefill:
                    self.pumpsBnoArduino.pumpControll(self.pumpsBnoArduino.PUMP_OFF_COMMAND, pumpId)
                    print(f"PUMP {pumpId} OFF.")
                else:
                    self.motorsVelocityController.grippersArduino.moveGripper(wateringLegId, self.motorsVelocityController.grippersArduino.OPEN_COMMAND)
                break
            if not doRefill:
                if self.safetyKillEvent.wait(timeout = 0.05):
                    self.pumpsBnoArduino.pumpControll(self.pumpsBnoArduino.PUMP_OFF_COMMAND, pumpId)
                    print(f"PUMP {pumpId} OFF.")
                    return False
            else:
                time.sleep(0.05)
        
        # Move leg back on starting position.
        with self.statesObjectsLocker:
            xALegAfterWatering = self.xA[wateringLegId]

        self.motorsVelocityController.moveLegAsync(wateringLegId, xALegAfterWatering, xALegBeforeWatering, config.LEG_ORIGIN, 3, config.BEZIER_TRAJECTORY)
        if self.safetyKillEvent.wait(timeout = 3.5):
            if not doRefill:
                return False
            
        if doRefill:
            self.pumpsBnoArduino.pumpControll(self.pumpsBnoArduino.PUMP_OFF_COMMAND, pumpId)
            print(f"PUMP {pumpId} OFF.") 
            self.wateringCounter = 0
        else:
            self.wateringCounter += 1
            
        self.motorsVelocityController.grippersArduino.moveGripper(wateringLegId, self.motorsVelocityController.grippersArduino.CLOSE_COMMAND)
        if self.safetyKillEvent.wait(1.0):
            if not doRefill:
                return False
        
        # Correct if leg does not reach starting pin successfully.
        if wateringLegId not in self.motorsVelocityController.grippersArduino.getIdsOfAttachedLegs():
            rpy = self.pumpsBnoArduino.getRpy()
            spiderRotationInGlobal = tf.xyzRpyToMatrix(rpy, True)
            legOriginOrientationInGlobal = np.linalg.inv(np.dot(spiderRotationInGlobal, spider.T_ANCHORS[wateringLegId][:3, :3]))
            globalZDirectionInLegOrigin = np.dot(legOriginOrientationInGlobal, np.array([0.0, 0.0, 1.0], dtype = np.float32))

            return self.__correction(wateringLegId, globalZDirectionInLegOrigin, xALegBeforeWatering)

        return True
    #endregion

if __name__ == '__main__':
    app = App()
    time.sleep(1)
    app.spiderStatesManager(config.WORKING_STATE)
