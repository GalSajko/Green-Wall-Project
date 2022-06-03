import numpy as np
import time
import serial
import threading

import calculations 
import environment as env
import dynamixel as dmx
import planning


class VelocityController:
    """ Class for velocity-control of spider's movement.
    """
    def __init__ (self):
        self.matrixCalculator = calculations.MatrixCalculator()
        self.kinematics = calculations.Kinematics()
        self.geometryTools = calculations.GeometryTools()
        self.spider = env.Spider()
        self.trajectoryPlanner = planning.TrajectoryPlanner()
        self.pathPlanner = planning.PathPlanner(0.05, 0.1, 'squared')
        motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
        self.motorDriver = dmx.MotorDriver(motors)
        self.gripperController = GripperController()
    
    def getQdQddLegFF(self, legIdx, xD, xDd):
        """Feed forward calculations of reference joints positions and velocities for single leg movement.

        :param legIdx: Leg id.
        :param xD: Leg tip reference trajectory.
        :param xDd: Leg tip reference velocities.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = []
        qDd = []
        for idx, pose in enumerate(xD):
            currentQd = self.kinematics.legInverseKinematics(legIdx, pose[:3])
            J = self.kinematics.legJacobi(legIdx, currentQd)
            currentQdd = np.dot(np.linalg.inv(J), xDd[idx][:3])
            qD.append(currentQd)
            qDd.append(currentQdd)
        
        return np.array(qD), np.array(qDd)
    
    def getQdQddPlatformFF(self, xD, xDd, globalLegsPositions):
        """Feed forward calculations of reference joints positions and velocities for parallel platform movement. 

        :param xD: Platform referenece positions.
        :param xDd: Platform reference velocities.
        :param globalLegsPositions: Legs positions during parallel movement in global origin.
        :return: Matrices of reference joints positions and velocities.
        """
        qD = []
        qDd = []
        for idx, pose in enumerate(xD):
            currentQd = self.kinematics.platformInverseKinematics(pose[:-1], globalLegsPositions)
            referenceLegsVelocities = self.kinematics.getSpiderToLegReferenceVelocities(xDd[idx])
            currentQdd = []
            for leg in range(self.spider.NUMBER_OF_LEGS):
                J = self.kinematics.legJacobi(leg, currentQd[leg])
                currentQdd.append(np.dot(np.linalg.inv(J), referenceLegsVelocities[leg]))
            qD.append(currentQd)
            qDd.append(currentQdd)
        
        return np.array(qD), np.array(qDd)

    def moveLegs(self, legsIds, trajectories, velocities, grippersCommands = None):
        """Move any number of legs (within number of spiders legs) along given trajectories.

        :param ledIds: Array of legs ids.
        :param trajectory: 2D array of legs tips trajectories.
        :param velocity: 2D array of legs tips velocities.
        :return: True if movements were successfull, false otherwise.
        """
        if len(legsIds) != len(trajectories) and len(legsIds) != len(velocities):
            raise ValueError("Invalid parameters!")

        qDs = []
        qDds = []
        # Time steps of all trajectories should be the same (it is defined in trajectory planner).
        timeStep = trajectories[0][:,-1][1] - trajectories[0][:,-1][0]
        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        if not self.motorDriver.addGroupSyncReadParams(legsIds):
            return False
        for idx, leg in enumerate(legsIds):
            qD, qDd = self.getQdQddLegFF(leg, trajectories[idx], velocities[idx])
            qDs.append(qD)
            qDds.append(qDd)

        # Index of longer trajectory:
        longerIdx = trajectories.index(max(trajectories, key = len))
        lastErrors = np.zeros([len(legsIds), 3])
        Kp = 10
        Kd = 1

        # Open grippers (if needed) and wait for them to open before moving legs.
        if grippersCommands == ['o'] * len(legsIds):
            self.gripperController.openGrippersAndWait(legsIds)

        # Use indexes of longest trajectory.
        for i, _ in enumerate(trajectories[longerIdx]):
            startTime = time.time()
            # Read motors positions in all legs.
            qA = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds)
            qCds = []
            for l, leg in enumerate(legsIds):
                # If index is still whithin boundaries of current trajectory.
                if i < len(trajectories[l]) - 1:
                    currentQd = qDs[l][i]
                    currentQdd = qDds[l][i]
                    currentQa = qA[l]
                    error = currentQd - currentQa
                    dE = (error - lastErrors[l]) / timeStep
                    qCd = Kp * error + Kd * dE + currentQdd
                    lastErrors[l] = error
                # If not, stop the leg.
                elif i >= len(trajectories[l]) - 1:
                    qCd = [0, 0, 0]
                    if grippersCommands is not None and grippersCommands[l] == self.gripperController.CLOSE_COMMAND: 
                        self.gripperController.moveGripper(leg, self.gripperController.CLOSE_COMMAND)
                qCds.append(qCd)
            if not self.motorDriver.syncWriteMotorsVelocitiesInLegs(legsIds, qCds, i == 0):
                return False
            
            timeLeft = timeStep - (time.time() - startTime)
            sleepTime = timeLeft if timeLeft > 0 else 0
            time.sleep(sleepTime)
   

        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        return True

    def moveLegsWrapper(self, legsIds, globalGoalPositions, spiderPose, durations, gripperCommands = None, readLegs = True, globalStartPositions = None, trajectoryType = "bezier"):
        """Wrapper function for moving any number of legs (within number of spiders legs) to desired pins on the wall. 
        Includes transformation from global to legs-local pins positions and computing trajectories.

        :param legsIds: Array of legs ids, if value is 5 than all legs are selected.
        :param globalGoalPositions: Array of global goal positions (represents desired pins positions).
        :param spiderPose: Array of xyzrpy global position of spider's body.
        :param durations: Array of durations for legs movements in seconds.
        :param readLegs: If true, read local leg position on the beginning of each leg movement. Otherwise take FF calculations of global legs positions.
        :param globalStartPositions: Array of FF calculations of starting global legs positions.
        :param trajectoryType: 'bezier' for bezier curve or 'minJerk' for minimal jerk trajectory.
        :raises ValueError: Exception is thrown if legsIds, globalGoalPositions and durations parameters dont have same length.
        :return: True if movements were successfull, false otherwise.
        """
        if legsIds == 5:
            legsIds = [0, 1, 2, 3, 4]
        if len(legsIds) != len(globalGoalPositions) or len(legsIds) != len(durations):
            raise ValueError("Invalid values of legsIds, goalGlobalPositions or durations parameters.")
        if not readLegs and not np.array(globalStartPositions).any():
            raise ValueError("Legs starting positions are not given!")
        if not readLegs and (len(globalGoalPositions) != len(globalStartPositions)):
            raise ValueError("Invalid values of globalStartPositions parameter.")

        # Transformation matrix of global spider's pose.
        T_GS = self.matrixCalculator.xyzRpyToMatrix(spiderPose)
        # Calculate trajectories for each leg movement from local positions.
        trajectories = []
        velocities = []
        for legIdx, leg in enumerate(legsIds):
            # Transform global positions into local.
            T_GA = np.dot(T_GS, self.spider.T_ANCHORS[leg])
            if readLegs:
                # startPosition = self.motorDriver.syncReadMotorsPositionsInLegs([leg], True)[0]
                startPosition = self.motorDriver.readLegPosition(leg)
            else:
                legGlobalStartPosition = np.append(globalStartPositions[legIdx], 1)
                startPosition = np.dot(np.linalg.inv(T_GA), legGlobalStartPosition)[:3]
            legGlobalGoalPosition = np.append(globalGoalPositions[legIdx], 1)
            localGoalPosition = np.dot(np.linalg.inv(T_GA), legGlobalGoalPosition)[:3]
            # Calculate trajectory and velocity.
            if trajectoryType == 'bezier':
                traj, vel = self.trajectoryPlanner.bezierTrajectory(startPosition, localGoalPosition, durations[legIdx])
            elif trajectoryType == 'minJerk':
                traj, vel = self.trajectoryPlanner.minJerkTrajectory(startPosition, localGoalPosition, durations[legIdx])
            else:
                print("Invalid trajectory type.")
                return False
            trajectories.append(traj)
            velocities.append(vel)
        
        result = self.moveLegs(legsIds, trajectories, velocities, gripperCommands)

        return result
    
    def moveLegsAndGrabPins(self, legsIds, globalGoalPositions, spiderPose, durations, readLegs = True, globalStartPositions = None):
        """Open grippers, detach legs from pin, move them on approach positions above goal pins and than lower them on pins. Finally, close the grippers.

        :param legsIds: Legs ids.
        :param globalGoalPositions: Global goal positions of legs (pins).
        :param spiderPose: Global pose of spider during legs movements.
        :param durations: Array of durations for each leg movement.
        :param readLegs: If true, read starting legs positions before movements, otherwise use FF calculations, defaults to True
        :param globalStartPositions: Global start positions of legs, defaults to None.
        :param trajectoryType: Trajectory type for first movement (above the pin). Second movement is always along minJerk trajectory.
        :return: True if movements were successfull, false otherwise.
        """
        if legsIds == 5:
            legsIds = [0, 1, 2, 3, 4]
        approachTime = 1.5
        detachTime = 1
        durations = np.array(durations) - approachTime - detachTime
        approachPoints = self.matrixCalculator.getLegsApproachPositionsInGlobal(legsIds, spiderPose, globalGoalPositions)
        if globalStartPositions:
            detachPoints = np.copy(globalStartPositions)
        else:
            detachPoints = []
            # localLegsPositions = self.motorDriver.syncReadMotorsPositionsInLegs([legsIds], True)
            localLegsPositions = [self.motorDriver.readLegPosition(leg) for leg in legsIds]
            detachPoints = self.matrixCalculator.getLegsInGlobal(legsIds, localLegsPositions, spiderPose)
        detachPoints[:, 2] += 0.02
 
        if not self.moveLegsWrapper(legsIds, detachPoints, spiderPose, np.ones(len(legsIds)) * detachTime, [self.gripperController.OPEN_COMMAND] * len(legsIds), readLegs, globalStartPositions, 'minJerk'):
            print("Legs movement error!")
            return False
        if not self.moveLegsWrapper(legsIds, approachPoints, spiderPose, durations, readLegs = True, globalStartPositions = detachPoints):
            print("Legs movement error!")
            return False
        if not self.moveLegsWrapper(legsIds, globalGoalPositions, spiderPose, np.ones(len(legsIds)) * approachTime, [self.gripperController.CLOSE_COMMAND] * len(legsIds), True, approachPoints, 'minJerk'):
            print("Legs movement error!")
            return False

        return True

    def movePlatform(self, trajectory, velocity, globalLegsPositions):
        """Move spider body as a parallel platform along a given trajectory.

        :param trajectory: Spider's body trajectory.
        :param velocity: Spider's body velocity.
        :param globalStartPose: Global pose of spider at the beginning of the platform movement.
        :param globalLegsPositions: Global positions of legs during platform movement.
        :return: True if movement was successfull, false otherwise.
        """

        legsIds = [leg for leg in range(self.spider.NUMBER_OF_LEGS)]
        qDs, qDds = self.getQdQddPlatformFF(trajectory, velocity, globalLegsPositions)

        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        if not self.motorDriver.addGroupSyncReadParams(legsIds):
            return False

        lastErrors = np.zeros([len(legsIds), 3])
        Kp = 10
        Kd = 1
        timeStep = trajectory[1][-1] - trajectory[0][-1]

        for idx, qD in enumerate(qDs):
            startTime = time.time()
            qA = self.motorDriver.syncReadMotorsPositionsInLegs(legsIds)
            errors = np.array(qD - qA, dtype = object)
            dE = (errors - lastErrors) / timeStep
            qCds = Kp * errors + Kd * dE + qDds[idx]
            lastErrors = errors

            if idx == len(trajectory) - 1:
                qCds = np.zeros([len(legsIds), 3])

            if not self.motorDriver.syncWriteMotorsVelocitiesInLegs(legsIds, qCds, idx == 0):
                return False

            try:
                time.sleep(timeStep - (time.time() - startTime))
            except:
                time.sleep(0)

        self.motorDriver.clearGroupSyncReadParams()
        self.motorDriver.clearGroupSyncWriteParams()
        return True

    def movePlatformWrapper(self, globalStartPose, globalGoalPose, globalLegsPositions, duration):
        """Wrapper function for moving a platform. Includes trajectory calculations.

        :param globalStartPose: Starting pose in global origin.
        :param globalGoalPose: Goal pose in global origin.
        :param globalLegsPositions: Global positions of legs during platform movement.
        :param duration: Desired duration of movement.
        :return: True if movement was successfull, false otherwise.
        """
        traj, vel = self.trajectoryPlanner.minJerkTrajectory(globalStartPose, globalGoalPose, duration)
        result = self.movePlatform(traj, vel, globalLegsPositions)

        return result

    def walk(self, globalStartPose, globalGoalPose):
        """Walking procedure for spider to walk from start to goal point on the wall.

        :param globalStartPose: Spider's starting pose on the wall.
        :param globalGoalPose: Spider's goal pose on the wall.
        :return: True if walking procedure was successfull, false otherwise.
        """
        platformPoses, pins = self.pathPlanner.calculateWalkingMovesFF(globalStartPose, globalGoalPose)

        # Move legs on starting positions, based on calculations for starting spider's position. 
        result = self.moveLegsAndGrabPins(5, pins[0], globalStartPose, np.ones(5) * 6)
        if not result:
            print("Legs movement error!")
            return False

        # Move through calculated poses.
        for poseIdx, pose in enumerate(platformPoses):
            if poseIdx == 0:
                continue

            # Move platform.
            linDist = np.linalg.norm(platformPoses[poseIdx - 1][:3] - pose[:3])
            if linDist != 0:
                parallelMovementDuration = 2.5 * linDist / 0.05
            else:
                rotDist = abs(platformPoses[poseIdx - 1][3] - pose[3])
                parallelMovementDuration = 2 * rotDist / 0.1
 
            result = self.movePlatformWrapper(platformPoses[poseIdx - 1], pose, pins[poseIdx - 1], parallelMovementDuration)
            if not result:
                print("Platform movement error!")
                return False
                
            # Select indexes of legs which have to move.
            legsToMoveIdxs = np.array(np.where(np.any(pins[poseIdx] - pins[poseIdx - 1] != 0, axis = 1))).flatten()
            # Move legs and grab pins.
            for legIdx in legsToMoveIdxs:
                result = self.moveLegsAndGrabPins([legIdx], [pins[poseIdx][legIdx]], pose, np.ones(1) * 4, False, [pins[poseIdx - 1][legIdx]])
                if not result:
                    print("Legs movement error!")
                    return False

        return True

class GripperController:
    """Class for controlling a gripper via serial communication with Arduino.
    """
    def __init__(self):
        """Init serial communication with Arduino.
        """
        self.OPEN_COMMAND = "o"
        self.CLOSE_COMMAND = "c"
        self.INIT_MESSAGE = "init"
        self.GRIPPER_OPENED_RESPONSE = "1"
        self.GRIPPER_CLOSED_RESPONSE = "0"
        self.INIT_RESPONSE = "OK"

        self.receivedMessage = ""

        self.comm = serial.Serial('/dev/ttyUSB0', 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.lock = threading.Lock()

        readingThread = threading.Thread(target = self.readData, name = "serial_reading_thread", daemon = True)
        readingThread.start()

        self.handshake()
    
    def readData(self):
        """Constantly receiving data from Arduino.
        """
        while True:
            time.sleep(0.001)
            with self.lock:
                msg = self.comm.readline()
            while not '\\n' in str(msg):
                time.sleep(0.001)
                with self.lock:
                    msg += self.comm.readline()

            self.receivedMessage = msg.decode("utf-8", errors = "ignore").rstrip()

    def sendData(self, msg):
        """Send data to Arduino.

        :param msg: Message to send.
        """ 
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        with self.lock:
            self.comm.write(msg)
    
    def moveGripper(self, legId, command):
        """Send command to move gripper on selected leg.

        :param legId: Leg id.
        :param command: Command to execute on gripper.
        """
        if (command == self.OPEN_COMMAND or command == self.CLOSE_COMMAND):
            msg = command + str(legId) + "\n"
            self.sendData(msg)
    
    def openGrippersAndWait(self, legsIds):
        """Open grippers on given legs and wait for them to come in open state.

        :param legsIds: Legs ids.
        :return: True when all grippers are opened.
        """
        for leg in legsIds:
            self.moveGripper(leg, self.OPEN_COMMAND)
        checkArray = np.array([False] * len(legsIds))
        while not checkArray.all():
            with self.lock:
                recMsg = self.receivedMessage
            if len(recMsg) == 5:
                for id, leg in enumerate(legsIds):
                    if recMsg[leg] == self.GRIPPER_OPENED_RESPONSE:
                        checkArray[id] = True
        return True

    def handshake(self):
        """Handshake procedure with Arduino.
        """
        self.sendData(self.INIT_MESSAGE)
        time.sleep(0.01)
        while not self.receivedMessage == self.INIT_RESPONSE:
            time.sleep(0.01)
            self.sendData(self.INIT_MESSAGE)
        print("Handshake with Arduino successfull.")




