"""
This module is meant as a testing sandbox for other modules, during implementation.
"""

from logging.handlers import RotatingFileHandler
import numpy as np
import time
import math
from threading import Thread
import sys
import matplotlib.pyplot as plt

import environment
import simulaton
import calculations
import mappers
import dynamixel
import planning

if __name__ == "__main__":
    spider = environment.Spider()
    trajectoryPlanner = planning.TrajectoryPlanner()
    kinematics = calculations.Kinematics()
    matrixCalculator = calculations.MatrixCalculator()
    motors = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
    legs = [0, 1, 2, 3, 4]
    startPose = [0, 0, 0.3, 0, 0, 0]
    goalPose = [0, 0, 0.35, 0, 0, 0.5]
    trajectory, velocity = trajectoryPlanner.minJerkTrajectory(startPose, goalPose, 5)
    timeVector = trajectory[:,-1]
    # Starting leg positions in leg-based origins.
    startLocalLegsPositions = np.array([
        [0.364, 0, -0.3],
        [0.364, 0, -0.3],
        [0.364, 0, -0.3],
        [0.364, 0, -0.3],
        [0.364, 0, -0.3],
    ])

    # Starting legs poisitons in global origin.
    startGlobalLegsPositions = matrixCalculator.getLegsInGlobal(spider.T_ANCHORS, startLocalLegsPositions, startPose)
    
    # Reference joints positions and velocities.
    qD = []
    qDd = []
    # Reference leg-tips velocities.
    xDd = []
    for idx, traj in enumerate(trajectory):
        # Extract data from trajectory and velocity.
        pose = traj[:-1]
        velocityGlobal = velocity[idx]
        linearVelocityGlobal = velocityGlobal[:3]
        velocityGlobalMatrix = matrixCalculator.xyzRpyToMatrix(velocityGlobal)

        # Calculate current qD from pose with platform IK.
        currentQd = kinematics.platformInverseKinematics(pose, startGlobalLegsPositions)

        # Rotate spiders reference velocity into anchors velocities which represent reference velocities for single leg.
        anchorsVelocities = [np.dot(np.linalg.inv(tAnchor[:3,:3]), linearVelocityGlobal) for tAnchor in spider.T_ANCHORS]

        # Add angular velocities.
        for i, anchorVelocity in enumerate(anchorsVelocities):
            anchorPosition = spider.LEG_ANCHORS[i]
            wx, wy, wz = velocityGlobal[3:]
            anchorsVelocities[i] = np.array([
                anchorVelocity[0],
                anchorVelocity[1] + spider.BODY_RADIUS * wz, 
                anchorVelocity[2] - anchorPosition[0] * wy + anchorPosition[1] * wx
                ])

        referenceLegVelocities = np.copy(anchorsVelocities) * (-1)

        # Legs jacobi matrices.
        J = [kinematics.legJacobi(leg, currentQd[leg]) for leg in legs]
        currentQdd = [np.dot(np.linalg.inv(jac), referenceLegVelocities[j]) for j, jac in enumerate(J)]

        qD.append(currentQd)
        qDd.append(currentQdd)
        xDd.append(referenceLegVelocities)
    
    qD = np.array(qD)
    qDd = np.array(qDd)
    xDd = np.array(xDd)

    # Plot trajectory and velocities.
    fig, axs = plt.subplots(3,3)
    axs[0,0].title.set_text("X direction.")
    axs[0,0].plot(timeVector, trajectory[:,0], timeVector, velocity[:,0])
    axs[0,0].legend(["trajectory", "velocity"])
    axs[1,0].title.set_text("Y direction.")
    axs[1,0].plot(timeVector, trajectory[:,1], timeVector, velocity[:,1])
    axs[1,0].legend(["trajectory", "velocity"])
    axs[2,0].title.set_text("Z direction.")
    axs[2,0].plot(timeVector, trajectory[:,2], timeVector, velocity[:,2])
    axs[2,0].legend(["trajectory", "velocity"])
    axs[0,1].title.set_text("Roll direction.")
    axs[0,1].plot(timeVector, trajectory[:,3], timeVector, velocity[:,3])
    axs[0,1].legend(["trajectory", "velocity"])
    axs[1,1].title.set_text("Pitch direction.")
    axs[1,1].plot(timeVector, trajectory[:,4], timeVector, velocity[:,4])
    axs[1,1].legend(["trajectory", "velocity"])
    axs[2,1].title.set_text("Yaw direction.")
    axs[2,1].plot(timeVector, trajectory[:,5], timeVector, velocity[:,5])
    axs[2,1].legend(["trajectory", "velocity"])

    # Plot reference positions for leg with id = legId.
    legId = 0
    axs[0,2].title.set_text("First leg joints reference positions.")
    axs[0,2].plot(timeVector, qD[:,legId][:,0], timeVector, qD[:,legId][:,1], timeVector, qD[:,legId][:,2])
    axs[0,2].legend(["1.joint", "2.joint", "3.joint"])
    # Plot reference joints velocities for leg.
    axs[1,2].title.set_text("First leg reference joints velocities.")
    axs[1,2].plot(timeVector, qDd[:,legId][:,0], timeVector, qDd[:,legId][:,1], timeVector, qDd[:,legId][:,2])
    axs[1,2].legend(["1.joint", "2.joint", "3.joint"])
    # Plot reference velocities for leg.
    axs[2,2].title.set_text("Reference leg-tip velocities.")
    axs[2,2].plot(timeVector, xDd[:,legId][:,0], timeVector, xDd[:,legId][:,1], timeVector, xDd[:,legId][:,2])
    axs[2,2].legend(["X direction", "Y direction", "Z direction"])
    plt.show()
    