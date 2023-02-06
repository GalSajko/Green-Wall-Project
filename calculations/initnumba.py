import numpy as np

import config
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import transformations as tf
from calculations import mathtools as mt

from environment import spider

fiveThreeArray = np.zeros((5, 3), dtype = np.float32)
oneThreeArray = np.zeros(3, dtype = np.float32)
randomFiveThreeArray = np.random.rand(5, 3).astype(np.float32)
randomThreeThreeMatrix = np.random.rand(3, 3).astype(np.float32)
randomOneThreeArray = np.random.rand(3).astype(np.float32)
randomFiveThreeThreeMatrix = np.random.rand(5, 3, 3).astype(np.float32)
randomThreeThreeThreeMatrix = np.random.rand(3, 3, 3).astype(np.float32)

def initNumbaFunctions():
    dyn.getTorquesAndForcesOnLegsTips(fiveThreeArray, fiveThreeArray, oneThreeArray)
    dyn.getGravityRotationMatrices(oneThreeArray, 0.0)
    dyn.getForceRotationMatrices(oneThreeArray)
    dyn.getTorquesInLegs(randomFiveThreeArray, randomFiveThreeArray, randomOneThreeArray)
    dyn.getGravityCompensationTorques(randomFiveThreeArray, randomOneThreeArray)
    dyn.calculateGravityVectors(randomFiveThreeThreeMatrix, randomOneThreeArray)
    dyn.calculateTorques(0, randomThreeThreeThreeMatrix, randomThreeThreeMatrix)
    dyn.createDiagTransposeJHash(randomFiveThreeArray)

    kin.allLegsPositions(fiveThreeArray, config.LEG_ORIGIN)
    kin.legForwardKinematics(oneThreeArray)
    kin.spiderBaseToLegTipForwardKinematics(0, oneThreeArray)
    kin.legInverseKinematics(np.array((0.3, 0.0, 0.0), dtype = np.float32))
    kin.legJacobi(oneThreeArray)
    kin.spiderBaseToLegTipJacobi(0, oneThreeArray)
    kin.getJointsVelocities(randomFiveThreeArray, fiveThreeArray)
    kin.getXdXddFromOffsets(spider.LEGS_IDS, fiveThreeArray, fiveThreeArray)

    tf.R_B1(0.0, 0.25)
    tf.R_12(0.25)
    tf.R_23(0.3)
    tf.R_B2(0.0, 0.3, 0.2)
    tf.R_B3(0.0, 0.12, 0.2, 0.3)

    mt.dampedPseudoInverse(randomThreeThreeMatrix)
