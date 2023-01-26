import numpy as np

from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import transformations as tf
from environment import spider

fiveThreeArray = np.zeros((5, 3), dtype = np.float32)
oneThreeArray = np.zeros(3, dtype = np.float32)
randomFiveThreeArray = np.random.rand(5, 3).astype(np.float32)

def initNumbaFunctions():
    dyn.getTorquesAndForcesOnLegsTips(fiveThreeArray, fiveThreeArray, oneThreeArray)
    kin.allLegsPositions(fiveThreeArray, 'l')
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

