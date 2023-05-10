import numpy as np

import config
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import transformations as tf
from calculations import mathtools as mt

from environment import spider

five_three_array = np.zeros((5, 3), dtype = np.float32)
one_three_array = np.zeros(3, dtype = np.float32)
random_five_three_array = np.random.rand(5, 3).astype(np.float32)
random_three_three_matrix = np.random.rand(3, 3).astype(np.float32)
random_one_three_array = np.random.rand(3).astype(np.float32)
random_five_three_three_matrix = np.random.rand(5, 3, 3).astype(np.float32)
random_three_three_three_matrix = np.random.rand(3, 3, 3).astype(np.float32)

def initNumbaFunctions():
    dyn.get_torques_and_forces_on_legs_tips(five_three_array, five_three_array, one_three_array)
    dyn.get_gravity_rotation_matrices(one_three_array, 0.0)
    dyn.get_force_rotation_matrices(one_three_array)
    dyn.get_torques_in_legs(random_five_three_array, random_five_three_array, random_one_three_array)
    dyn.calculate_gravity_compensation_torques(random_five_three_array, random_one_three_array)
    dyn.calculate_gravity_vectors(random_five_three_three_matrix, random_one_three_array)
    dyn.calculate_torques(0, random_three_three_three_matrix, random_three_three_matrix)
    dyn.create_diag_transpose_J_hash(random_five_three_array)

    kin.all_legs_positions(five_three_array, config.LEG_ORIGIN)
    kin.leg_forward_kinematics(one_three_array)
    kin.spider_base_to_leg_tip_forward_kinematics(0, one_three_array)
    kin.legInverseKinematics(np.array((0.3, 0.0, 0.0), dtype = np.float32))
    kin.legJacobi(one_three_array)
    kin.spider_base_to_leg_tip_jacobi(0, one_three_array)
    kin.getJointsVelocities(random_five_three_array, five_three_array)
    kin.getXdXddFromOffsets(spider.LEGS_IDS, five_three_array, five_three_array)

    tf.R_B1(0.0, 0.25)
    tf.R_12(0.25)
    tf.R_23(0.3)
    tf.R_B2(0.0, 0.3, 0.2)
    tf.R_B3(0.0, 0.12, 0.2, 0.3)

    mt.damped_pseudo_inverse(random_three_three_matrix)
