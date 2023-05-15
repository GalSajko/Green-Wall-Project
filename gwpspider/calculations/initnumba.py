import numpy as np

import config
import spider
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools as mt

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
    kin.leg_inverse_kinematics(np.array((0.3, 0.0, 0.0), dtype = np.float32))
    kin.leg_jacobi(one_three_array)
    kin.spider_base_to_leg_tip_jacobi(0, one_three_array)
    kin.get_joints_velocities(random_five_three_array, five_three_array)
    kin.get_xd_xdd_from_offsets(spider.LEGS_IDS, five_three_array, five_three_array)

    mt.damped_pseudoinverse(random_three_three_matrix)
