"""Module for all dynamics calculations.
"""
import numpy as np
import math
import numba

import config
import spider
from calculations import kinematics as kin
from calculations import mathtools as mathTools
from calculations import transformations as tf

A_TORQUE_POLYNOM = 0.0
B_TORQUE_POLYNOM = 2.9326
C_TORQUE_POLYNOM = -0.1779

# region public methods
@numba.jit(nopython=True, cache=False)
def get_torques_and_forces_on_legs_tips(
    joints_values: np.ndarray,
    currents_in_motors: np.ndarray,
    spider_gravity_vector: np.ndarray,
    number_of_legs: int = spider.NUMBER_OF_LEGS) -> tuple[np.ndarray, np.ndarray]:
    """Calculate forces, applied to tips of all legs, from currents in motors.

    Args:
        joints_values (np.ndarray): 5x3 array of angles in joints.
        currents_in_motors (np.ndarray): 5x3 array of currents in motors.
        spider_gravity_vector (np.ndarray): 1x3 gravity vector in spider's origin.
        number_of_legs (int, optional): Number of legs used in calculation. Defaults to spider.NUMBER_OF_LEGS.

    Returns:
        tuple[np.ndarray, np.ndarray]: Calculated torques in motors and forces on legs' tips.
    """
    torques = get_torques_in_legs(joints_values, currents_in_motors, spider_gravity_vector)
    forces = np.zeros((number_of_legs, 3), dtype=np.float32)

    for leg_id, joint_values in enumerate(joints_values):
        J = kin.spider_base_to_leg_tip_jacobi(leg_id, joint_values)
        J_hash = mathTools.damped_pseudoinverse(J)
        forces[leg_id] = np.dot(np.transpose(J_hash), torques[leg_id])

    return torques, forces

def calculate_distributed_forces(measured_torques: np.ndarray, joints_values: np.ndarray, legs_ids: list, offload_leg_id: int) -> np.ndarray:
    """Calculated distributed forces from minimized torques.

    Args:
        measured_torques (np.ndarray): 5x3 array of measured torques in joints.
        joints_values (np.ndarray): 5x3 array of angles in joints, in radians.
        legs_ids (list): List of used legs' ids.
        offload_leg_id (int): Id of leg which will be offloaded.

    Returns:
        np.ndarray: Distributed forces on legs' tips.
    """
    W, J_x, J_hash_trans_diag = _get_spider_external_forces(measured_torques, joints_values)

    # weights = np.eye(15)
    # weights[0, 0] = 0.1
    # weights[3, 3] = 0.1
    # weights[6, 6] = 0.1
    # weights[9, 9] = 0.1
    # weights[12, 12] = 0.1

    if len(offload_leg_id):
        J_x = np.delete(J_x, range(offload_leg_id[0] * 3, (offload_leg_id[0] * 3) + 3), axis=1)
        # weights = np.delete(weights, range(offload_leg_id[0] * 3, (offload_leg_id[0] * 3) + 3), axis = 0)
        # weights = np.delete(weights, range(offload_leg_id[0] * 3, (offload_leg_id[0] * 3) + 3), axis = 1)
        J_hash_trans_diag = np.delete(J_hash_trans_diag, range(offload_leg_id[0] * 3, (offload_leg_id[0] * 3) + 3), axis=0)
        J_hash_trans_diag = np.delete(J_hash_trans_diag, range(offload_leg_id[0] * 3, (offload_leg_id[0] * 3) + 3), axis=1)

    # Jxw = mathTools.weighted_pseudoinverse(J_x, weights)
    dist_torques_array = np.dot(np.linalg.pinv(J_x), W)
    # distTorquesArray = np.dot(Jxw, W)
    dist_forces_array = np.dot(J_hash_trans_diag, dist_torques_array)

    return np.reshape(dist_forces_array, (len(legs_ids), 3))

@numba.jit(nopython=True, cache=False)
def get_gravity_rotation_matrices(joints_values_in_leg: np.ndarray, q_b: float) -> np.ndarray:
    """Get array of rotation matrices from spider's origin to segments' origins in order that can be used in recursive dynamics calculations.
    Args:
        joints_values_in_leg (np.ndarray): 1x3 array of values in joints, in radians.
        q_b (float): Z-axis rotation between spider's and leg's origin.

    Returns:
        np.ndarray: 3x3x3 array of rotation matrices.
    """
    q_1, q_2, q_3 = joints_values_in_leg

    rotation_matrices = np.zeros((3, 3, 3), dtype=np.float32)
    rotation_matrices[0] = tf.R_B1(q_b, q_1)
    rotation_matrices[1] = tf.R_B2(q_b, q_1, q_2)
    rotation_matrices[2] = tf.R_B3(q_b, q_1, q_2, q_3)

    return rotation_matrices

@numba.jit(nopython=True, cache=False)
def get_force_rotation_matrices(joints_values_in_leg: np.ndarray) -> np.ndarray:
    """Get array of rotation matrices from consecutive segments' origins in order that can be used in recursive dynamics calculations.

    Args:
        joints_values_in_leg (np.ndarray): 1x3 array of values in joints, in radians.

    Returns:
        np.ndarray: 2x3x3 array of rotation matrices.
    """
    force_matrices = np.zeros((2, 3, 3), dtype=np.float32)
    force_matrices[0] = tf.R_23(joints_values_in_leg[2])
    force_matrices[1] = tf.R_12(joints_values_in_leg[1])

    return force_matrices

@numba.jit(nopython=True, cache=False)
def get_torques_in_legs(joints_values: np.ndarray, currents_in_motors: np.ndarray, spider_gravity_vector: np.ndarray) -> np.ndarray:
    """Calculate torques in leg-joints from measured currents.

    Args:
        joints_values (np.ndarray): 5x3 array of angles in joints.
        currents_in_motors (np.ndarray): 5x3 array of currents in motors.
        spider_gravity_vector (np.ndarray): 3x1 gravity vector in spider's origin.

    Returns:
        np.ndarray: 5x3 array of torques in motors.
    """
    currents = np.copy(currents_in_motors)
    currents[:, 1] *= -1
    currents[:, 2] *= 2

    gravity_torques = calculate_gravity_compensation_torques(joints_values, spider_gravity_vector)
    torques = ((A_TORQUE_POLYNOM + B_TORQUE_POLYNOM * currents + C_TORQUE_POLYNOM * currents**2) - gravity_torques).astype(np.float32)

    return torques

@numba.jit(nopython=True, cache=False)
def calculate_gravity_compensation_torques(
    joints_values: np.ndarray,
    spider_gravity_vector: np.ndarray,
    number_of_legs: int =spider.NUMBER_OF_LEGS,
    number_of_motors_in_leg: int =spider.NUMBER_OF_MOTORS_IN_LEG,
    angle_between_legs: float = spider.ANGLE_BETWEEN_LEGS) -> np.ndarray:
    """Calculate torques in joints (for all legs), required to compensate movement, caused only by gravity.

    Args:
        joints_values (np.ndarray): 5x3 array of angles in joints, in radians.
        spider_gravity_vector (np.ndarray): 1x3 gravity vector in spider's origin.
        number_of_legs (int, optional): Number of legs. Defaults to spider.NUMBER_OF_LEGS.
        number_of_motors_in_leg (int, optional): Number of motors in leg. Defaults to spider.NUMBER_OF_MOTORS_IN_LEG.
        angle_between_legs (float, optional): Angle between legs. Defaults to spider.ANGLE_BETWEEN_LEGS.

    Returns:
        np.ndarray: Torques in motors required for compensation of gravity force.
    """
    torques = np.zeros((number_of_legs, number_of_motors_in_leg), dtype=np.float32)

    for leg_id, joints_in_leg in enumerate(joints_values):
        q_b = leg_id * angle_between_legs + math.pi / 2.0
        gravity_rotation_matrices = get_gravity_rotation_matrices(joints_in_leg, q_b)
        force_rotation_matrices = get_force_rotation_matrices(joints_in_leg)

        local_gravity_vectors = calculate_gravity_vectors(gravity_rotation_matrices, spider_gravity_vector)
        leg_torques = calculate_torques(leg_id, force_rotation_matrices, local_gravity_vectors)
        torques[leg_id] = leg_torques

    return torques

@numba.jit(nopython=True, cache=False)
def calculate_gravity_vectors(
    gravity_rotation_matrices: np.ndarray,
    spider_gravity_vector: np.ndarray,
    number_of_segments: int = spider.NUMBER_OF_MOTORS_IN_LEG) -> np.ndarray:
    """Calculate gravity vectors in segments' origins.

    Args:
        gravity_rotation_matrices (np.ndarray): 5x3x3 array of five 3x3 rotation matrices.
        spider_gravity_vector (np.ndarray): 1x3 gravity vector in spider's origin.
        number_of_segments (int, optional): Number of segments in leg. Defaults to spider.NUMBER_OF_MOTORS_IN_LEG.

    Returns:
        np.ndarray: 3x3 array of three local gravity vectors, given in segments' origins.
    """
    local_gravity_vectors = np.zeros((number_of_segments, 3), dtype=np.float32)
    for i in range(3):
        local_gravity_vectors[i] = np.dot(
            np.transpose(gravity_rotation_matrices[i]), spider_gravity_vector
        )
    return local_gravity_vectors

@numba.jit(nopython=True, cache=False)
def calculate_torques(
    leg_id: int,
    force_rotation_matrices: np.ndarray,
    local_gravity_vectors: np.ndarray,
    cog_vectors: np.ndarray = spider.VECTORS_TO_COG_SEGMENT,
    legs_dimensions: np.ndarray = spider.LEGS_DIMENSIONS,
    segment_masses: np.ndarray = spider.SEGMENTS_MASSES) -> np.ndarray:
    """Calculate torques in the motors, using Newton-Euler method.

    Args:
        leg_id (int): Leg id.
        force_rotation_matrices (np.ndarray): 3x3x3 array of three rotation matrices.
        local_gravity_vectors (np.ndarray): 3x3 array of three gravity vectors in segments' origins.
        cog_vectors (np.ndarray, optional): Vectors from start of segment to its center of gravity. Defaults to spider.VECTORS_TO_COG_SEGMENT.
        legs_dimensions (np.ndarray, optional): Dimensions of leg's segments. Defaults to spider.LEGS_DIMENSIONS.
        segment_masses (np.ndarray, optional): Masses of leg's segments. Defaults to spider.SEGMENTS_MASSES.

    Returns:
        np.ndarray: Torques in motors.
    """
    torques_vectors_in_leg = np.zeros((3, 3), dtype=np.float32)
    torques_values_in_leg = np.zeros(3, dtype=np.float32)
    forces = np.zeros((3, 3), dtype=np.float32)

    for i in range(3):
        l_c = np.array([1, 0, 0], dtype=np.float32) * cog_vectors[leg_id][2 - i]
        l = np.array([1, 0, 0], dtype=np.float32) * legs_dimensions[2 - i]

        if i != 0:
            f_g_segment = segment_masses[leg_id][2 - i] * local_gravity_vectors[2 - i]
            forces[i] = (np.dot(force_rotation_matrices[i - 1], forces[i - 1]) - f_g_segment)
            torques_vectors_in_leg[i] = (
                np.dot(force_rotation_matrices[i - 1], torques_vectors_in_leg[i - 1])
                + np.cross(f_g_segment, l_c)
                - np.cross(np.dot(force_rotation_matrices[i - 1], forces[i - 1]), l)
            )
            torques_values_in_leg[i] = torques_vectors_in_leg[i][2]
            continue

        forces[i] = -segment_masses[leg_id][2 - i] * local_gravity_vectors[2 - i]
        torques_vectors_in_leg[i] = (-1) * np.cross(forces[i], l_c)
        torques_values_in_leg[i] = torques_vectors_in_leg[i][2]

    return np.flip(torques_values_in_leg)

@numba.jit(nopython=True, cache=False)
def create_diag_transpose_J_hash(joints_values: np.ndarray) -> np.ndarray:
    """Create diagonal matrix from transposed damped pseudo-inverses of jacobian matrices.

    Args:
        joints_values (np.ndarray): nx3 array of angles in joints in radians, where n is number of used legs.

    Returns:
        np.ndarray: (3*n)x(3*n) diagonal matrix of transposed damped-pseudo-inverses of jacobian matrices.
    """
    diag_J_hash_trans = np.zeros((3 * len(joints_values), 3 * len(joints_values)))
    for leg_id, joints_in_leg in enumerate(joints_values):
        J_hash_trans = np.transpose(mathTools.damped_pseudoinverse(kin.spider_base_to_leg_tip_jacobi(leg_id, joints_in_leg)))
        diag_J_hash_trans[3 * leg_id : 3 * leg_id + len(J_hash_trans), 3 * leg_id : 3 * leg_id + len(J_hash_trans)] = J_hash_trans

    return diag_J_hash_trans
# endregion

# region private methods
def _get_spider_external_forces(measured_torques: np.ndarray, joints_values: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate external forces and torques from all internal torques and joints values.

    Args:
        measured_torques (np.ndarray): 5x3 array of measured torques in motors.
        joints_values (np.ndarray): 5x3 array of angles in joints in radians.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: External forces and torques, internal to external force transformation matrix, diagonal matrix of transpose psevdoinverse Jacobian matrices.
    """
    measured_torques_array = measured_torques.flatten()
    J_f = _create_J_f_matrix()
    x_a = kin.all_legs_positions(joints_values, config.SPIDER_ORIGIN)
    J_m = _create_J_m_matrix(x_a)
    Jfm = np.r_[J_f, J_m]
    diag_J_hash_trans = create_diag_transpose_J_hash(joints_values)
    J_x = np.dot(Jfm, diag_J_hash_trans)
    W = np.dot(J_x, measured_torques_array)

    return W, J_x, diag_J_hash_trans

def _create_J_m_matrix(legs_positions: np.ndarray) -> np.ndarray:
    """Create J_m matrix from antisimetric matrices of position vectors

    Args:
        legs_positions (np.ndarray): nx3 array of legs' positions.

    Returns:
        np.ndarray: 3x(3*n) J_m matrix, where n is number of used legs.
    """
    legs_positions = np.array(legs_positions, dtype=np.float32)
    for i, leg_position in enumerate(legs_positions):
        x, y, z = leg_position
        antisim_matrix = np.array(
            [[0, -z, y], 
             [z, 0, -x], 
             [-y, x, 0]], dtype=np.float32
        )
        if i == 0:
            J_m = antisim_matrix
            continue
        J_m = np.c_[J_m, antisim_matrix]

    return J_m

def _create_J_f_matrix() -> np.ndarray:
    """Create J_f matrix from 3x3 identity matrices.

    Returns:
        np.ndarray: 3x(3*n) J_f matrix, where n is number of legs.
    """
    for i in range(spider.NUMBER_OF_LEGS):
        if i == 0:
            J_f = np.eye(3, dtype=np.float32)
            continue
        J_f = np.c_[J_f, np.eye(3, dtype=np.float32)]
    return J_f
# endregion
