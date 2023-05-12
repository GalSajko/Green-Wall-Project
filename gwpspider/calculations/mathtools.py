"""Math tools needed in other modules.
"""
import math
import numpy as np
import numba

import config

def calculate_signed_angle_between_two_vectors(first_vector, second_vector):
    """Calculate signed angle between two vectors.

    Args:
        first_vector (list): First vector.
        second_vector (list): Second vector.

    Returns:
        float: Signed angle in radians.
    """
    dot_product = np.dot(first_vector, second_vector)
    product_of_norms = np.linalg.norm(first_vector) * np.linalg.norm(second_vector)
    angle = math.acos(np.round(dot_product / product_of_norms, 4))
    cross_product = np.cross(first_vector, second_vector)
    # 2d vector.
    if len(first_vector) <= 2 and cross_product < 0:
        angle = -angle
    # 3d vector.
    elif (cross_product < 0).any() < 0:
        angle = -angle

    return angle

def running_average(buffer, counter, new_value):
    """Calculate running average of values in buffer.

    Args:
        buffer (list): Values, used to calculate average.
        counter (int): Index at which new values are writen in buffer, if it is not already full.
        new_value (list): New values, which will be written in the buffer.

    Returns:
        tuple: Average of the buffer (element wise), shifted buffer and updated counter.
    """
    if counter < len(buffer):
        buffer[counter] = new_value
        counter += 1
        average = np.mean(buffer[:counter], axis = 0)

        return average, buffer, counter
    
    buffer = np.roll(buffer, -1, axis = 0)
    buffer[-1] = new_value
    average = np.mean(buffer, axis = 0)

    return average, buffer, counter

@numba.njit
def damped_pseudoinverse(J, damping = config.FORCE_DAMPING):
    """Calculate damped Moore-Penrose pseudo inverse.

    Args:
        J (list): 3x3 matrix whose pseudo inverse will be calculated.

    Returns:
        numpy.ndarray: 3x3 damped pseudo inverse of J.
    """
    J_trans = np.transpose(J).astype(np.float32)
    J_J_trans = np.dot(J, J_trans).astype(np.float32)
    alpha = np.eye(len(J_J_trans), dtype = np.float32) * damping
    damped_factor = np.linalg.inv(J_J_trans + alpha).astype(np.float32)

    return np.dot(J_trans, damped_factor)

@numba.njit
def weighted_pseudoinverse(J, A):
    J_trans = np.transpose(J).astype(np.float32)
    A_inv = np.linalg.inv(A).astype(np.float32)
    J_w = np.dot(np.dot(A_inv, J_trans), np.linalg.inv(np.dot(J.astype(np.float32), np.dot(A_inv, J_trans))))

    return J_w