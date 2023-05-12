""" Module for mapping angles between leg model and motors. """

import math
import numpy as np

def map_position_encoder_values_to_model_angles_radians(encoders_values):
    """Map encoders' values of each motor in leg to model angles in radians.

    Args:
        encoders_values (list): 5x3 array of encoders' values of each motor in leg.

    Returns:
        numpy.ndarray: 5x3 array of leg-model angles in radians.
    """
    mapped_values = np.copy(encoders_values).astype(np.float32)
    k = np.array([math.pi / 2048, -math.pi / 2048, math.pi / 2048], dtype = np.float32)
    n = np.array([-math.pi, math.pi, -math.pi], dtype = np.float32)

    return k * mapped_values + n

def map_current_encoder_values_to_motors_currents_ampers(encoder_values):
    """Map encoded current values of each motor in leg to currents in motors in Ampers.

    Args:
        encoder_values (list): 5x3 array of encoded current values of each motor in leg.

    Returns:
        numpy.ndarray: 5x3 array of currents in motors in Ampers.
    """
    mapped_values = np.copy(encoder_values).astype(np.float32)
    mapped_values[mapped_values > 0x7fff] -= 65535

    return mapped_values * 0.00269

def map_model_velocities_to_velocity_encoder_values(model_velocities):
    """Map velocities of each joint in leg-model to velocity encoders values.

    Args:
        model_velocities (list): 1x3 array of velocities in each joint of leg-model.

    Returns:
        numpy.ndarray: 1x3 array of encoded motors' velocities, matching given leg-model velocities.
    """
    rpm_per_unit = 0.229
    model_velocities = np.array(model_velocities)
    # Rad/s to rad/min.
    joint_velocities_rpm = (60 / (2 * math.pi)) * model_velocities

    # Convert to encoder values.
    encoder_values = joint_velocities_rpm / rpm_per_unit
    encoder_values[1] = -encoder_values[1]

    return encoder_values
