""" Module for mapping angles between leg model and motors. """

import math
import numpy as np

def mapPositionEncoderValuesToModelAnglesRadians(encodersValues):
    """Map encoders' values of each motor in leg to model angles in radians.

    Args:
        encodersValues (list): 5x3 array of encoders' values of each motor in leg.

    Returns:
        numpy.ndarray: 5x3 array of leg-model angles in radians.
    """
    mappedValues = np.copy(encodersValues).astype(np.float32)
    k = np.array([math.pi / 2048, -math.pi / 2048, math.pi / 2048], dtype = np.float32)
    n = np.array([-math.pi, math.pi, -math.pi], dtype = np.float32)

    return k * mappedValues + n

def mapCurrentEncoderValuesToMotorsCurrentsAmpers(encoderValues):
    """Map encoded current values of each motor in leg to currents in motors in Ampers.

    Args:
        encoderValues (list): 5x3 array of encoded current values of each motor in leg.

    Returns:
        numpy.ndarray: 5x3 array of currents in motors in Ampers.
    """
    mappedValues = np.copy(encoderValues).astype(np.float32)
    mappedValues[mappedValues > 0x7fff] -= 65535

    return mappedValues * 0.00269

def mapModelVelocitiesToVelocityEncoderValues(modelVelocities):
    """Map velocities of each joint in leg-model to velocity encoders values.

    Args:
        modelVelocities (list): 1x3 array of velocities in each joint of leg-model.

    Returns:
        numpy.ndarray: 1x3 array of encoded motors' velocities, matching given leg-model velocities.
    """
    rpmPerUnit = 0.229
    modelVelocities = np.array(modelVelocities)
    # Rad/s to rad/min.
    jointVelocitiesRpm = (60 / (2 * math.pi)) * modelVelocities

    # Convert to encoder values.
    encoderValues = jointVelocitiesRpm / rpmPerUnit
    encoderValues[1] = -encoderValues[1]

    return encoderValues

def bno055MapYawDegrees(sensorValue):
    """Map sensor's yaw output to spider's convention.
    Sensor's convention - 0 to 360 in clockwise direction,
    Spider's convetion - 0 to 180, -180 to 0 in counter-clockwise direction.

    Args:
        sensorValue (float): Sensor's yaw value in degrees.

    Returns:
        float: Spider's yaw value in degrees.
    """
    if sensorValue < 0.0:
        sensorValue = 0.0
    elif sensorValue > 360.0:
        sensorValue = 360.0

    if 0.0 <= sensorValue < 180.0:
        return sensorValue * (-1)
    if 180.0 <= sensorValue <= 360.0:
        return (-1) * sensorValue + 360.0

def bno055MapPitchDegrees(sensorValue):
    """Map sensor's pitch output to spider's convention.
    Sensor's convention - 180 to -180 in clockwise direction,
    Spider's convention - 180 to -180 in counter-clockwise direction.

    Args:
        sensorValue (float): Sensor's pitch output in degrees.

    Returns:
        float: Spider's pitch value in degrees.
    """
    return sensorValue * (-1)
