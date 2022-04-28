""" Module for mapping angles between DH model and motor values. """

from json import encoder
import math
import numpy as np

def mapJointRadiansToMotorRadians(jointValues):
    """Map angles from Inverse kinematics in radians into motor angles in radians.

    :param q1: First joint angle.
    :param q2: Second joint angle.
    :param q3: Third joint angle.
    :return: Motors angles in radians that match input angles.
    """
    q1, q2, q3 = jointValues
    q1 = q1 + math.pi
    q2 = math.pi - q2
    q3 = 1.5 * math.pi + q3

    return q1, q2, q3

def mapMotorRadiansToJointRadians(motorValues):
    """Map angles from motors in degrees to joint angles in degrees.

    :param motorValues: Motor values in degrees.
    :return: Joint values in degrees.
    """

    q1, q2, q3 = motorValues
    

    q1 = q1 - math.pi
    q2 = -(q2 - math.pi)
    q3 = q3 - 1.5 * math.pi

    return q1, q2, q3

def mapMotorRadiansToEncoder(jointValues):
    """Map degrees in motors to encoder values.

    :param jointValues: Angles in motors in degrees. 
    :return: Encoder values to match input angles.
    """
    jointValues = np.degrees(jointValues)
    encoderBits = 12
    # Convert joints values to encoder values.
    jointValues = np.array(jointValues)
    encoderValues = np.array(jointValues * 2**encoderBits / 360.0)

    return encoderValues

def mapEncoderToMotorsDegrees(encoderValues):
    """Map encoder values to motors degrees.

    :param encoderValues: Encoder values for each motor in leg.
    :return: Motors degrees for each motor in leg.
    """
    encoderBits = 12
    # Convert encoders values in degrees.
    encoderValues = np.array(encoderValues)
    jointValues = []

    jointValues.append(encoderValues[0] * 360.0 / 2**encoderBits)
    jointValues.append(encoderValues[1] * 360.0 / 2**encoderBits)
    jointValues.append(encoderValues[2] * 360.0 / 2**encoderBits)

    return np.array(jointValues)

def mapEncoderToJointRadians(encoderValues):
    """Map encoders values to joints radians.

    :param encoderValues: Encoders values for each motor in single leg.
    :return: 1x3 array of joints values in leg.
    """
    encoderValues = np.array(encoderValues)
    k = np.array([math.pi / 2048, -math.pi / 2048, math.pi / 2048])
    n = np.array([-math.pi, math.pi, -3*math.pi / 2])

    return np.array(k * encoderValues + n)

def mapJointVelocitiesToEncoderValues(jointVelocities):
    """Map calculated joints velocities to encoder values.

    :param jointVelocities: Reference joints velocities in rad/s.
    :return: Encoder values to match desired joints velocities.
    """
    encoderVelocityLimit = 75
    jointVelocitiyRpmLimit = 17.18

    jointVelocities = np.array(jointVelocities)
    # Rad/s to rad/min.
    jointVelocitiesRpm = (60 / (2*math.pi)) * jointVelocities

    # Convert to encoder values.
    encoderValues = jointVelocitiesRpm * (encoderVelocityLimit / jointVelocitiyRpmLimit)
    encoderValues[1] = -encoderValues[1]

    return encoderValues

