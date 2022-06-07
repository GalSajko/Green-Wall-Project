""" Module for mapping angles between leg model and motor values. """

import math
import numpy as np

def mapJointRadiansToMotorRadians(jointsValues):
    """Map angles from Inverse kinematics in radians into motor angles in radians.

    :param jointsValues: Values of joints in leg.
    :return: Motors angles in radians that match input angles.
    """
    q1, q2, q3 = jointsValues
    q1 = q1 + math.pi
    q2 = math.pi - q2
    q3 = 1.5 * math.pi + q3

    return q1, q2, q3

def mapMotorRadiansToJointRadians(motorValues):
    """Map angles from motors in radians to joint angles in radians.

    :param motorValues: Motor values in radians.
    :return: Joint values in radians.
    """
    q1, q2, q3 = motorValues

    q1 = q1 - math.pi
    q2 = -(q2 - math.pi)
    q3 = q3 - 1.5 * math.pi

    return q1, q2, q3

def mapMotorRadiansToEncoder(jointValues):
    """Map radians in motors to encoder values.

    :param jointValues: Angles in motors in radians. 
    :return: Encoder values to match input angles.
    """
    jointValues = np.degrees(jointValues)
    encoderBits = 12
    # Convert joints values to encoder values.
    jointValues = np.array(jointValues)
    encoderValues = np.array(jointValues * math.pow(2, encoderBits) / 360.0)

    return encoderValues

def mapEncoderToMotorsDegrees(encoderValues):
    """Map encoder values to motors degrees.

    :param encoderValues: Encoder values for each motor in leg.
    :return: Motors degrees for each motor in leg.
    """
    encoderBits = 12
    # Convert encoders values in degrees.
    encoderValues = np.array(encoderValues)

    return encoderValues * 360.0 / math.pow(2, encoderBits)

def mapEncoderToJointsRadians(encoderValues):
    """Map encoders values to joints radians.

    :param encoderValues: Encoders values for each motor in single leg.
    :return: 1x3 array of joints radians in leg.
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

