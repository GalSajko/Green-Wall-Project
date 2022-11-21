# import adafruit_bno055
# import board
# import numpy as np

# class BNO055:
#     """Class for communication with BNO055 sensor, connected with i2c protocol.
#     """
#     def __init__(self, isVertical = False):
#         i2c = board.I2C()
#         self.bno055 = adafruit_bno055.BNO055_I2C(i2c)
#         self.isVertical = isVertical
#         # Remap axis to fix initial value of pitch, when spider is verticaly on the wall.
#         if isVertical:
#             self.bno055.axis_remap = (0, 2, 1, 0, 1, 0)
#         print("BNO055 initializing...")
#         time.sleep(2)
#         self.initRpyOffsets = mappers.mapBno055ToSpiderDegrees(self.bno055.euler, isVertical)
#         self.prevGravityVector = np.zeros(3)
#         print(f"Initial offsets are: {self.initRpyOffsets}")
    
#     def readEulers(self):
#         """Read, map and return spider's orientation given in rpy euler angles, considering initial offsets of sensor.

#         Returns:
#             numpy.ndarray: 1x3 array of spider's roll, pitch and yaw angles in radians.
#         """
#         rpy = mappers.mapBno055ToSpiderDegrees(self.bno055.euler)
#         rpy -= self.initRpyOffsets

#         return rpy
    
#     def readGravity(self):
#         """Read gravity vector.

#         Returns:
#             list: 1x3 gravity vector.
#         """
#         gravity = self.bno055.gravity
#         try:
#             gravity = mappers.mapGravityVectorToSpiderOrigin(gravity, self.isVertical)
#             if np.linalg.norm(gravity) > 10.0:
#                 return self.prevGravityVector
#             self.prevGravityVector = gravity
#             return gravity
#         except TypeError:
#             return self.prevGravityVector