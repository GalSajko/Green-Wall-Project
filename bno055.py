import adafruit_bno055
import board
import mappers


class BNO055:
    """Class for communication BNO055 sensor, connected with i2c protocol.
    """
    def __init__(self):
        i2c = board.I2C()
        self.bno055 = adafruit_bno055.BNO055_I2C(i2c)
        # Remap axis to fix initial value of pitch, when spider is verticaly on the wall.
        self.bno055.axis_remap = (0, 2, 1, 0, 1, 0)
    
    def readEulers(self, returnRadians = False):
        """Read, map and return spider's orientation given in rpy euler angles.

        Args:
            returnRadians (bool, optional): If True, return angles in radians, else in degrees. Defaults to False.

        Returns:
            list: Spider's roll, pitch and yaw angles.
        """
        return mappers.mapBno055ToSpiderDegrees(self.bno055.euler, returnRadians)

        