import adafruit_bno055
import board
import mappers
import time


class BNO055:
    """Class for communication BNO055 sensor, connected with i2c protocol.
    """
    def __init__(self):
        i2c = board.I2C()
        self.bno055 = adafruit_bno055.BNO055_I2C(i2c)
        # Remap axis to fix initial value of pitch, when spider is verticaly on the wall.
        self.bno055.axis_remap = (0, 2, 1, 0, 1, 0)
    
    def readEulers(self):
        """Read, map and return spider's orientation given in rpy euler angles.

        Returns:
            tuple: Spider's orientation given as r, p, y euler angles in radians.
        """
        return mappers.mapBno055ToSpiderDegrees(self.bno055.euler, True)

if __name__ == '__main__':
    bno = BNO055()

    while True:
        r, p, y = bno.readEulers()
        print(r, p, y)
        time.sleep(0.1)
        