"""Module for leg calibration.
"""
#!/usr/bin/python3

import dynamixel as dxl
import math


class Calibrator:

    def __init__(self, legId, referencePoints):
        self.legId = legId
        motorsIds = [[11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43], [51, 52, 53]]
        self.motorDriver = dxl.MotorDriver(motorsIds[legId])
        self.referencePoints = referencePoints

    def calibrateLeg(self, initValues):
        L1, L2, L3, L4 = initValues
        

        self.motorDriver.addGroupSyncReadParams(self.legId)

        for idx, referencePoint in enumerate(self.referencePoints):  
            _ = raw_input(f"Move leg on {idx + 1}. reference point and press any key.")

            q1, q2, q3 = self.motorDriver.syncReadMotorsPositionsInLegs([self.legId])
            x = math.cos(q1)*(L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2))
            y = math.sin(q1)*(L1 + L2*math.cos(q2) + L4*math.cos(q2+q3) + L3*math.sin(q2))
            z = -L3*math.cos(q2) + L2*math.sin(q2) + L4*math.sin(q2+q3)

            dXdL1 = math.cos(q1)
            dXdL2 = math.cos(q1) * math.cos(q2)
            dXdL3 = math.cos(q1) * math.sin(q2)
            dXdL4 = math.cos(q1) * math.cos(q2 + q3)

            dYdL1 = math.sin(q1)
            dYdL2 = math.sin(q1) * math.cos(q2)
            dYdL3 = math.sin(q1) * math.sin(q2)
            dYdL4 = math.sin(q1) * math.cos(q2 + q3)

            dZdL1 = 0
            dZdL2 = math.sin(q2)
            dZdL3 = -math.cos(q2)
            dZdL4 = math.sin(q2 + q3)