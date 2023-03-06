import requests
import json
import sys
import numpy as np
sys.path.append('..')

import threadmanager
import config
import threading
import config

class CommunicationWithServer:
    def __init__(self, spiderDictPath) :
        self.threadManager = threadmanager.CustomThread()
        self.sensorPosition = []
        self.locker = threading.Lock()
        self.spiderDictPath = spiderDictPath
        self.updateSensorPositionData()
    def getGoalPos(self):
        yOffset = 24.0
           
        start  = 0
        try:
            request = requests.get(config.GET_SENSOR_POSITION_ADDR)
            xDim = 20
            yDim = 25
            if len(request._content.decode()) != 0:
                with self.locker:
                    self.data = json.loads(request._content)
                if self.data[0] == 1 or self.data[0] == 4:
                    start = 20
                elif self.data[0] == 3 or self.data[0] == 6:
                    start = -20
                if self.data[0] < 4:
                    y = (((5-self.data[1]) + 6) * yDim + yOffset) / 100.0
                    x = (start +((self.data[0] - 1) * 7 + (config.SENSOR_IDS.index(self.data[2]))) * xDim + xDim/2) / 100.0
                else:
                    y = (((5-self.data[1])) * yDim + yOffset) / 100.0
                    x = (start + ((self.data[0] - 4) * 7 + (config.SENSOR_IDS.index(self.data[2]))) * xDim + xDim / 2) / 100.0
                    # TODO: Calibrate plant z offset.
                    self.sensorPosition=np.array([x , y, 0.0])
        except Exception as e:
                    print(f"Exception {e} at reading sensor position data.")
        return self.sensorPosition
    def updateSensorPositionData(self):
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server. It also sends POST requests with spider position to the server.
        """
        def updatingSensorPositionData(killEvent):
            while True:
                try:
                    with open(self.spiderDictPath, "r", encoding = 'utf-8') as f:
                        pins = json.loads(f.read())
                    request = requests.post(config.POST_SPIDER_POSITION,json=pins, headers= {"Access-Control-Allow-Origin": "*"})
                except Exception as e:
                    print(f"Exception {e} at sending spider state data.")
                if killEvent.wait(timeout = 5): 
                    break
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.threadManager.run(updatingSensorPositionData, config.UPDATE_DATA_THREAD_NAME, False, True)
