import requests
import json
import sys
import numpy as np
sys.path.append('..')

import threadmanager
import config
import threading
import config
from jsonfilemanager import JsonFileManager

class CommunicationWithServer:
    def __init__(self) :
        self.threadManager = threadmanager.CustomThread()
        self.sensorPosition = []
        self.locker = threading.Lock()
        self.updateSensorPositionData()

    def updateSensorPositionData(self):
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server. It also sends POST requests with spider position to the server.
        """
        def updatingSensorPositionData(killEvent):
            
            while True:
                start  = 0
                try:
                    request = requests.get(config.GET_SENSOR_POSITION_ADDR)
                    xDim = 20
                    yDim = 25
                    if len(request._content.decode()) != 0:
                        with self.locker:
                            self.data = json.loads(request._content)
                            print(self.data)
                            if self.data[0] == 1 or self.data[0] == 4:
                                start = 20
                            elif self.data[0] == 3 or self.data[0] == 6:
                                start = -20
                            if self.data[0]<4:
                                y = (((self.data[1])+6)*yDim+yDim/2)/100.0
                                x = (start +( (self.data[0]-1)*7+(config.SENSOR_IDS.index(self.data[2])))*xDim+xDim/2)/100.0
                            else:
                                y = (((self.data[1]))*yDim+yDim/2)/100.0
                                x = (start +(( self.data[0]-4)*7+(config.SENSOR_IDS.index(self.data[2])))*xDim+xDim/2)/100.0
                            self.sensorPosition=np.array([x , y, 0.0])
                            print(self.sensorPosition)
                except:
                    print("will retry in a second")
                try:
                    f = open("../spider_state_dict","r")
                    pins = json.loads(f.read())
                    print(pins)
                    request = requests.post(config.POST_SPIDER_POSITION,json=pins, headers= {"Access-Control-Allow-Origin": "*"})
                except:
                    print("Spider pos error")
          
                if killEvent.wait(timeout = 30): 
                    break
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.threadManager.run(updatingSensorPositionData, config.UPDATE_DATA_THREAD_NAME, False, True)
