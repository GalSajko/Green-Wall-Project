import requests
import json
import sys
sys.path.append('..')
import threadmanager
import config
import threading
import config

class CommunicationWithServer:
    def __init__(self) :
        self.threadManager = threadmanager.CustomThread()
        self.sensorPosition = []
        self.locker = threading.Lock()

    def updateSensorPositionData(self):
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server. It also sends POST requests with spider position to the server.
        """
        def updatingSensorPositionData(killEvent):
            while True:
                try:
                    request = requests.get(config.GET_SENSOR_POSITION_ADDR)
                    
                    if len(request._content.decode()) != 0:
                        with self.locker:
                            print(json.loads(request._content))
                            self.sensorPosition=json.loads(request._content)
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
