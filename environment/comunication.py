import requests
from flask import Flask, request, jsonify, render_template
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
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server.
        """
        def updatingSensorPositionData(killEvent):
            while True:
                try:
                    request = requests.get(config.GET_SENSOR_POSITION_ADDR)
                    if len(request._content.decode()) != 0:
                        with self.locker:
                            self.sensorPosition=json.loads(request._content)
                    
                # Tukaj lovis vse mogoce exceptione, do katerih lahko pride pri klicih get(), decode(), len() in loads() metod. 
                # Ne pravim, da to ni OK, samo mogoce si vzami par minut in premisli, ce bi kateri od teh zahteval kaksen poseben handling.
                except:
                    print("will retry in a second")
                f = open("../spider_state_dict","r")
                pins = json.loads(f.read())
                print(pins)
                request = requests.post(config.POST_SPIDER_POSITION,pins, headers= {"Access-Control-Allow-Origin": "*"})
          
                if killEvent.wait(timeout = 30): 
                    break
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.threadManager.run(updatingSensorPositionData, config.UPDATE_DATA_THREAD_NAME, False, True)
