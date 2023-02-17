import requests
import json
import sys
sys.path.append('..')
import threadmanager
import config
import threading

class CommunicationWithServer:
    def __init__(self) :
        self.threadManager = threadmanager.CustomThread()
        self.statesObjectsLocker = threading.Lock()

        
    def updatePositionData(self):
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server.
        """
        def updatingPositionData(killEvent):
            while 1:
                try:
                    request = requests.get('http://192.168.1.20:5000/zalij')
                    if len(request._content.decode())!=0:
                        self.data=json.loads(request._content)
                except:
                    print("will retry in a second")
                if killEvent.wait(timeout = 2): 
                    break
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.threadManager.run(updatingPositionData, config.UPDATE_DATA_THREAD_NAME, False, True)
