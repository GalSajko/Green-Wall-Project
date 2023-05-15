import requests
import json
import sys
import numpy as np
import threading
sys.path.append('..')

from utils import threadmanager
import config

class ServerComm:
    def __init__(self, spider_dict_path) :
        self.thread_manager = threadmanager.CustomThread()
        self.sensor_position = []
        self.locker = threading.Lock()
        self.spider_dict_path = spider_dict_path
        self.data = None

        self.update_sensor_position_data()

    def get_goal_position(self):
        try:
            request = requests.get(config.GET_SENSOR_POSITION_ADDR, timeout = 1.0)
            if len(request._content.decode()) != 0:
                with self.locker:
                    self.data = json.loads(request._content)
                self.sensor_position = np.array([self.data[0], self.data[1], 0.0])
        except Exception as e:
                    print(f"Exception {e} at reading sensor position data.")
        return self.sensor_position
    
    def update_sensor_position_data(self):
        """Comunication procedure, creates a thread that continuously sends GET requests to the server and updates the data variable with values from the server. It also sends POST requests with spider position to the server.
        """
        def updating_sensor_position_data(kill_event):
            while True:
                try:
                    with open(self.spider_dict_path, "r", encoding = 'utf-8') as f:
                        pins = json.loads(f.read())
                    _ = requests.post(config.POST_SPIDER_POSITION, json = pins, headers = {"Access-Control-Allow-Origin": "*"}, timeout = 1.0)
                except Exception as e:
                    print(f"Exception {e} at sending spider state data.")

                if kill_event.wait(timeout = 5):
                    break
        self.updatingDataThread, self.updatingDataThreadKillEvent = self.thread_manager.run(updating_sensor_position_data, config.UPDATE_DATA_THREAD_NAME, False, True)
   
    def post_refilling(self):
        try:
            _ = requests.post(config.POST_REFILL, data = "Refilling", headers = {"Access-Control-Allow-Origin": "*"}, timeout = 1.0)
        except Exception as e:
            print(f"Exception {e} at posting refilling intention.")

    def post_stop(self):
        try:
            _ = requests.post(config.POST_STOP, data = "stop refilling", headers = {"Access-Control-Allow-Origin": "*"}, timeout = 1.0)
        except Exception as e:
            print(f"Exception {e} at posting refilling intention.")

