import requests
import json
import sys
import numpy as np
import threading
sys.path.append('..')

from utils import threadmanager
import config

class ServerComm:
    """Class for communication with server via http requests.
    """
    def __init__(self, spider_dict_path) :
        self.HEADERS = {"Access-Control-Allow-Origin": "*"}

        self.thread_manager = threadmanager.CustomThread()
        self.locker = threading.Lock()
        self.spider_dict_path = spider_dict_path
        self.posting_data_thread = None
        self.posting_data_thread_kill_event = None

        self.start_posting_spider_position()

    def request_goal_position(self):
        """Request goal position of a spider's movement.

        Returns:
            numpy.ndarray: 1x3 array of goal position.
        """
        try:
            request = requests.get(config.REQUEST_SENSOR_POSITION_ADDR, timeout = 1.0)
            if len(request._content.decode()) != 0:
                with self.locker:
                    data = json.loads(request._content)
                goal_position = np.array([data[0], data[1], 0.0])
        except requests.exceptions.RequestException as e:
                    print(f"Exception {e} at requesting sensor position data.")
        return goal_position
    
    def start_posting_spider_position(self):
        """Start thread for continuously posting spider's position.
        """
        def posting_spider_position(kill_event):
            while True:
                try:
                    with open(self.spider_dict_path, "r", encoding = 'utf-8') as f:
                        pins = json.loads(f.read())
                    _ = requests.post(config.POST_SPIDER_POSITION_ADDR, json = pins, headers = self.HEADERS, timeout = 1.0)
                except requests.exceptions.RequestException as e:
                    print(f"Exception {e} at sending spider state data.")

                if kill_event.wait(timeout = 5):
                    break
        self.posting_data_thread, self.posting_data_thread_kill_event = self.thread_manager.run(posting_spider_position, config.UPDATE_DATA_THREAD_NAME, False, True)
   
    def post_to_server(self, address, data):
        """Post data to server.

        Args:
            address (str): Http addres for posting.
            data (str): Data to be posted.
        """
        try:
            _ = requests.post(address, data = data, headers = self.HEADERS, timeout = 1.0)
        except requests.exceptions.RequestException as e:
            print(f"Exception {e} at posting to server.")
