import requests
import json
import sys
import numpy as np
import threading
import time
sys.path.append('..')

from utils import threadmanager
import config

class ServerComm:
    """Class for communication with server via http requests.
    """
    def __init__(self, spider_dict_path) :
        self.HEADERS = {"Access-Control-Allow-Origin": "*"}
        self.MESSAGE = 'm'
        self.WARNING = 'w'
        self.ERROR = 'e'

        self.thread_manager = threadmanager.CustomThread()
        self.locker = threading.Lock()
        self.post_message_event = threading.Event()
        self.spider_dict_path = spider_dict_path
        self.posting_spider_position_thread = None
        self.posting_spider_position_thread_kill_event = None
        self.posting_message_thread = None
        self.posting_message_thread_kill_event = None
        self.message = str()

        self.__start_posting_spider_position()
        self.__start_posting_messages()

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
      
    def send_message(self, message):
        """Send message to server.

        Args:
            message (str): Message.
        """
        self.post_message_event.set()
        with self.locker:
            self.message = message
    
    def __start_posting_spider_position(self):
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
        self.posting_spider_position_thread, self.posting_spider_position_thread_kill_event = self.thread_manager.run(
            posting_spider_position,
            config.UPDATE_DATA_THREAD_NAME,
            False,
            True
        )
   
    def __start_posting_messages(self):
        """Post messages to server when event is set.
        """
        def posting_messages(kill_event):
            while not kill_event.is_set():
                if self.post_message_event.is_set():
                    try:
                        _ = requests.post(config.POST_STATE_MESSAGE_ADDR, data = self.message, headers = self.HEADERS, timeout = 1.0)
                    except requests.exceptions.RequestException as e:
                        print(f"Exception {e} at posting to server.")
                    self.post_message_event.clear()
                time.sleep(1)
        
        self.posting_message_thread, self.posting_message_thread_kill_event = self.thread_manager.run(posting_messages, config.SEND_MESSAGE_DATA_THREAD_NAME, False)
