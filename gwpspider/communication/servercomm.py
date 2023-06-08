import requests
import json
import sys
import numpy as np
import threading
import time
from gwpconfig import commconstants as cc
sys.path.append('..')

from utils import threadmanager
import config

class ServerComm:
    """Class for communication with server via http requests.
    """
    def __init__(self, spider_dict_path: str):
        """Class constructor. Starts threads for posting messages.

        Args:
            spider_dict_path (str): Path to file for reading spider's current state.
        """
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
    
    def request_watering_action_instructions(self) -> tuple[np.ndarray, bool, float]:
        """Request instruction for following spider's move.

        Returns:
            tuple[np.ndarray, bool, float]: Goal position, whether or not to refill water tank, volume of water that needs to be pumped using water pumps.
        """
        try:
            request = requests.get(cc.REQUEST_WATERING_INSTRUCTION_ADDR, timeout = 1.0)
            if request.status_code == requests.codes.ok:
                data = json.loads(request.content)
                goal_position = np.array([data[0], data[1], 0.0])
                action = data[2]
                volume = data[3]
            else:
                print(f"Watering instructions request status code {request.status_code}")
        except requests.exceptions.RequestException as e:
            print(e)

        return goal_position, bool(action), volume
      
    def send_message(self, message: str):
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
                    _ = requests.post(cc.POST_SPIDER_POSITION_ADDR, json = pins, headers = self.HEADERS, timeout = 1.0)
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
                        _ = requests.post(cc.POST_STATE_MESSAGE_ADDR, data = self.message, headers = self.HEADERS, timeout = 1.0)
                    except requests.exceptions.RequestException as e:
                        print(f"Exception {e} at posting to server.")
                    self.post_message_event.clear()
                time.sleep(1)
        
        self.posting_message_thread, self.posting_message_thread_kill_event = self.thread_manager.run(posting_messages, config.SEND_MESSAGE_DATA_THREAD_NAME, False)
