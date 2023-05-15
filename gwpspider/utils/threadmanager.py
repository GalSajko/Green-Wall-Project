import threading
import time

class CustomThread:
    def run(self, function, thread_name, is_daemon, start = True, use_kill_event = True, do_print = True, function_args = ()):
        """Create and run given function in separate thread. 

        Args:
            function (function): Function to run in separate thread.
            thread_name (str): Name of the thread.
            is_daemon (bool): Whether or not thread to be a daemon.
            start (bool, optional): Whether or not to start thread. Defaults to True.
            use_kill_event (bool, optional): Whether or not to use event to kill a thread. Defaults to True.
            do_print(bool, optional): Whether or not print message.
            function_args (tuple, optional): Needed arguments for given function, given as a Tuple. Defaults to ().

        Returns:
            Tuple or Thread: If kill event is used return thread and kill event, otherwise return only thread.
        """
        if use_kill_event:
            kill_thread_event = threading.Event()
            function_args = function_args + (kill_thread_event, )
        thread = threading.Thread(target = function, args = function_args, name = thread_name, daemon = is_daemon)
        try:
            if start:
                thread.start()
                while not thread.is_alive():
                    time.sleep(0)
            if do_print:
                print(f"Thread {thread_name} is running.")
        except RuntimeError as re:
            print(f"Cannot run thread {thread_name}, because {re}.")
        
        if use_kill_event:
            return thread, kill_thread_event
        return thread 

