import threading
import time

class CustomThread:
    def run(self, function, threadName, isDaemon, start = True, useKillEvent = True, doPrint = True, funcArgs = ()):
        """Create and run given function in separate thread. 

        Args:
            function (function): Function to run in separate thread.
            threadName (str): Name of the thread.
            isDaemon (bool): Whether or not thread to be a daemon.
            start (bool, optional): Whether or not to start thread. Defaults to True.
            useKillEvent (bool, optional): Whether or not to use event to kill a thread. Defaults to True.
            doPrint(bool, optional): Whether or not print message.
            funcArgs (tuple, optional): Needed arguments for given function, given as a Tuple. Defaults to ().

        Returns:
            Tuple or Thread: If kill event is used return thread and kill event, otherwise return only thread.
        """
        if useKillEvent:
            killThreadEvent = threading.Event()
            funcArgs = funcArgs + (killThreadEvent, )
        thread = threading.Thread(target = function, args = funcArgs, name = threadName, daemon = isDaemon)
        try:
            if start:
                thread.start()
                while not thread.is_alive():
                    time.sleep(0)
            if doPrint:
                print(f"Thread {threadName} is running.")
        except RuntimeError as re:
            print(f"Cannot run thread {threadName}, because {re}.")
        
        if useKillEvent:
            return thread, killThreadEvent
        return thread 