import threading

class CustomThread:
    def run(self, function, threadName, isDaemon, start = True, funcArgs = ()):
        killThreadEvent = threading.Event()
        funcArgs = funcArgs + (killThreadEvent, )
        thread = threading.Thread(target = function, args = funcArgs, name = threadName, daemon = isDaemon)
        try:
            if start:
                thread.start()
            print(f"Thread {threadName} is running.")
        except RuntimeError as re:
            print(f"Cannot run thread {threadName}, because {re}.")
            
        return thread, killThreadEvent