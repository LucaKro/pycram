import functools


def failure_handling(max_retries=-1):
    def decorator_retry(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            retries = 0
            while True:
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    retries += 1
                    if max_retries >= 0 and retries > max_retries:
                        raise
                    print(f"Retrying due to error: {e}, attempt {retries}")
        return wrapper
    return decorator_retry


import threading

class Fluent:
    def __init__(self, initial_value=None):
        self.value = initial_value
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.pulsed = False

    def set_value(self, new_value):
        with self.lock:
            self.value = new_value
            self.pulsed = True
            self.condition.notify_all()

    def get_value(self):
        with self.lock:
            return self.value

    def wait_for(self):
        with self.lock:
            while self.value is None:
                self.condition.wait()

    def wait_for_pulse(self):
        with self.lock:
            was_pulsed = self.pulsed
            while not was_pulsed:
                self.condition.wait()
                was_pulsed = self.pulsed
            self.pulsed = False

    def pulse(self):
        with self.lock:
            self.pulsed = True
            self.condition.notify_all()
