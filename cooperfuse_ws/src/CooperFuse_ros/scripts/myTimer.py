import time

class Timer:
    def __init__(self):
        self.start_time = time.time() * 1000  # Initial time in milliseconds

    def takeRealTime(self):
        # Get the real time elapsed since the timer was created (in milliseconds)
        current_time = time.time() * 1000
        elapsed_time = current_time - self.start_time
        return elapsed_time

# Example usage
if __name__ == "__main__":
    timer = Timer()  # Create a timer

    # Simulate some operation
    time.sleep(2)

    # Get the elapsed time from creating the timer until now
    elapsed_time = timer.takeRealTime()
    print(f"Elapsed Time (ms): {elapsed_time}")
