# import random
from game_files.agents.pace_setting import Pace

import time

class CustomPace(Pace):
    def __init__(self):
        super().__init__()
        self.shooting_interval = 2  # Set a custom shooting interval here

class EqualSupport(CustomPace):
    def __init__(self):
        super().__init__()
        self.support = 1
        self.start_time = time.time()
        self.timer_duration = 120  # Set the duration of the timer in seconds
        self.last_check_time = self.start_time

    def run_ai(self, state):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        remaining_time = self.timer_duration - elapsed_time



        # Print the remaining time
        minutes = int(remaining_time // 60)
        seconds = int(remaining_time % 60)
        print(f"Time remaining: {minutes:02d}:{seconds:02d}")

        if current_time - self.last_check_time >= 5:
            self.last_check_time = current_time
            if elapsed_time < self.timer_duration:
                if self.support == 1:
                    self.support = 2
                else:
                    self.support = 1
            else:
                self.support = 1
        return True
