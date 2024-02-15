import time
import math
from statistics import mean

class LoopTimer:
    def __init__(self):
        self.reset()

    def reset(self): 
        self.loop_total_active_time = 0.0
        self.loop_sum_of_squared_durations = 0.0
        self.loop_iterations = 0
        self.loop_min_time = 1000000.0
        self.loop_max_time = 0.0
        self.loop_current_time = 0.0
        self.loop_iteration_duration = 0.0
        self.loop_total_active_time = 0.0
        self.loop_average_duration = 0.0
        self.loop_average_sum_of_squared_durations = 0.0
        self.loop_duration_standard_deviation = 0.0
        self.loop_recent_timing = []
        self.loop_recent_timing_length = 10
        
    def start_of_iteration(self):
        self.loop_current_iteration_start_time = time.time()

    def pretty_print(self):
        print()
        print('--- LOOP TIMING ---')
        print('number of iterations =', self.loop_iterations)
        print('average period =', "{:.2f}".format(self.loop_average_duration * 1000.0),  'ms')
        print('period standard deviation =', "{:.2f}".format(self.loop_duration_standard_deviation * 1000.0),  'ms')
        print('min period =', "{:.2f}".format(self.loop_min_time * 1000.0),  'ms')
        print('max period =', "{:.2f}".format(self.loop_max_time * 1000.0),  'ms')
        print('min frequency =', "{:.2f}".format(1.0/self.loop_max_time),  'Hz')
        print('max frequency =', "{:.2f}".format(1.0/self.loop_min_time),  'Hz')
        small_std_dev_freq = 1.0/(self.loop_average_duration + self.loop_duration_standard_deviation)
        high_std_dev_freq = 1.0/(self.loop_average_duration - self.loop_duration_standard_deviation)
        print('one standard deviation frequencies =', "{:.2f} Hz, {:.2f} Hz".format(small_std_dev_freq, high_std_dev_freq))
        print('average frequency over all time =', "{:.2f}".format(1.0/self.loop_average_duration),  'Hz')
        recent_length = len(self.loop_recent_timing)
        if recent_length > 0:
            print('average frequency over last ' + str(recent_length) + ' iterations = {:.2f} Hz'.format(1.0/mean(self.loop_recent_timing)))
        print('-----------------------------------------------')

        
    def end_of_iteration(self):
        self.loop_current_time = time.time()
        self.loop_iteration_duration = self.loop_current_time - self.loop_current_iteration_start_time
        self.loop_recent_timing.append(self.loop_iteration_duration)
        if len(self.loop_recent_timing) > self.loop_recent_timing_length:
            self.loop_recent_timing.pop(0)
        self.loop_total_active_time = self.loop_total_active_time + self.loop_iteration_duration
        self.loop_iterations = self.loop_iterations + 1
        self.loop_average_duration = self.loop_total_active_time / self.loop_iterations
        self.loop_sum_of_squared_durations = self.loop_sum_of_squared_durations + self.loop_iteration_duration**2
        self.loop_average_sum_of_squared_durations = self.loop_sum_of_squared_durations / self.loop_iterations
        self.loop_duration_standard_deviation = math.sqrt(self.loop_average_sum_of_squared_durations - self.loop_average_duration**2)
        if self.loop_min_time > self.loop_iteration_duration:
            self.loop_min_time = self.loop_iteration_duration
        if self.loop_max_time < self.loop_iteration_duration:
            self.loop_max_time = self.loop_iteration_duration

