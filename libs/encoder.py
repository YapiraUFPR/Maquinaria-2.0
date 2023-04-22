import RPi.GPIO as GPIO
import time
import math


class Encoder:
    def __init__(self, encoder_a, encoder_b, steps, radius_wheel):
        # init pins
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        GPIO.setup(encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.steps = steps
        self.radius_wheel = radius_wheel
        
        # init state vars
        self.last_a_state = GPIO.input(encoder_a)
        self.current_dir = 1
        self.pulse_counter = 0
        self.modular_pulse_counter = 0
        self.period = 1
        self.period_start = 0
        self.wave_state = 0
        self.last_ts = time.time()
        self.start_ts = self.last_ts

        self.frequency = 0
        self.calc_rpm = 0
        self.rotations = 0
        self.distance = 0
        should_read = True
        GPIO.add_event_detect(encoder_a, GPIO.RISING, callback=self.read_encoders_callback)

    
    def read_encoders_callback(self, channel):
        # this is a highly experimental function
        # this should be able to detect encoder readings according to encoder_test.py

        current_a_state = 1
        # check if encoder detected a turn
        self.modular_pulse_counter += 1

        b_state = GPIO.input(self.encoder_b)
        # check direction
        if b_state != current_a_state:
            self.current_dir = 1
        else:
            self.current_dir = -1

    #def calculate_encoder_values(self):

        # some rotory encoder calculations
        self.frequency = 1/self.period
        self.calc_rpm = self.frequency * 60 // self.steps
        self.rotations = self.modular_pulse_counter // self.steps 

        self.distance += ((2 * math.pi * self.radius_wheel) / self.steps) * self.current_dir
        # self.distance = ((2 * math.pi * self.radius_wheel) / self.steps) * self.modular_pulse_counter * self.current_dir

        # if self.modular_pulse_counter >= self.steps:
        #     self.modular_pulse_counter = 0

    # def __del__(self):
    #     GPIO.remove_event_detect(self.encoder_a)
    #     print("deleted")
        