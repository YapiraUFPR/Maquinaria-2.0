import RPi.GPIO as GPIO
import time

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
        self.current_dir = ""
        self.pulse_counter = 0
        self.total_pulse_counter = 0
        self.period = 1
        self.period_start = 0
        self.wave_state = 0
        self.last_ts = time.time()
        self.start_ts = self.last_ts

        self.frequency = 0
        self.calc_rpm = 0
        self.rotations = 0
        self.radius_wheel = 0
        self.distance = 0

    
    def read_encoders(self):
    # this is a highly experimental function
    # this should be able to detect encoder readings according to encoder_test.py

        current_a_state = GPIO.input(self.encoder_a)

        # calculate frequency
        if self.wave_state == 0:
            if current_a_state == 1 and self.last_a_state == 0:
                self.period = (time.time_ns() / 1000) - self.period_start
                self.period_start = time.time_ns() / 1000
                self.wave_state = 1
        elif self.wave_state == 1:
            if current_a_state == 0 and self.last_a_state == 1:
                self.wave_state = 0

        # check if encoder detected a turn
        if current_a_state != self.last_a_state and current_a_state == 1:
            self.total_pulse_counter += 1

            b_state = GPIO.input(self.encoder_b)
            # check direction
            if b_state != current_a_state:
                self.current_dir = "anti-horário"
                self.pulse_counter += 1
            else:
                self.current_dir = "horário"
                self.pulse_counter -= 1

        self.last_a_state = current_a_state

        # some rotory encoder calculations
        self.frequency = 1/self.period
        self.calc_rpm = self.frequency * 60 // self.steps
        self.rotations = self.total_pulse_counter // self.steps 
        self.distance = ((2 * 3.14 * self.radius_wheel) / self.steps) * self.pulse_counter

    def __del__(self):
        GPIO.cleanup()