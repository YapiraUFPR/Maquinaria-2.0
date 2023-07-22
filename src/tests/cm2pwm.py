from libs.DC_Motor_pi import DC_Motor
from libs.encoder import Encoder
import time

# encoder and mapping values
STEPS_NUMBER = 7 * 20
RPM = 800
RADIUS_WHEEL = 1.6  # cm
STATIC_COEFICIENT = 1
AXIS_DISTANCE = 10  # cm
A = 1
B = 1
MAP_FNAME = "map.json"

# pins setup
clockwise_pin_1 = 13
counterclockwise_pin_1 = 11
pwm_pin_1 = 12

clockwise_pin_2 = 16
counterclockwise_pin_2 = 15
pwm_pin_2 = 18

encoder_a_ml = 35
encoder_b_ml = 33
encoder_a_mr = 19
encoder_b_mr = 21

motor_left = DC_Motor(clockwise_pin_1, counterclockwise_pin_1, pwm_pin_1)
motor_right = DC_Motor(clockwise_pin_2, counterclockwise_pin_2, pwm_pin_2)

encoder_ml = Encoder(encoder_a_ml, encoder_b_ml, STEPS_NUMBER, RADIUS_WHEEL)
encoder_mr = Encoder(encoder_a_mr, encoder_b_mr, STEPS_NUMBER, RADIUS_WHEEL)


for i in range(25, 101, 25):
    for j in range(2):
        input(f"Will run for the {j} time with speed {i}")
        encoder_ml.distance = 0
        encoder_mr.distance = 0
        start_ts = time.time()
        motor_left.run(i)
        motor_right.run(i)

        distance = (encoder_ml.distance + encoder_mr.distance) / 2
        while distance < 100:
            print(distance)
            distance = (encoder_ml.distance + encoder_mr.distance) / 2

        end_ts = time.time() - start_ts
        motor_left.stop()
        motor_right.stop()
        time_s = end_ts
        print(f"Made {distance} in {time_s}. {i} PWM = {distance/time_s} cm/s")
