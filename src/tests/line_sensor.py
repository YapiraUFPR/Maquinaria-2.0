import RPi.GPIO as GPIO
import time

line_sensor_out = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(line_sensor_out, GPIO.IN)

while True:
    print(GPIO.input(line_sensor_out))
    time.sleep(1)

GPIO.cleanup()