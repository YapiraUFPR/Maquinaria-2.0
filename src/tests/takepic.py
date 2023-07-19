import cv2
from time import sleep

capture = cv2.VideoCapture(0)

for i in range(1, 20):  
    sleep(2)
    print("took picture")
    retval, frame = capture.read()
    if retval:
        cv2.imwrite(f"out_{i}.jpg", frame)
