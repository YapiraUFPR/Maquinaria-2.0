import cv2 

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FPS, 90)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

retval, image = video.read()

fps_count = 0
ts = time.time()

while retval:
    retval, image = video.read()

    fps_count += 1
    if time.time() - ts >= 1:
        print(f"FPS: {fps_count}")
        fps_count = 0
        ts = time.time()

