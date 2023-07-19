import cv2
import numpy as np 

ALPHA = 1.5
BETA = 30
CLIP_LIMIT = 8

video = cv2.VideoCapture("../samples/shadow-57.mp4")

retval, image = video.read()
h, w, _ = image.shape

output_shape = ((w//2)*5, h)

writer = cv2.VideoWriter_fourcc(*"mp4v")
output_video = cv2.VideoWriter("output.mp4", writer, 10, output_shape)

clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT)

rw = w//2

while retval:
    cut = image[:, 0:w//2, :]


    gray = cv2.cvtColor(cut, cv2.COLOR_BGR2GRAY)

    glahe_img = clahe.apply(gray)
    scale_img = cv2.convertScaleAbs(cut, alpha=ALPHA, beta=BETA)
    scale_img_gray = cv2.convertScaleAbs(gray, alpha=ALPHA, beta=BETA)
    both = clahe.apply(cv2.cvtColor(scale_img, cv2.COLOR_BGR2GRAY))

    output = np.hstack((cut, scale_img, cv2.cvtColor(scale_img_gray, cv2.COLOR_GRAY2BGR), cv2.cvtColor(glahe_img, cv2.COLOR_GRAY2BGR), cv2.cvtColor(both, cv2.COLOR_GRAY2BGR)))

    cv2.putText(output, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(output, "Scale", (w//2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(output, "Scale Gray", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(output, "CLAHE", (w + rw + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(output, "Scale CLAHE", (w + rw*2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    output_video.write(output)

    retval, image = video.read()

output_video.release()
video.release()