import cv2 as cv
import numpy as np


# image = cv.imread("/home/gnhn/maquiteste/photo2.jpeg")

capture = cv.VideoCapture(0)
capture.set(cv.CAP_PROP_FPS, 90)
capture.set(cv.CAP_PROP_FRAME_WIDTH, 320)
capture.set(cv.CAP_PROP_FRAME_HEIGHT, 400)



retval, image = capture.read()

while retval:



    height, width, _ = image.shape


    # image = cv.resize(image, (width//2, height//2))
    # height = height // 2
    # width = width // 2

    w, h = width, height



    points = [(3 * width // 8, (height // 2) + 30), (5 * width // 8, (height // 2) + 30), (1 * width // 8, height-10), (7 * width // 8, height-10)]
    original_perspective = np.float32(points)
    new = np.float32([(0, 0), (w, 0), (0, h), (width, height)])

    plot = image
    for point in points:
        #print(point)
        plot = cv.circle(plot, point, 3, (255, 155, 155), cv.FILLED)


    matrix = cv.getPerspectiveTransform(original_perspective, new)
    output = cv.warpPerspective(image, matrix, dsize=(w, h))


    cv.imshow("BRUH1", output)
    cv.waitKey(5)
    #cv.imshow("BRUH2", output)

    retval, image = capture.read()

    cv.waitKey(1000 // 30)
