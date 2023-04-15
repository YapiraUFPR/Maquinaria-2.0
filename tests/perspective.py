import cv2 as cv
import numpy as np

image = cv.imread("./perspective.png")

height, width, _ = image.shape


points = [(3 * width // 8, (height // 2) + 30), (5 * width // 8, (height // 2) + 30), (1 * width // 8, height - 5), (7 * width // 8, height - 5)]
original_perspective = np.float32(points)
new = np.float32([(0, 0), (width, 0), (0, height), (width, height)])
matrix = cv.getPerspectiveTransform(original_perspective, new)

perspective = cv.warpPerspective(image, matrix, dsize=(width, height))


plot = image
for point in points:
    #print(point)
    plot = cv.circle(plot, point, 3, (255, 155, 155), cv.FILLED)

matrix = cv.getPerspectiveTransform(original_perspective, new)


cv.imshow("BRUH1", image)
cv.imshow("BRUH2", perspective)
cv.waitKey(0)
