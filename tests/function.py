import cv2
import numpy as np
import astar
import math

## MAKE ALL THESE VALUES INTERACTIVE
lower_bgr_values = np.array([115,  115,  40])
upper_bgr_values = np.array([255, 255, 255])
MIN_AREA = 10000
MIN_AREA_TRACK = 30000
MAX_CONTOUR_VERTICES = 100
ERODE_KERNEL_SIZE = 5



def get_contour_data(mask, out):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line)
    and draw all contours on 'out' image
    """

    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}
    over = False
    tried_once = False

    possible_tracks = []
    for contour in contours:
        M = cv2.moments(contour)
        # Search more about Image Moments on Wikipedia 🙂 (it's the 'center')

        contour_vertices = len(cv2.approxPolyDP(contour, 1.5, True))

        if (M['m00'] < MIN_AREA):
            continue

        if (contour_vertices < MAX_CONTOUR_VERTICES) and (M['m00'] > MIN_AREA_TRACK):

            # Contour is part of the track
            line['x'] = int(M["m10"]/M["m00"])
            line['y'] = int(M["m01"]/M["m00"])

            possible_tracks.append(line)

            # plot the amount of vertices in light blue
            cv2.drawContours(out, contour, -1, (255,255,0), 2)

            cv2.putText(out, f"({contour_vertices}){M['m00']}", (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])-200),
                cv2.FONT_HERSHEY_PLAIN, 3, (100,100,255), 2)

            points = []
            for element in contour:
                points.append(list(element[0]))
            

            low = max(points, key=lambda x:x[1])
            # print(points)
            
            points.remove(low)

            max_d = 0
            for other_point in points:
                d = math.dist(low, other_point)
                if d > max_d:
                    max_d = d
                    high = other_point
            

            # cv2.circle(image, low, 5, (0,255,255), 5)
            # cv2.circle(image, high, 5, (255,255,0), 5)

            # rows,cols = out.shape[:2]
            vx,vy,x1,y1 = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
            y2 = y1 + vy
            x2 = x1 + vx

            y = line["y"] - 200
            x = int(x1 + (y - y1)/((y2-y1)/(x2-x1)))
            if x < 0:
                x = 0
            if x > width:
                x = width

            cv2.circle(image, (x, y), 50, (45,50,255), 50)
            print(x,y)


        else:
            # plot the area in pink
            cv2.drawContours(out, contour, -1, (255,0,255), 2)
            cv2.putText(out, f"({contour_vertices}){M['m00']}", (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                cv2.FONT_HERSHEY_PLAIN, 3, (100,100,255), 2)

    return possible_tracks

# image_path = "glare.jpeg"
# image_path = "color_semifuck.png"
# image_path = "./new_colorfuck2.png"
# image_path = "./theory.png"
image_path = "./theory2.png"



image = cv2.imread(image_path)
mask = cv2.inRange(image, lower_bgr_values, upper_bgr_values)

## SHOW IMAGE ACCORGING TO COLOR THRESHOLDS


kernel = np.ones((ERODE_KERNEL_SIZE, ERODE_KERNEL_SIZE), np.uint8)
mask = cv2.erode(mask, kernel)

height, width, _ = image.shape

## SHOW IMAGE ACCORGING TO KERNEL SIZE

possible_tracks = get_contour_data(mask, image)

for line in possible_tracks:
    cv2.circle(image, (line['x'], line['y']), 5, (0,255,0), 5)

## SHOW ALL POSSIBLE TRACKS ACCORDING TO MIN_AREA, MIN_AREA_TRACK and MAX_CONTOUR_VERTICES

cv2.imshow("image", image)
cv2.imwrite("output.png", image)

# cv2.imshow("mask", mask)
cv2.waitKey(0)