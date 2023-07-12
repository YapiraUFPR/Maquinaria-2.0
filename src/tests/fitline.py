import cv2

glare = cv2.imread('image-000.jpg')
thrsh = cv2.imread('image-002.png')

thrsh = cv2.cvtColor(thrsh, cv2.COLOR_BGR2GRAY)
thrsh = cv2.threshold(thrsh, 127, 255, cv2.THRESH_BINARY)[1]

contours, _ = cv2.findContours(thrsh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

(height, width) = thrsh.shape

line = {}
for contour in contours:
    M = cv2.moments(contour)
    # Search more about Image Moments on Wikipedia :)

    contour_vertices = len(cv2.approxPolyDP(contour, 1.5, True))
    # print("vertices: ", contour_vertices)

    if M["m00"] < 17000:
        continue

    if (contour_vertices < 65) and (M["m00"] > 22000):

        cv2.drawContours(glare, contour, -1, (255, 255, 0), 2)
        # Contour is part of the track
        line["x"] = int(M["m10"] / M["m00"])
        line["y"] = int(M["m01"] / M["m00"])

        cv2.circle(glare, (line["x"], line["y"]), 10, (0, 128, 0), -1)
        # put the coordinates of the points
        cv2.putText(glare, f"({line['x']},{line['y']})", (line["x"] + 20, line["y"] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 128, 0), 2)

        # plot the amount of vertices in light blue
        #cv2.putText(glare, "track", (line["x"] + 100, line["y"] - 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        vx, vy, x1, y1 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        y2 = y1 + vy
        x2 = x1 + vx

        y = line["y"] - 200
        x = int(x1 + (y - y1) / ((y2 - y1) / (x2 - x1)))
        if x < 0 or x > width:
            x = line["x"]

        # # print(vy, y, y1)
        # [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        # lefty = int((-vy/vx) + y)
        # righty = int(((width-x)*vy/vx)+y)
        # cv2.line(glare, (width-1, righty), (0, lefty), (128, 0, 128),1, 2)

        rows,cols = glare.shape[:2]
        cv2.circle(glare, (int(width / 2), int(height / 2)), 10, (0, 165, 255), -1)
        cv2.putText(glare, f"({int(width / 2)},{int(height / 2)})", (int(width / 2) + 10, int(height / 2) + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
        cv2.circle(glare, (int(x), int(y)), 10, (128, 0, 0), -1)
        cv2.putText(glare, f"({int(x)},{int(y)})", (int(x) + 10, int(y) + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 0, 0), 2)
        [vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
        lefty = int((-x*vy/vx) + y)
        righty = int(((cols-x)*vy/vx)+y)
        cv2.line(glare,(cols-1,righty),(0,lefty),(128,0,128),2)


  

        #print(f"circle at {x, y}")
        # plot the line point in dark green
        #cv2.putText(glare, "line point", (x + 100, y - 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 0), 2)
        # plot the center of the image in orange

cv2.imshow("glare", glare)
cv2.waitKey(0)
cv2.imwrite("fortnite.png", glare)