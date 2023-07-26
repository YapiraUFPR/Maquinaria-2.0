import json 
import numpy as np

## User-defined parameters: (Update these values as necessary)

RESIZE_FACTOR = 3
min_area = 400
min_area_track = 1500
max_contour_vertices = 25
LINE_CONTOUR_VERTICES = 5

linear_speed = 60.0
linear_speed_on_curve = 30.0
linear_speed_on_loss = 30.0

kp = 0.45
kd = 0.50                                                                                                                                                                                               
ALPHA = 1
BETA = 0

CURVE_ERROR_THRH = 22
LOSS_THRH = 40

FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS = 10
after_loss_count = FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS + 1

LOSS_FACTOR = 1.2
NO_MOVEMENT_FRAMES = 3

lower_bgr_values = np.array([80, 80, 80])
upper_bgr_values = np.array([255, 255, 255])

# Inserting the parameters into the dictionary
params_dict = {
    "RESIZE_FACTOR": RESIZE_FACTOR,
    "MIN_AREA": min_area,
    "MIN_AREA_TRACK": min_area_track,
    "MAX_CONTOUR_VERTICES": max_contour_vertices,
    "LINE_CONTOUR_VERTICES": LINE_CONTOUR_VERTICES,
    "LINEAR_SPEED": linear_speed,
    "LINEAR_SPEED_ON_CURVE": linear_speed_on_curve,
    "LINEAR_SPEED_ON_LOSS": linear_speed_on_loss,
    "KP": kp,
    "KD": kd,
    "lower_bgr_values": lower_bgr_values.tolist(),
    "upper_bgr_values": upper_bgr_values.tolist()
}

json.dump(params_dict, open("safe.json", "w"))
