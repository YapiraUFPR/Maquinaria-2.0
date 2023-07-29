#!/usr/bin/env python3
"""
A program used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
You may change the parameters BECAUSE IT IS NECESSARY.
"""
__author__ = "Gabriel Hishida, Gabriel Pontarolo, Allan Cedric, Tiago Serique, Isadora Botassari and Maite Aska"

import numpy as np
import cv2
import RPi.GPIO as GPIO
from libs.DC_Motor_pi import DC_Motor
from libs.encoder import Encoder
from libs.track_map import TrackMap
import requests
from datetime import datetime, timedelta
import argparse
import time
import csv
from os.path import exists
from os import makedirs
import pickle
import json

# init arg parser
parser = argparse.ArgumentParser()
parser.add_argument("-s", "--start", action="store_true", help="Follow line")
parser.add_argument("-r", "--record", action="store_true", help="Record masked image")
parser.add_argument(
    "-w", "--write", action="store_true", help="Write encoder values to csv"
)
parser.add_argument(
    "-o",
    "--output",
    metavar="address",
    action="store",
    help="Show output image to ip address",
)
parser.add_argument(
    "-p", "--stop", metavar="store_true", help="Stop the robot in `RUNTIME` seconds"
)
parser.add_argument(
    "-l", "--linestop", action="store_true", help="Stop by reading the side mark"
)
parser.add_argument(
    "-d",
    "--distance",
    metavar="store_true",
    help="Stop the robot in `DISTANCE` centimetres",
)
parser.add_argument(
    "-m", "--map", action="store_true", help="Create a map of the track"
)
parser.add_argument(
    "-um", "--usemap", metavar="map_file", action="store", help="Use map to follow the line"
)
parser.add_argument(
    "-f", "--file", metavar="store_true", default=None, type=str, help="Load file with parameters"
)
args = parser.parse_args()

############################# DEFINES #############################
###################################################################

# encoder and mapping values
STEPS_NUMBER = 7 * 20
RPM = 800
RADIUS_WHEEL = 1.6  # cm
STATIC_COEFICIENT = 1.7320508075688772
AXIS_DISTANCE = 14.5  # cm
A = 1.0574
B = -3.2959
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

# Global vars. initial values
runtime = 0  # time until it should stop
init_time = int(datetime.now().timestamp())
init_time_iso = datetime.now()
image_input = None
error = 0
error_deriv = 0
angular = 0
last_error = 0
last_error_deriv = 0
total_distance = 0
no_movement_count = 0
just_seen_line = False
saw_right_mark = False
should_stop_for_mark = False
should_move = False
should_record = False
should_show = False
should_stop = False
should_stop_for_distance = False
should_write = False
should_map = False
should_use_map = False
current_strech = 0
record_frames = []
shape = ()
ip_addr = "0.0.0.0"
csv_writer = None
csv_file = None
track_map = None
right_mark_count = 0
frame_count = 0
image_ts = 0
last_image_ts = 0

## User-defined parameters: (Update these values as necessary)

RESIZE_FACTOR = 3
# Minimum size for a contour to be considered anything
# MIN_AREA = 5000
min_area = 400

# Minimum size for a contour to be considered part of the track
# MIN_AREA_TRACK = 17500 // RESIZE_FACTOR * 3
# MIN_AREA_TRACK = 2000
min_area_track = 1500
# MIN_AREA_TRACK = 9500

# MAX_CONTOUR_VERTICES = 65
max_contour_vertices = 25
mark_contour_vertices = 5

# Robot's speed when following the line
# LINEAR_SPEED = 14.0
linear_speed = 60.0
linear_speed_on_curve = 30.0
linear_speed_on_loss = 30.0
speed_limit = 85.0

left_mark_buffer_count = 0

# Proportional constant to be applied on speed when turning
# (Multiplied by the error value)
# KP = 180 / 1000
# KD = 500 / 1000
# KD = 0.45
kp = 0.45
kd = 0.50                                                                                                                                                                                               
# KD = 0.50
ALPHA = 1
BETA = 0

# error when the curve starts
CURVE_ERROR_THRH = 22
LOSS_THRH = 40

FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS = 10
after_loss_count = FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS + 1


# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# frames without diff in the speed
NO_MOVEMENT_FRAMES = 3

# BGR values to filter only the selected color range
lower_bgr_values = np.array([80, 80, 80])
upper_bgr_values = np.array([255, 255, 255])

# lower_hsv_values = np.array([0, 0, 60])
# upper_hsv_values = np.array([180, 50, 255])

# RECORD_PERIOD = 3
RECORD_PERIOD = 3
OUTPUT_FOLDER = "../outputs"

track_size = 2812
STOP_DISTANCE_FACTOR = 0.82

############################ CALLBACKS ############################
###################################################################

def load_file_callback(filename):
    global kd
    global kp

    global linear_speed
    global linear_speed_on_curve
    global linear_speed_on_loss
    global speed_limit

    global lower_bgr_values
    global upper_bgr_values

    global mark_contour_vertices
    global max_contour_vertices
    global min_area
    global min_area_track

    global track_size

    with open(filename, "r") as f:
    
        json_dict = json.load(f)
        print("LOADED")
        print(json_dict)

        kd = json_dict["KD"]
        kp = json_dict["KP"]
        linear_speed = json_dict["LINEAR_SPEED"]
        linear_speed_on_curve = json_dict["LINEAR_SPEED_ON_CURVE"]
        linear_speed_on_loss = json_dict["LINEAR_SPEED_ON_LOSS"]
        speed_limit = json_dict["SPEED_LIMIT"]
        lower_bgr_values = np.array(json_dict["lower_bgr_values"])
        upper_bgr_values = np.array(json_dict["upper_bgr_values"])
        max_contour_vertices = json_dict["MAX_CONTOUR_VERTICES"]
        mark_contour_vertices = json_dict["MARK_CONTOUR_VERTICES"]
        min_area_track = json_dict["MIN_AREA_TRACK"]
        min_area = json_dict["MIN_AREA"]
        track_size = json_dict["TRACK_SIZE"]

def show_callback():
    global should_show
    global ip_addr
    should_show = True
    ip_addr = args.output
    print("SHOWING")
    print(">>", end="")


def record_callback():
    global should_record
    global record_frames
    should_record = True
    record_frames = []
    print("RECORDING")
    print(">>", end="")


def end_record():
    try:
        global should_record
        global record_frames
        global shape
        if should_record:
            writer = cv2.VideoWriter(
                f"{OUTPUT_FOLDER}/pov-{datetime.now().minute}.mp4",
                cv2.VideoWriter_fourcc(*"mp4v"),
                30,
                shape,
            )
            print(len(record_frames))
            for frame in record_frames:
                writer.write(frame)
            writer.release()
            print("Finished recording")
    except KeyboardInterrupt:
        pass

def save_map():
    global track_map
    global should_map
    if should_map:
        track_map.to_file(f"map-{datetime.now().minute}.json")

def write_callback():
    global should_write
    global csv_writer
    global csv_file
    should_write = True
    csv_file = open(f"{OUTPUT_FOLDER}/values-{datetime.now().minute}.csv", "w")
    csv_writer = csv.writer(csv_file)
    header = [
        "timestamp",
        "resultante",
        "distancia direita",
        "distancia esquerda",
        "linear",
        "angular",
        "erro",
    ]
    # write the header
    csv_writer.writerow(header)


def end_write():
    global should_write
    global csv_file
    if should_write:
        csv_file.close()
        print("Finished writing")


def stop_callback():
    global should_stop
    global runtime
    runtime = int(args.stop)
    runtime = timedelta(seconds=runtime)

    should_stop = True
    print("WILL STOP")
    print(">>", end="")


def stop_for_distance_callback():
    global should_stop_for_distance
    global track_length
    track_length = int(args.distance)

    should_stop_for_distance = True
    print("WILL STOP")
    print(">>", end="")


def stop_for_mark_callback():
    global should_stop_for_mark
    should_stop_for_mark = True

    print("WILL STOP")
    print(">>", end="")


def map_callback(filename=None):
    global should_map
    global track_map
    global encoder_ml
    global encoder_mr

    global linear_speed
    global speed_limit 

    should_map = True
    track_map = TrackMap(
        encoder_mr, encoder_ml, A, B, linear_speed, speed_limit, STATIC_COEFICIENT, AXIS_DISTANCE
    )
    if not filename is None:
        track_map.from_file(filename)

    print("WILL MAP")
    print(">>", end="")


def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    print("STARTING!")
    global should_move
    global right_mark_count
    global finalization_countdown
    lost = False
    should_move = True
    # right_mark_count = 0
    # finalization_countdown = None

    print(">>", end="")
    return response


def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    print("STOPPED!")
    global should_move
    # global finalization_countdown
    should_move = False
    # finalization_countdown = None

    print(">>", end="")
    return response


###################################################################


def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
    Width_left_boundary, Width_right_boundary)
    """
    ## Update these parameters as well.

    # return (3*height//5, height, 0, width)
    # return (2*height//5, 3*height//5, 0, width)
    # return (4*height//5, height, 0, width)
    return (2 * height // 6, height, 0, width)
    # return (2 * height // 6, height, 1 * width // 6, 5 * width // 6)


def get_contour_data(mask, out, previous_pos):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line)
    and return the side in which the smaller contour is (the track mark)
    (If there are any of these contours),
    and draw all contours on 'out' image
    """

    global lower_bgr_values
    global upper_bgr_values

    global max_contour_vertices
    global min_area
    global min_area_track

    # erode image (filter excessive brightness noise)
    kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.erode(mask, kernel, iterations=1)

    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    over = False
    tried_once = False

    possible_tracks = []

    x = None

    height, width, _ = out.shape

    while not over:
        for contour in contours:
            line = {}
            M = cv2.moments(contour)
            # Search more about Image Moments on Wikipedia :)

            contour_vertices = len(cv2.approxPolyDP(contour, 1.5, True))
            # print("vertices: ", contour_vertices)

            if M["m00"] < min_area:
                continue

            # Contour is part of the track
            line["x"] = crop_w_start + int(M["m10"] / M["m00"])
            line["y"] = int(M["m01"] / M["m00"])

            line["valid"] = False
            if (contour_vertices < max_contour_vertices) and (
                M["m00"] > min_area_track
            ):
            # IDEIA: filter using bouding rectangle

                line["valid"] = True
                if not line in possible_tracks:
                    possible_tracks.append(line)
                    # plot the area in BLUE
                    cv2.drawContours(out, contour, -1, (255, 0, 0), 2)
                    cv2.putText(
                        out,
                        f"{np.linalg.norm(line['x'] - previous_pos)}",
                        
                        (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN,
                        2 / (RESIZE_FACTOR / 3),
                        (255, 0, 255),
                        1,
                    ) 
                    # cv2.circle(out, (x, y), 3, (45, 50, 255), 5)



            # plot the amount of vertices in light blue
            cv2.drawContours(out, contour, -1, (255, 255, 0), 2)
            # cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
            #     cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (100,200,150), 1)

            cv2.putText(
                out,
                str(contour_vertices),
                (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                cv2.FONT_HERSHEY_PLAIN,
                2 / (RESIZE_FACTOR / 3),
                (100, 200, 150),
                1,
            )

            vx, vy, x1, y1 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            y2 = y1 + vy
            x2 = x1 + vx

            y = line["y"] - 20
            if (x2 == x1) or (y2 == y1):
                x = width//2
            else:
                x = int(x1 + (y - y1) / ((y2 - y1) / (x2 - x1)))
            if x < 0 or x > width:
                x = line["x"]

            # print(vy, y, y1)

            # check if contour is a crossing
            cx, cy, cw, ch = cv2.boundingRect(contour)
            line["is_crossing"] = cw >= (width - 10) and ch >= (height - 10) and cx <= 10 and cy <= 10

            line["area"] = M["m00"]
            line["len"] = contour_vertices
            line["expected_x"] = x

            # print(f"circle at {x, y}")
            # is blur or something not relevant
            if not line["valid"]:
                # plot the area in pink
                cv2.drawContours(out, contour, -1, (255, 0, 255), 2)
                cv2.putText(
                    out,
                    f"{contour_vertices}-{M['m00']}",
                    (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN,
                    2 / (RESIZE_FACTOR / 3),
                    (255, 0, 255),
                    1,
                )

        if possible_tracks:
            over = True

        # Did not find the line. Try eroding more?
        elif not tried_once:
            mask = cv2.erode(mask, kernel, iterations=1)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
            )
            tried_once = True

        # Did not find anything
        else:
            over = True

    if not possible_tracks:
        # chosen_line = {"valid": False}
        chosen_line = {}
    else:
        chosen_line = min(
            possible_tracks, key=lambda line: np.linalg.norm(line["expected_x"] - previous_pos)
        )
        # #debug
        # if len(possible_tracks) > 1:
        #     print(f"Previous - {previous_pos}")
        #     for p in possible_tracks:
        #         if p == chosen_line:
        #             print("*", end="")
        #         print(f"    p - {line['expected_x']} ({line['expected_x'] - previous_pos})")
        # #enddebug


        cv2.circle(out, (chosen_line["expected_x"], chosen_line["y"] - 20), 3, (45, 255, 255), 5)
    
    return chosen_line

def check_stop_mark(mask, out):

    global mark_contour_vertices
    global should_stop_for_mark
    global saw_right_mark
    if should_stop_for_mark:
        right_marks, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for mark in right_marks: 
            
            M = cv2.moments(mark)
            # Search more about Image Moments on Wikipedia :)

            contour_vertices = len(cv2.approxPolyDP(mark, 1.5, True))
            # print("vertices: ", contour_vertices):

            if (min_area < M["m00"] < min_area_track) and contour_vertices < mark_contour_vertices:
                # plot the area in purple
                cv2.drawContours(out, mark, -1, (255, 0, 255), 2)
                cv2.putText(
                    out,
                    f"{contour_vertices}-{M['m00']}",
                    (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN,
                    2 / (RESIZE_FACTOR / 3),
                    (0, 0, 255),
                    1,
                )
                print("SAW STOP MARK")
                saw_right_mark = True

def check_curve_mark(mask, out):
    
    global min_area
    global min_area_track

    global should_map
    global should_use_map

    global mark_contour_vertices
    global left_mark_buffer_count
    
    global track_map 

    left_marks, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    saw_left_mark = False
    for mark in left_marks: 
        
        M = cv2.moments(mark)
        # Search more about Image Moments on Wikipedia :)

        contour_vertices = len(cv2.approxPolyDP(mark, 1.5, True))
        # print("vertices: ", contour_vertices):

        if (min_area < M["m00"] < min_area_track) and contour_vertices < mark_contour_vertices:
            # plot the area in purple
            cv2.drawContours(out, mark, -1, (0, 255, 255), 2)
            cv2.putText(
                out,
                f"{contour_vertices}-{M['m00']}",
                (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                cv2.FONT_HERSHEY_PLAIN,
                2 / (RESIZE_FACTOR / 3),
                (0, 0, 255),
                1,
            )

            # if contour is a right mark append to map
            if should_map and left_mark_buffer_count == 0:
                track_map.append_map()
                print("SAW LEFT MARK")
                saw_left_mark = True

            if should_use_map and left_mark_buffer_count == 0:
                track_map.mark_count += 1
                print("SAW LEFT MARK")
                saw_left_mark = True

            # saw a right mark recently
            if left_mark_buffer_count < 5:
                left_mark_buffer_count = 5

    if not saw_left_mark and left_mark_buffer_count > 0:
        left_mark_buffer_count -= 1    

zeros = None
started_check_stop_mark = False
recently_ignored_crossing = False

def process_frame(image_input, last_res_v):
    """
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    debug_str2 = ""
    global angular
    global error
    global error_deriv
    global last_error
    global last_error_deriv
    global image_ts
    global last_image_ts
    global just_seen_line
    global left_mark_buffer_count

    global should_move
    global should_stop
    global should_stop_for_distance
    global should_map
    global should_use_map

    global track_length
    global init_time
    global lost
    global no_movement_count
    global after_loss_count
    global track_map
    global total_distance

    global encoder_ml
    global encoder_mr

    global frame_count

    global kd
    global kp

    global linear_speed
    global linear_speed_on_curve
    global linear_speed_on_loss

    global lower_bgr_values
    global upper_bgr_values

    global max_contour_vertices
    global min_area
    global min_area_track

    frame_count += 1

    height, width, _ = image_input.shape
    image = image_input

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    cx = width // 2

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]

    red = crop[:, :, 2]

    # hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _,mask = cv2.threshold(red,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    # mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)
    # print(lower_bgr_values)
    # print(upper_bgr_values)
    # mask = cv2.inRange(crop, lower_hsv_values, upper_hsv_values)

    output = image 

    mark_mask = None
    ch, cw, _ = crop.shape

    global should_stop_for_mark
    global started_check_stop_mark
    global zeros
    global track_size
    if zeros is None:
        # zeros = np.zeros([ch, 2*cw//3, 3], dtype=np.uint8)
        zeros = np.zeros_like(crop[:, 2*cw//3:], dtype=np.uint8)

    if should_stop_for_mark and (((encoder_ml.distance + encoder_mr.distance) / 2) >= STOP_DISTANCE_FACTOR*track_size):
        # mark_mask = cv2.inRange(crop[:, 2*cw//3:], lower_bgr_values, upper_bgr_values)
        mark_mask = mask[:, 2*cw//3:]
        check_stop_mark(mark_mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
        if not started_check_stop_mark:
            print("IS CHECKING STOP MARK")
            started_check_stop_mark = True

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on th e cropped part of the output image
    line = get_contour_data(
        mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop], error + cx
    )
    # also get the side in which the track mark "is"
    
    global recently_ignored_crossing
    curve_mark_mask = mask[:, :1*cw//3]
    if should_map or should_use_map:
        if not line.get("is_crossing"):
            check_curve_mark(curve_mark_mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
            recently_ignored_crossing = False
        elif not recently_ignored_crossing:
            print("IGNORED CROSSING")
            recently_ignored_crossing = True 

    x = None

    if line.get("valid"):
        x = line["x"] if line["is_crossing"] else line["expected_x"]
        new_error = ALPHA * cx - x
        new_error_deriv = BETA * cx - x
    else:
        new_error = None
        new_error_deriv = None

    if line.get("valid"):
        # if ((not lost) or (abs(new_error - error) < LOSS_THRH)): # robot is following the line, there IS some error, but not that much
        # error:= The difference between the center of the image and the center of the line
        last_error = error
        error = new_error
        last_error_deriv = error_deriv
        error_deriv = new_error_deriv

        # if lost:
        #     after_loss_count = 0

        lost = False

        # if should_use_map:
        #     global track_map
        #     linear = track_map.track_map[mark_count].pwm_speed

        if abs(error) > CURVE_ERROR_THRH:
            linear = linear_speed_on_curve
        else:
            if total_distance > 2000:
                linear = speed_limit
            else:
                linear = linear_speed

        # if after_loss_count < FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS:
        #     linear = LINEAR_SPEED_ON_LOSS
        #     after_loss_count += 1

        just_seen_line = True
        # error = new_error
        P = float(error) * kp
        D = (float(error_deriv - last_error_deriv) * kd) # / (
        #     (image_ts - last_image_ts) / 1e7
        # )

        angular = P + D

    else:
        debug_str2 = f"last={error}| new={new_error}"
        lost = True
        if just_seen_line:
            just_seen_line = False
            error *= LOSS_FACTOR
            error_deriv *= LOSS_FACTOR
            angular *= LOSS_FACTOR

        linear = linear_speed_on_loss

    global runtime

    # Check for final countdown
    if should_move and should_stop:
        if datetime.now() >= runtime:
            should_move = False
            print(f"STOPPED AT {datetime.now()}")

    # check for distance
    elif should_move and should_stop_for_distance:
        if (total_distance) >= track_length:
            should_move = False
            print(f"STOPPED AT {total_distance} centimetres.")

    # check line sensor
    global saw_right_mark
    if not should_stop and should_move and should_stop_for_mark:
        if saw_right_mark:
            should_stop = True
            runtime = datetime.now() + timedelta(milliseconds=300)
            print(f"READ STOP MARK")
    
    # Determine the speed to turn and get the line in the center of the camera.
    # angular = float(error) * -KP
    # resulting speed
    res_v = {
        "left": int(linear - angular),  # left motor resulting speed
        "right": int(linear + angular),  # right motor resulting speed
        "linear": linear,
    }

    if should_move:
        motor_left.run(res_v["left"])
        motor_right.run(res_v["right"])

    else:
        motor_left.stop()
        motor_right.stop()

    # Show the output image to the user
    global should_record
    global should_show
    global should_write
    global ip_addr
    global record_frames
    global csv_writer

    # now = f"{datetime.now().strftime('%M:%S.%f')[:-4]}"
    debug_str = f"A: {int(angular)}|L: {linear}|E: {error}|"
    debug_str += f"lft{res_v['left']}|rgt{res_v['right']}|"
    debug_str3 = f"cVert:{line['len']}|cArea:{line['area']}" if line else ""

    if should_record or should_show:
        text_size, _ = cv2.getTextSize(debug_str, cv2.FONT_HERSHEY_PLAIN, 1, 1)
        text_w, text_h = text_size

        # cv2.rectangle(output, (0, 90), (text_w, 110 + text_h), (255,255,255), -1) <- mysterious white line
        # Plot the boundaries where the image was cropped
        cv2.rectangle(
            output,
            (crop_w_start, crop_h_start),
            (crop_w_stop, crop_h_stop),
            (0, 0, 255),
            2,
        )

        # center of the image
        cv2.circle(output, (cx, crop_h_start + (height // 2)), 1, (75, 0, 130), 1)
        # cv2.putText(output, now, (0, text_h - 10), cv2.FONT_HERSHEY_PLAIN, 0.5, (50, 255, 255), 1)
        cv2.putText(
            output,
            debug_str,
            (0, text_h),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (0, 255, 100),
            1,
        )
        cv2.putText(
            output,
            debug_str2,
            (0, text_h + 15),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (0, 255, 100),
            1,
        )
        cv2.putText(
            output,
            debug_str3,
            (0, text_h + 30),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (0, 255, 100),
            1,
        )

        # plot the rectangle around contour center
        if x:
            cv2.circle(output, (line["x"], crop_h_start + line["y"]), 1, (0, 255, 0), 1)
            cv2.rectangle(
                output,
                (x - width, crop_h_start),
                (x + width, crop_h_stop),
                (0, 0, 255),
                2,
            )

        out_mask = np.zeros_like(output)
        out_mask[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop] = cv2.cvtColor(
            mask, cv2.COLOR_GRAY2BGR
        )
        # output_frame = np.append(output, out_mask, axis=1)
        if mark_mask is None:
            mark_mask = zeros
        else:
            mark_mask = cv2.cvtColor(mark_mask, cv2.COLOR_GRAY2BGR)

        curve_mark_mask = cv2.cvtColor(curve_mark_mask, cv2.COLOR_GRAY2BGR)

        height_diff = out_mask.shape[0] - mark_mask.shape[0]
        top_padding = abs(height_diff) // 2
        bottom_padding = abs(height_diff) - top_padding
        # Use cv2.copyMakeBorder to add the padding to 'mark_mask'
        mark_mask_padded = cv2.copyMakeBorder(mark_mask, top_padding, bottom_padding, 0, 0, cv2.BORDER_CONSTANT, value=(255,0,0))

        height_diff = out_mask.shape[0] - curve_mark_mask.shape[0]
        top_padding = abs(height_diff) // 2
        bottom_padding = abs(height_diff) - top_padding
        # Use cv2.copyMakeBorder to add the padding to 'mark_mask'
        curve_mark_mask_padded = cv2.copyMakeBorder(curve_mark_mask, top_padding, bottom_padding, 0, 0, cv2.BORDER_CONSTANT, value=(0,255,0))
        output_frame = np.hstack((curve_mark_mask_padded, mark_mask_padded, output, out_mask))

        global shape
        out_h, out_w, _ = output_frame.shape
        shape = (out_w, out_h)

        if should_show:
            _, imdata = cv2.imencode(".jpg", output_frame)
            requests.put(
                f"http://{ip_addr}:5000/upload", data=imdata.tobytes()
            )  # send image to webserver

        if should_record and (frame_count % RECORD_PERIOD) == 0:
            record_frames.append(output_frame)

    total_distance = (encoder_mr.distance + encoder_ml.distance) / 2
    if should_write:
        data = [
            datetime.now().second,
            round(total_distance, 3),
            round(encoder_mr.distance, 3),
            round(encoder_ml.distance, 3),
            linear,
            angular,
            error,
        ]
        csv_writer.writerow(data)

    return res_v  # return speed of the current iteration


def main():
    global lost
    global error
    global no_movement_count
    global csv_writer
    global shape
    global image_ts
    global last_image_ts

    lost = False

    print(datetime.now())

    # set camera captura settings
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FPS, 90)
    # video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # video.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

    retval, image = video.read()
    image_ts = time.time_ns()
    last_image_ts = 0

    # set resize settings
    height, width, _ = image.shape
    # shape = (width, height)
    # error = width // (RESIZE_SIZE * 2)
    error = 0
    print(image.shape)
    print(">>", end="")

    # check output files dir
    if not exists("outputs"):
        makedirs("outputs")

    if args.start:  # should start following line
        start_follower_callback(None, None)

    if args.record:  # should record image
        record_callback()

    if args.output != None:
        show_callback()
        # thread_stream.start()

    if args.write:  # should write values to csv
        write_callback()

    if args.stop != None:  # should stop
        stop_callback()

    if args.distance != None:  # should stop
        stop_for_distance_callback()

    if args.linestop:  # should stop
        stop_for_mark_callback()

    ##############################
    # HIGHLY EXPERIMENTAL CODE
    # NEED TESTING
    if args.map:
        map_callback()

    if args.usemap != None:
        map_callback(args.usemap)
    ##############################

    if args.file != None:
        load_file_callback(args.file)

    last_res_v = {"left": 0, "right": 0}

    fps_count = 0
    ts = time.time()

    calib_cam = pickle.load(open("calib_cam.pkl", "rb"))
    mtx = calib_cam["mtx"]
    dist = calib_cam["dist"]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        mtx, dist, (width, height), 1, (width, height)
    )

    while retval:
        try:
            image = cv2.resize(
                image,
                (width // RESIZE_FACTOR, height // RESIZE_FACTOR),
                interpolation=cv2.INTER_CUBIC,
            )
            # corrected = cv2.undistort(image, mtx, dist, None, newcameramtx)
            scaled = cv2.convertScaleAbs(image, alpha=1.5, beta=10)
            # hsv_img = cv2.cvtColor(scaled, cv2.COLOR_BGR2HSV)

            last_res_v = process_frame(scaled, last_res_v)
            retval, image = video.read()
            last_image_ts = image_ts
            image_ts = time.time_ns()

            fps_count += 1
            if time.time() - ts >= 1:
                print(f"FPS: {fps_count}")
                fps_count = 0
                ts = time.time()

        except TimeoutError:
            pass

    print("Exiting...")


try:
    main()

except KeyboardInterrupt:
    now = datetime.now()
    print(now)
    print(f"TOTAL TIME {now - init_time_iso}")
    print(f"TOTAL distance {total_distance}")
    print("\nExiting...")

# except Exception as e:
#     print(e)

finally:
    del motor_right
    del motor_left
    end_write()
    end_record()
    save_map()
    print("Ended successfully.")
    GPIO.remove_event_detect(encoder_a_ml)
    GPIO.remove_event_detect(encoder_a_mr)
    GPIO.cleanup()



