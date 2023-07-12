#!/usr/bin/env python3
"""
A program used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
You may change the parameters BECAUSE IT IS NECESSARY.
"""
__author__ = "Gabriel Hishida, Gabriel Pontarolo, Tiago Serique, Isadora Botassari and Maite Aska"

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

# init arg parser
parser = argparse.ArgumentParser()
parser.add_argument("-s", "--start", action="store_true", help="Follow line")
parser.add_argument("-r", "--record", action="store_true", help="Record masked image")
parser.add_argument("-w", "--write", action="store_true", help="Write encoder values to csv")
parser.add_argument("-o","--output",metavar="address",action="store",help="Show output image to ip address")
parser.add_argument("-p", "--stop", metavar="store_true", help="Stop the robot in `RUNTIME` seconds")
parser.add_argument("-l", "--linestop", action="store_true", help="Stop by reading the side mark")
parser.add_argument("-d","--distance",metavar="store_true",help="Stop the robot in `DISTANCE` centimetres")
parser.add_argument("-m", "--map", action="store_true", help="Create a map of the track")
parser.add_argument("-um", "--usemap", metavar="map_file", help="Use map to follow the line")
args = parser.parse_args()

############################# DEFINES #############################
###################################################################

# encoder values
STEPS_NUMBER = 7 * 20
RPM = 800
RADIUS_WHEEL = 1.65  # centimeters
# RADIUS_WHEEL = 16.5 # millimeters

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
angular = 0
last_error = 0
total_distance = 0
no_movement_count = 0
just_seen_line = False
read_start_mark = False
ignore_mark_countdown = None
should_ignore_mark = False
right_mark_buffer_count = 0
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

## User-defined parameters: (Update these values as necessary)

# Minimum size for a contour to be considered anything
# MIN_AREA = 5000
MIN_AREA = 17000

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 17500
# MIN_AREA_TRACK = 9500

MAX_CONTOUR_VERTICES = 80
# MAX_CONTOUR_VERTICES = 65

# Robot's speed when following the line
# LINEAR_SPEED = 14.0
LINEAR_SPEED = 60.0
LINEAR_SPEED_ON_CURVE = 35.0
LINEAR_SPEED_ON_LOSS = 20.0

# Proportional constant to be applied on speed when turning
# (Multiplied by the error value)
# KP = 180 / 1000
# KD = 500 / 1000
KP = 0.18
KD = 0.5

# error when the curve starts
CURVE_ERROR_THRH = 22
LOSS_THRH = 40

FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS = 10
after_loss_count = FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS + 1


# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# frames without diff in the speed
NO_MOVEMENT_FRAMES = 3

# RESIZE_SIZE = 4
RESIZE_SIZE = 6

# BGR values to filter only the selected color range
lower_bgr_values = np.array([40, 40, 40])
upper_bgr_values = np.array([255, 255, 255])

RECORD_PERIOD = 3
OUTPUT_FOLDER = "../outputs"

############################ CALLBACKS ############################
###################################################################

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
    global should_record
    global record_frames
    global shape
    if should_record:
        writer = cv2.VideoWriter(
            f"{OUTPUT_FOLDER}/pov-{datetime.now().minute}.mp4",
            cv2.VideoWriter_fourcc(*"mp4v"),
            10,
            shape,
        )
        print(len(record_frames))
        for frame in record_frames:
            writer.write(frame)
        writer.release()
        print("Finished recording")


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


def map_callback():
    global should_map
    global track_map
    global encoder_ml
    global encoder_mr

    should_map = True
    track_map = TrackMap(encoder_mr, encoder_ml, 1, 1)

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


def get_contour_data(mask, out, previous_pos):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line)
    and return the side in which the smaller contour is (the track mark)
    (If there are any of these contours),
    and draw all contours on 'out' image
    """

    global should_map
    global track_map
    global right_mark_buffer_count
    global right_mark_count

    # erode image (filter excessive brightness noise)
    kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.erode(mask, kernel, iterations=1)

    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}
    over = False
    tried_once = False

    possible_tracks = []

    x = None

    height, width, _ = out.shape

    while not over:
        saw_right_mark = False
        for contour in contours:
            M = cv2.moments(contour)
            # Search more about Image Moments on Wikipedia :)

            contour_vertices = len(cv2.approxPolyDP(contour, 2.5, True))
            # print("vertices: ", contour_vertices)

            if M["m00"] < MIN_AREA:
                continue

            line["valid"] = False
            if (contour_vertices < MAX_CONTOUR_VERTICES) and (
                M["m00"] > MIN_AREA_TRACK
            ):
                line["valid"] = True

            # Contour is part of the track
            line["x"] = crop_w_start + int(M["m10"] / M["m00"])
            line["y"] = int(M["m01"] / M["m00"])

            possible_tracks.append(line)

            # plot the amount of vertices in light blue
            cv2.drawContours(out, contour, -1, (255, 255, 0), 1)
            # cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
            #     cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (100,200,150), 1)

            cv2.putText(
                out,
                str(contour_vertices),
                (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                cv2.FONT_HERSHEY_PLAIN,
                2 / (RESIZE_SIZE / 3),
                (100, 200, 150),
                1,
            )

            vx, vy, x1, y1 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            y2 = y1 + vy
            x2 = x1 + vx

            y = line["y"] - 20
            x = int(x1 + (y - y1) / ((y2 - y1) / (x2 - x1)))
            if x < 0 or x > width:
                x = line["x"]

            # print(vy, y, y1)

            # check if contour is a crossing
            cx, cy, cw, ch = cv2.boundingRect(contour)
            line["is_crossing"] = cw >= width and ch >= height and cx == 0 and cy == 0

            line["area"] = M["m00"]
            line["len"] = contour_vertices
            line["expected_x"] = x

            # print(f"circle at {x, y}")
            cv2.circle(out, (x, y), 3, (45, 50, 255), 10)

            if not line["valid"]:
                # plot the area in pink
                cv2.drawContours(out, contour, -1, (255, 0, 255), 1)
                cv2.putText(
                    out,
                    f"{contour_vertices}-{M['m00']}",
                    (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN,
                    2 / (RESIZE_SIZE / 3),
                    (255, 0, 255),
                    2,
                )

                saw_right_mark = True
                # if contour is a right mark append to map
                if should_map and right_mark_buffer_count == 0:
                    track_map.append_map()

                # saw a right mark recently
                if right_mark_buffer_count < 5:
                    right_mark_buffer_count += 1

        if not saw_right_mark:
            right_mark_buffer_count -= 1

        if line.get("valid"):
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
            possible_tracks, key=lambda line: abs(line["x"] - previous_pos)
        )

    return chosen_line

def process_frame(image_input, last_res_v):
    """
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    debug_str2 = ""
    global angular
    global error
    global last_error
    global just_seen_line
    global right_mark_buffer_count
    global should_move
    global should_stop
    global should_stop_for_distance
    global track_length
    global right_mark_count
    global init_time
    global lost
    global no_movement_count
    global after_loss_count
    global should_map
    global should_use_map
    global track_map
    global total_distance

    global encoder_ml
    global encoder_mr

    global frame_count

    frame_count += 1

    height, width, _ = image_input.shape
    image = image_input

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    cx = width // 2

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]

    # hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    line = get_contour_data(
        mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop], error + cx
    )
    # also get the side in which the track mark "is"

    x = None

    if line.get("valid"):
        x = line["x"] if line["is_crossing"] else line["expected_x"]
        new_error = x - cx
    else:
        new_error = None


    if line.get("valid"):
        # if ((not lost) or (abs(new_error - error) < LOSS_THRH)): # robot is following the line, there IS some error, but not that much
        # error:= The difference between the center of the image and the center of the line
        last_error = error
        error = new_error

        # if lost:
        #     after_loss_count = 0

        lost = False

        if abs(error) > CURVE_ERROR_THRH:
            linear = LINEAR_SPEED_ON_CURVE
        else:
            linear = LINEAR_SPEED

        # if after_loss_count < FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS:
        #     linear = LINEAR_SPEED_ON_LOSS
        #     after_loss_count += 1

        just_seen_line = True
        # error = new_error
        angular = float(error) * -KP + (error - last_error) * -KD

    else:
        # if new_error:
        debug_str2 = f"last={error}| new={new_error}"
        # print("LOST", end=". ")
        lost = True
        # There is no line in the image.
        # Turn on the spot to find it again.
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
            angular = angular * LOSS_FACTOR

        linear = LINEAR_SPEED_ON_LOSS

    global runtime

    # Check for final countdown
    if should_move and should_stop:
        if (datetime.now() - init_time_iso) >= runtime:
            should_move = False
            print(f"STOPPED AT {datetime.now()}")

    # check for distance
    elif should_move and should_stop_for_distance:
        if (total_distance) >= track_length:
            should_move = False
            print(f"STOPPED AT {total_distance} centimetres.")

    # check line sensor
    global read_start_mark
    global should_stop_for_mark
    global ignore_mark_countdown
    global should_ignore_mark
    global line_sensor_reading
    if should_move and should_stop_for_mark:
        if line_sensor_reading:
            print(error < 30, should_ignore_mark, read_start_mark)
        if (
            error < 30
            and not should_ignore_mark
            and read_start_mark
            and line_sensor_reading
        ):
            should_stop = True
            runtime = datetime.now() + timedelta(milliseconds=1500)
            print(f"READ STOP MARK")

        if not read_start_mark and line_sensor_reading:
            read_start_mark = True
            ignore_mark_countdown = datetime.now()
            print("READ START MARK")

        should_ignore_mark = not (
            read_start_mark
            and (datetime.now() - ignore_mark_countdown >= timedelta(seconds=17))
        )

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
        text_size, _ = cv2.getTextSize(debug_str, cv2.FONT_HERSHEY_PLAIN, 2, 2)
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
        output_frame = np.append(output, out_mask, axis=1)
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

    lost = False

    print(datetime.now())

    # set camera captura settings
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FPS, 90)
    # video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # video.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

    retval, image = video.read()

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

    # if args.usemap != None:
    #    load_map(args.usemap)
    ##############################

    last_res_v = {"left": 0, "right": 0}

    fps_count = 0
    ts = time.time()

    while retval:
        try:

            last_res_v = process_frame(image, last_res_v)
            retval, image = video.read()

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
    print("Ended successfully.")
    GPIO.remove_event_detect(encoder_a_ml)
    GPIO.remove_event_detect(encoder_a_mr)
    GPIO.cleanup()
