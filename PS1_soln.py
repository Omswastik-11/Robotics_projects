# make sure to press "s" just after starting the simulation to off the sun..
# make sure to press "s" just after starting the simulation to off the sun..
import gym
import os
import numpy as np
import cv2
import pybullet as p
import LaRoboLiga24
import time

# make sure to press "s" just after starting the simulation to off the sun..

CAR_LOCATION = [-25.5,0,1.5]

VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 13,
    'cam_yaw'        : 0,
    'cam_pitch'      : -110,
    'cam_target_pos' : [0,4,0]
})

os.chdir(os.path.dirname(os.getcwd()))
env = gym.make('LaRoboLiga24',
    arena = "arena1",
    car_location=CAR_LOCATION,
    visual_cam_settings=VISUAL_CAM_SETTINGS
)

# PID controller parameters
kp = 0.037
ki = 0.000095
kd = 0.000034
previous_error = 0
integral = 0


previous_detection_time = time.time()



def process_image(img):
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, track_image = cv2.threshold(blurred, 200, 210, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(track_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("thershold",track_image)
    return contours


def pid_control(error, kp, ki, kd, previous_error, integral):
    integral = integral + error
    derivative = error - previous_error
    output = kp*error + ki*integral + kd*derivative
    previous_error = error
    return output, previous_error, integral

def has_crossed_line(contours):
    global previous_detection_time
    if not contours:
        current_time = time.time()
        time_difference = current_time - previous_detection_time
        if time_difference > 1:
            previous_detection_time = current_time
            return True
        previous_detection_time = time.time() 

def calculate_error(img, contours):
    height, width = img.shape[:2]
    region_of_interest = img[height // 2:, :]

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # Calculate the deviation from the center of the region of interest
            error = cx - (width // 2)  # width // 2 corresponds to the center of the ROI
        else:
            error = 0
    else:
        error = 0

    return error
revolution_count = 0
stop_line_detected = False
# Main loop
while not stop_line_detected:
    img = env.get_image(cam_height=0, dims=[512, 512])
    contours = process_image(img)
    error = calculate_error(img, contours)

    # Use the error for control signal calculation
    control_signal, previous_error, integral = pid_control(error, kp, ki, kd, previous_error, integral)

    left_velocity = 11 - control_signal  
    right_velocity = 11 + control_signal 
    env.move(vels=[[left_velocity, right_velocity], [left_velocity, right_velocity]])

   
    if has_crossed_line(contours):
        revolution_count += 1
        if revolution_count >= 1:
            stop_line_detected = True
            env.move(vels=[[0, 0], [0, 0]])

    cv2.imshow("img", img)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break

env.close()