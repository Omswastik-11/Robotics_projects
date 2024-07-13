
import gym
import LaRoboLiga24
import os
import cv2
import time as t
import numpy as np
import pybullet_workshop_23
import math

CAR_LOCATION = [0,0,1.5]
FRAME_WIDTH = 400
BALLS_LOCATION = dict({
    'red': [7, 4, 1.5],
    'blue': [2, -6, 1.5],
    'yellow': [-6, -3, 1.5],
    'maroon': [-5, 9, 1.5]
})
BALLS_LOCATION_BONOUS = dict({
    'red': [9, 10, 1.5],
    'blue': [10, -8, 1.5],
    'yellow': [-10, 10, 1.5],
    'maroon': [-10, -9, 1.5]
})
BALLS_LOCATION_BONUS = {
    'red': [9, 10, 1.5],
    'blue': [10, -8, 1.5],
    'yellow': [-10, 10, 1.5],
    'maroon': [-10, -9, 1.5]
}
HUMANOIDS_LOCATION = dict({
    'red': [11, 1.5, 1],
    'blue': [-11, -1.5, 1],
    'yellow': [-1.5, 11, 1],
    'maroon': [-1.5, -11, 1]
})

VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 13,
    'cam_yaw'        : 0,
    'cam_pitch'      : -110,
    'cam_target_pos' : [0,4,0]
})


os.chdir(os.path.dirname(os.getcwd()))
env = gym.make('LaRoboLiga24',
    arena = "arena2",
    car_location=CAR_LOCATION,
    ball_location=BALLS_LOCATION_BONUS,  # toggle this to BALLS_LOCATION_BONOUS to load bonous arena
    humanoid_location=HUMANOIDS_LOCATION,
    visual_cam_settings=VISUAL_CAM_SETTINGS
)

#### Functions :: ---
def wait(time=1):
    t.sleep(time)

def open():
    env.open_grip()

def stop(time=1):
    wait(time)
    env.move(vels=[[0, 0], [0, 0]])

def close():
    env.close_grip()

def shoot(hit=100):
    env.shoot(hit)

def move(mode='f', speed=5):
    if mode.lower() == "f":
        mat = [[speed, speed], [speed, speed]]
    elif mode.lower() == "r":
        mat = [[speed, -speed], [speed, -speed]]
    elif mode.lower() == "ro":
        mat = [[-speed, speed], [-speed, speed]]
    elif mode.lower() == "l":
        mat = [[-speed, speed], [-speed, speed]]
    elif mode.lower() == "b":
        mat = [[-speed, -speed], [-speed, -speed]]
    else:
        print("Error Occurred, Unexpected mode in Move Function.")
        return "Error"
    env.move(vels=mat)

def isBall(cnt):
    # Ensure the contour is not empty and has enough points to form a circle
    if cnt is not None and len(cnt) > 4:
        # Convert contour to the correct type if it's not already
        cnt = np.array(cnt, dtype=np.float32)
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter != 0 else 0
        return True if (0.48 < circularity < 6.5) and (area > 50) else False
    else:
        return False

def detect_balls(img):
    color_ranges = {
        'red': ([0, 0, 100], [10, 10, 255]),
        'blue': ([100, 0, 0], [255, 100, 50]),  # Adjusted blue range
        'yellow': ([10, 100, 100], [30, 255, 255]),  # Adjusted yellow range
        'maroon': ([40, 0, 40], [105, 10, 200]),  # Add any other colors if needed
    }
    
    ball_contours = {}
    ball_centers = {}
    ball_color = 'unknown'  # Default value

    for color, (lower, upper) in color_ranges.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(img, lower, upper)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in cnts:  
            if len(cnt) > 4:
                if isBall(cnt):
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        ball_contours[color] = cnt
                        ball_centers[color] = {'contour': cnt, 'center': center}
                        ball_color = color
                        cv2.drawContours(img, [cnt], -1, (255, 255, 255), 2)
                        break

    return img, ball_contours, ball_centers, ball_color

def is_goal_post(contour, min_area_threshold, max_circularity_threshold):
    contour_np = np.asarray(contour)
    # Check if the contour is a valid numpy array
    if contour_np is not None and len(contour_np.shape) == 3:
        area = cv2.contourArea(contour_np)
        if area < min_area_threshold:
            return False
        epsilon = 0.0132999 * cv2.arcLength(contour_np, True)
        approx = cv2.approxPolyDP(contour_np, epsilon, True)
        num_vertices = len(approx)

        if 5.5 < num_vertices <= 25:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)
            # Check if the contour is approximately rectangular with a specific aspect ratio
            if 0.075 < aspect_ratio < 13.9:
                # Calculate circularity
                circularity = 4 * math.pi * area / (cv2.arcLength(contour_np, True) ** 2)
                # Check if circularity is below a certain threshold
                if circularity < max_circularity_threshold:
                    return True

    # If the contour doesn't meet the criteria, consider it as not a goal post
    return False

def detect_goal_post_and_draw(img):
    goal_post_color_ranges = {
        'red': ([0, 0, 100],[5, 45, 255]),
        'blue': ([40, 0, 0],[255, 100, 50]),  
        'yellow': ([0, 140, 135],[0, 242, 255]),  
        'maroon': ([40, 0, 40],[105, 10, 200]),
    }
    goal_post_contours = {}
    goal_post_color = 'unknown'

    for color, (lower, upper) in goal_post_color_ranges.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(img, lower, upper)
        blurred_mask = cv2.GaussianBlur(mask, (9,9), 10)
        cnts, _ = cv2.findContours(blurred_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = [c for c in cnts if is_goal_post(c, min_area_threshold=100, max_circularity_threshold=0.230)]

        if valid_contours:
            # Use the largest valid contour
            largest_contour = max(valid_contours, key=lambda x: cv2.contourArea(x))
            goal_post_contours[color] = largest_contour
            goal_post_color = color
            # Draw a thicker contour border
            cv2.drawContours(img, [largest_contour], -1, (0, 255, 0), 2)  # Green color in BGR
            break  # Assuming one goal post per color

    return img, goal_post_contours, goal_post_color

def get_center(contour):
    """
    Calculate the center point of a contour.

    :param contour: The contour of which to find the center.
    :return: The center point (x, y) or None if the contour is invalid.
    """
    if contour is not None and isinstance(contour, np.ndarray) and len(contour.shape) == 3:
        area = cv2.contourArea(contour)
        if area > 0:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return (cX, cY)
    return None


def is_arms_within_goal_post(arm_contours, goal_post_rect):
    """
    Check if the arms position is within the rectangular contour of the goal post.

    :param arm_contours: Contours of the robot's arms.
    :param goal_post_rect: Tuple representing the rectangular contour (x, y, width, height) of the goal post.
    :return: True if arms are within the goal post, False otherwise.
    """
    if arm_contours is not None and len(arm_contours) > 0:
        for contour in arm_contours:
            # Calculate the bounding box of the current arm contour
            x, y, w, h = cv2.boundingRect(contour)
            arm_center_x = x + w // 2
            arm_center_y = y + h // 2
            rect_x, rect_y, rect_w, rect_h = goal_post_rect
            center = get_center(contour)
            # Check if the center is not None before attempting to unpack
            if center is not None:
                (Cx, Cy) = center
                # Check if the center of the arm is within the goal post rectangle
                if (rect_x/2 < arm_center_x < (rect_x + rect_w)*2) and (rect_y < arm_center_y < rect_y + rect_h):
                    return True  # At least one arm is within the goal post

    return False  # No arms are within the goal post
   
def get_center_of_goal_post(goal_post_contours):
    """
    Calculate the average center point of the goal post contours.

    :param goal_post_contours: Dictionary containing goal post contours for different colors.
    :return: The average center point (x, y) or None if the contours are invalid.
    """
    all_goal_post_centers = []

    for color, contour in goal_post_contours.items():
        # Convert the contour to a numpy array
        contour_np = np.asarray(contour)

        # Check if the contour is a valid numpy array
        if contour_np is not None and len(contour_np.shape) == 3:
            # Calculate the center point of the bounding rectangle
            x, y, w, h = cv2.boundingRect(contour_np)
            cX = x + w // 2
            cY = y + h // 2
            all_goal_post_centers.append((cX, cY))

    if all_goal_post_centers:
        # Calculate the average center point
        avg_cX = sum(center[0] for center in all_goal_post_centers) / len(all_goal_post_centers)
        avg_cY = sum(center[1] for center in all_goal_post_centers) / len(all_goal_post_centers)
        return (avg_cX, avg_cY)

    return None

def get_center_of_bounding_rect(contours):
    """
    Calculate the average center point of the bounding rectangles of multiple contours.

    :param contours: List of contours for which to find the bounding rectangle centers.
    :return: The average center point (x, y) or None if the contours are invalid.
    """
    if contours is not None and len(contours) > 0:
        total_cX = 0
        total_cY = 0
        count = 0

        for contour in contours:
            # Ensure contour is a numpy array
            if not isinstance(contour, np.ndarray):
                contour = np.array(contour)
            # Check if the contour has a reasonable number of dimensions
            if len(contour.shape) == 3:
                # Find the convex hull of the contour
                hull = cv2.convexHull(contour)
                # Find the bounding rectangle of the convex hull
                x, y, w, h = cv2.boundingRect(hull)
                if w > 0 and h > 0:
                    cX = x + w // 2
                    cY = y + h // 2
                    total_cX += cX
                    total_cY += cY
                    count += 1
        if count > 0:
            avg_cX = total_cX / count
            avg_cY = total_cY / count
            return (avg_cX, avg_cY)
    return None

def pid_controller(error):
    global integral, prev_error
    kp = 0.061  # Proportional gain
    ki = 0.071  # Integral gain
    kd = 0.0031  # Derivative gain
    # Initialize PID controller variables
    prev_error = 0
    integral = 0
    integral += error
    derivative = error - prev_error
    output = kp * error + ki * integral + kd * derivative
    prev_error = error
    return output

def detect_and_grab_ball(frame):
    """
    Detects the robot's arms and the ball, and issues commands to grab the ball when it is between the arms.

    Parameters:
    frame (numpy.ndarray): The current frame from the robot's camera.

    Returns:
    Tuple[bool, str]: Tuple containing a boolean indicating if the ball is successfully detected between the arms
                     and the color of the grabbed ball (if any).
    """
    global Holding
    # HSV color ranges for the robot's arms
    arm_color_ranges = {
        'arm_color': ([0, 0, 13], [0, 0, 95]),
    }
    grabbed_ball_color = 'unknown'
    for arm_color_ranges, (lower, upper) in arm_color_ranges.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    arm_mask = cv2.inRange(hsv, lower, upper)
    arm_contours, _ = cv2.findContours(arm_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, arm_contours, -1, (0, 165, 255), 2)
    # Assume the largest contour is the arm
    arm = max(arm_contours, key=cv2.contourArea, default=None)
    # Detect balls using the existing function
    _, ball_contours, ball_centers, ball_color = detect_balls(frame)
    if arm is not None and ball_centers and not Holding:
        # Calculate the bounding box of the arms
        arm_x, arm_y, arm_w, arm_h = cv2.boundingRect(arm)
        arm_center = get_center_of_bounding_rect(arm_contours)
        # Iterate over detected balls
        if ball_color != shooted_balls[0]and ball_color != shooted_balls[1] and ball_color != shooted_balls[2]and ball_color != shooted_balls[3]:
            for ball_color, ball_data in ball_centers.items():
                cnt = ball_data['contour']
                area = cv2.contourArea(cnt)
                # print(area)
                grabbed_ball_color = ball_color
                open()
                error = arm_center[0] - ball_data['center'][0]
                # Check if the ball is between the arms
                if not (-50 < error < 50):
                    contour_np = np.asarray(ball_contours)
                # Check if the contour is a valid numpy array
                    if contour_np is not None and len(contour_np.shape) > 0:
                        # Calculate the area of the goal post contour
                        area = cv2.contourArea(contour_np)
                        if area < 105100:
                            move ("f",5)
                else :    
                    move("ro",-5.5) # for bonous task
                    t.sleep(0.0055151)# for bonous task
                    move("f", 6.35)
                    if area > 12000: 
                        open()
                        move("f",5.25)
                        # t.sleep(1.75)
                        t.sleep(2.15) # for bonus task
                        close()
                        move("f",-7.5)
                        #t.sleep(2.5) 
                        t.sleep(5.55)# for bonous task
                        grabbed_ball_color = ball_color
                        Holding = True
                        return True, grabbed_ball_color, arm_contours
                    elif area> 100:
                            move("f",5)

    return False, grabbed_ball_color,arm_contours

Kp = 0.0107
Ki = 0.000369
Kd = 0.00071
prev_error = 0
integral = 0
def calculate_pid(error):
    global prev_error, integral
    # Proportional term
    P = Kp * error
    # Integral term
    integral += error
    I = Ki * integral
    # Derivative term
    derivative = error - prev_error
    D = Kd * derivative
    # Update previous error
    prev_error = error
    # Calculate total PID output
    output = P + I + D
    return output

def move_towards_goal(goal_post_contours, img,grabbed_ball_color):
    """
    Move the robot towards the goal post based on the contour area.

    :param goal_post_contours: Dictionary containing goal post contours for different colors.
    :param img: Image frame for detecting and grabbing arms.
    """
    global Kp, Ki, Kd,Holding,n,shooted_balls

    if goal_post_contours:
        for color, contour in goal_post_contours.items():
            # Check if the contour is valid
            if contour is not None and len(contour) > 0:
                # Convert the contour to a numpy array
                contour_np = np.asarray(contour)
                # Check if the contour is a valid numpy array
                if contour_np is not None and len(contour_np.shape) == 3:
                    # Calculate the area of the goal post contour
                    goal_post_area = cv2.contourArea(contour_np)

                    # Use the area to estimate the distance from the car (adjust as needed)
                    estimated_distance = ((Kp * goal_post_area) +(Ki * goal_post_area)+(Kd * goal_post_area))*3
                    # Calculate PID control
                    pid_output = calculate_pid(estimated_distance)
                    # Determine wheel velocities based on the PID output
                    wheel_speed_left = pid_output 
                    wheel_speed_right = pid_output 
                    # Control the robot's movement using the tire velocities
                    env.move(vels=[[wheel_speed_left, wheel_speed_right], [wheel_speed_left, wheel_speed_right]])
                    # Check if the goal post area is greater than a specific value
                    if goal_post_area > 3740:  # Adjust the value as needed
                        # Stop the robot
                        move("ro", 0)
                        # Check whether the two arms are within the goal post area
                        _, _, arm_contours = detect_and_grab_ball(img)
                        if arm_contours:
                            env.move(vels=[[0, 0], [0, 0]])  
                            arm_center = get_center_of_bounding_rect(arm_contours)
                            goal_post_center = get_center_of_goal_post(goal_post_contours)
                            if arm_center is not None and goal_post_center is not None:
                                    # Calculate the error (distance) between the arm center and goal post center
                                        error = goal_post_center[0] - arm_center[0]
                                    # If the arms are close to the center or slightly to the right side of the goal
                                        if not -20 < error < 25 :
                                            if error <0 :
                                                move("ro",2.75)
                                            else: 
                                               move("ro",-1.75)
                                        else :
                                            stop()
                                            shoot()
                                            stop() 
                                            move("ro",4)
                                            t.sleep(1.5) # for bonous task
                                            # t.sleep(3)
                                            shooted_balls[n] = grabbed_ball_color
                                            n = n + 1
                                            Holding = False
                                            if n == 4:
                                                print("Namaste 🙏  ")
                                                print("Task completed")
                                            return True
                        else:
                                perform_shm(3)
                else:
                    print(f"Invalid numpy array contour for {color}.")
            else:
              print(f"Invalid goal post contour or no goal post detected for {color}.")
    else:
        print("No goal post contours provided.")

shm_amplitude = 5  # Maximum speed in one direction for the oscillation
shm_frequency = 0.85  # Frequency of the oscillation in Hz
shm_phase = 0  # Starting phase of the oscillation

def perform_shm( duration=5):
    global shm_amplitude, shm_frequency, shm_phase
    start_time = t.time()
    end_time = start_time + duration

    while t.time() < end_time:
        elapsed_time = t.time() - start_time
        shm_phase = 2 * math.pi * shm_frequency * elapsed_time
        speed = shm_amplitude * math.sin(shm_phase)
        mode = 'l' if speed < 0 else 'r'
        move(mode, abs(speed))
        t.sleep(0.1)
    move('f', 0)
        
# Main loop for robot behavior based on detected objects
Holding = False  # Initialize Holding variable
End = False  # Initialize End variable
humanoid = False  # Initialize humanoid
largest_area = 0
largest_ball_color = None
n =0
shooted_balls =['','','','']
while True:
    img = env.get_image(cam_height=0, dims=[400, 400])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)   
    img, ball_contours, ball_centers,ball_color = detect_balls(img)
    img , goal_post_contours,goal_post_color= detect_goal_post_and_draw(img)
    cv2.imshow("husky camera feed",img)
    if not Holding:
        move('ro', 2.05)
        ball_grabbed,grabbed_ball_color,arm_contours = detect_and_grab_ball(img)

    if ball_grabbed:
        # navigate_to_center(img)
        largest_ball_contour = max(ball_contours.values(), key=lambda x: cv2.contourArea(x), default=None)

        if largest_ball_contour is not None and cv2.contourArea(largest_ball_contour) > 0:
            img, ball_contours, ball_centers,ball_color = detect_balls(img)
            img , goal_post_contours,goal_post_color= detect_goal_post_and_draw(img)
    
            if goal_post_color!=grabbed_ball_color:
                move("ro",8)

            else:           
                    img, goal_post_contours, goal_post_color = detect_goal_post_and_draw(img)
                    if goal_post_color in goal_post_contours:
                        goal_post_area = cv2.contourArea(goal_post_contours[goal_post_color])
                        goal_post_center = get_center_of_goal_post(goal_post_contours)
                        arm_center = get_center_of_bounding_rect(arm_contours)
                        move_towards_goal(goal_post_contours, img,grabbed_ball_color)                      
        else:
            move("ro",4) 

    k = cv2.waitKey(1)
    if k == ord('q'):
            break

t.sleep(3)
cv2.destroyAllWindows()