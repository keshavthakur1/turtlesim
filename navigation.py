
# Navigation using GNSS data
# Author : Rishav KUMAR(Mtech AI)
# Author Email ID : ai22mtech12003@iith.ac.in

import math
import os
import logging
import datetime
import socket
import time
import novatel_oem7_msgs
import numpy as np
import rospy
from novatel_oem7_msgs.msg import BESTPOS, BESTVEL, INSPVA
from std_msgs.msg import Float32, String
from sensor_msgs.msg
import Imu

# Define the MABX(vehicle control controller) IP address and port for sending data
mabx_IP = "192.168.50.1"
mabx_PORT = 30000

# Define buffer size and local interface
BUFFER_SIZE = 4096
local_interface = "eth0"

# Conversion factor for latitude and longitude to meters
LAT_LNG_TO_METER = 1.111395e5

# Initialize the ROS node for the algorithm
rospy.init_node("GNSS_navigation", anonymous=True)

# Initialize the UDP socket for MABX communication
mabx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mabx_addr = (mabx_IP, mabx_PORT)  


def setup_logging(file_path):
    log_dir = os.path.expanduser(
        "~/Desktop/Solio-ADAS/Solio-Suzuki/navigation/logs")
    base_filename = os.path.basename(file_path)
    # Remove the file extension to get the desired string
    logFileName = os.path.splitext(base_filename)[0]

    # Create the log directory if it does not exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Set up the logging format
    log_format = "%(asctime)s [%(levelname)s]: %(message)s"
    logging.basicConfig(level=logging.DEBUG, format=log_format)
    # this part of the code initializes the logging system, sets the log format, and configures it to capture log messages with a severity level of DEBUG or higher.
   
   
    # Create a log file with the current timestamp as the name
    current_time = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    log_filename = f"{logFileName}-{current_time}.log"
    #log filename by combining the original base filename (logFileName) extracted earlier and the current timestamp (current_time). The resulting filename will be something like "example_log-01-02-2024_15-30-45.log
    log_path = os.path.join(log_dir, log_filename)
     #os.path.join is used to create the full path to the log file by combining the log directory (log_dir) and the constructed log filename. The variable log_path now holds the complete path to the log file
    

    # Add a file handler to save logs to the file
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)

    # Set the log format for the file handler
    file_handler.setFormatter(logging.Formatter(log_format))

    # Add the file handler to the logger
    logger = logging.getLogger("")
    logger.addHandler(file_handler)

    return logger


def log_and_print(str):
    logger.info(str)
    print(str)
#log_and_print function with a string argument, it both logs the string at the INFO level using the logger and prints the string to the console

# Function to parse and retrieve coordinates from a file
def get_coordinates(file_path):  #def get_coordinates(file_path):: This line defines a function named , get_coordinate_list =[] :Initializes an empty list to store the parsed coordiantes 
    coordinates_list = []   #Initializes an empty list to store the parsed coordinates.
    try:  #The code within this block is intended to be executed, and exceptions (errors) will be caught and handled if they occur.
        with open(file_path, "r") as file:
            for line in file:
                try:
                    coordinates = [
                        float(coord) for coord in line.strip().strip("[],").split(",")
                        ]
                    coordinates_list.append(coordinates)
                except ValueError:
                    # Handle the exception if a value cannot be converted to float
                    print(
                        f"Error: Unable to convert coordinates in line '{line}' to float."
                    )
    except FileNotFoundError:
        # Handle the exception if the file is not found
        print(f"Error: The file '{file_path}' could not be found.")
    except Exception as e:
        # Handle any other unexpected exceptions
        print(f"An error occurred: {e}")

    return coordinates_list
# this above thing i am not understanding 

def callback_flag(data):#This line defines a function named callback_flag that takes a parameter data
    global CW_flag  #CW_flag is a global variable. A global variable is accessible from anywhere in the program, both inside and outside functions.
    CW_flag = 0  #This line initializes or resets the value of the global variable CW_flag to 0.
    CW_flag = data.data #CW_flag = data.data: This line assigns the value of data.data to the global variable CW_flag. It seems like data is an object with a property or attribute named data, and the value of this attribute is assigned to CW_flag


def callback_pothole(data):
    global pothole_flag
    pothole_flag = 0
    pothole_flag = data.data
#callback_pothole sets the global variable pothole_flag to 0 and then updates its value with the data received as a parameter. The purpose and behavior of the code may depend on the broader context, such as how this function is called and how the pothole_flag variable is used elsewhere in the program. It's commonly used in scenarios where the function is a callback for some event, and the pothole_flag is updated based on the incoming data associated with that event.

rospy.Subscriber("/collision", Float32, callback_flag)
rospy.Subscriber("/pothole", Float32, callback_pothole)


# Callback functions for handling GNSS data
def callback_velocity(data):
    global current_vel
    current_vel = 3.6 * data.hor_speed  #calculates a new velocity value based on the horizontal speed (hor_speed) attribute of the data object. It multiplies the horizontal speed by 3.6 to convert it from m/s to km/h and assigns the result to the global variable current_vel.


def callback_heading(data): 
    global heading
    heading = (
        data.azimuth
    )  # Left-handed rotation around z-axis in degrees clockwise from North.


def callback_latlng(data):
    global lat, lng, lat_delta, lng_delta
    lat = data.lat
    lng = data.lon
    lat_delta = data.lat_stdev
    lng_delta = data.lon_stdev


def callback_gnss_imu(data):
    global acc_z
    global hit_time
    acc_z = hit_time = 0
    acc_z = data.linear_acceleration.z
    hit_time = data.header.stamp.secs


rospy.Subscriber("/novatel/oem7/bestvel", BESTVEL, callback_velocity)
rospy.Subscriber("/novatel/oem7/inspva", INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback_latlng)
rospy.Subscriber("/imu/data_raw", Imu, callback_gnss_imu)

time.sleep(0.1)


# Function to calculate and set steering angle based on current angle and target angle
def set_angle(current_angle, angle_change):
    # Steering gear ratio
    gear_ratio = 17.75
    offset = 65.536
    LSB = 0.002
    min_steering_angle, max_steering_angle = -40, 40

    # Limit steering angle within a range
    if max_steering_angle * gear_ratio < current_angle:
        current_angle = max_steering_angle * gear_ratio
    elif min_steering_angle * gear_ratio > current_angle:
        current_angle = min_steering_angle * gear_ratio
    # Calculate scaled angle for transmission
    scaled_angle = (current_angle / gear_ratio - (-65.536)) / LSB
    high_angle_byte, low_angle_byte = (int)(scaled_angle) >> 8, (int)(
        scaled_angle
    ) & 0xFF  # extract higher 8 bits and lower 8 bits of the angle
    return high_angle_byte, low_angle_byte


def calc_checksum(message_bytes):
    checksum = 0
    for m in message_bytes:
        checksum += m
    checksum = (0x00 - checksum) & 0x000000FF  #Calculates the one's complement (bitwise NOT) of the sum obtained so far. The 0x00 - checksum operation effectively negates the sum, and & 0x000000FF ensures that only the lower 8 bits are retained.
    checksum = checksum & 0xFF  # Further ensures that the checksum is an 8-bit value by applying bitwise AND with 0xFF
    return checksum


def set_speed(speed):
    speed = speed * 128  #Multiplies the input speed by 128. This operation is essentially left-shifting the speed value by 7 bits (since 2^7 = 128). This is a common way to pack information into a smaller bit space.
    high_byte_speed = (int)(speed) >> 8
    low_byte_speed = (int)(speed) & 0xFF
    return high_byte_speed, low_byte_speed
#this function is used to convert a speed value into two bytes, with the higher 8 bits in high_byte_speed and the lower 8 bits in low_byte_speed. This kind of byte-packing is common in embedded systems or communication protocols where data needs to be efficiently transmitted or stored in a compact format.

def get_speed(high_byte_speed, low_byte_speed):
    """
    Calculates the speed from the high and low bytes of the scaled speed.

    Args:
      high_byte_speed: The high byte of the scaled speed.
      low_byte_speed: The low byte of the scaled speed.

    Returns:
      The speed in km/h.
    """

    # Combine the high and low bytes into a single integer.
    speed = (high_byte_speed << 8) | low_byte_speed

    # Divide the speed by 128 to convert it back to km/h.
    speed = speed / 128

    return speed


def send_message_to_mabx(
    speed, current_angle, delta_angle, flasher_light, message_counter
):
    H_Angle, L_Angle = set_angle(current_angle, -1 * delta_angle)
    H_Speed, L_Speed = set_speed(speed)

    message_bytes = [
        1,
        message_counter,
        0,
        1,
        52,
        136,
        215,
        1,
        H_Speed,
        L_Speed,
        H_Angle,
        L_Angle,
        0,
        flasher_light,
        0,
        0,
        0,
        0,
    ]
    message_bytes[2] = calc_checksum(message_bytes)
    message = bytearray(message_bytes)
    # print("Reading the Speed from MABX: ", get_speed(message[8], message[9]))
    # print("Reading the sent Angle from MABX: ", message[10], message[11])
    log_and_print(
        "================================================================")
    return message


def calculate_steer_output(currentLocation, Current_Bearing):
    global wp
    RAD_TO_DEG_CONVERSION = 57.2957795
    STEER_GAIN = 750         # For Tight Turns 1200 can be used or 900 in general

    off_y = -currentLocation[0] + waypoints[wp][0]
    off_x = -currentLocation[1] + waypoints[wp][1]

    # calculate bearing based on position error
    target_bearing = 90.00 + math.atan2(-off_y, off_x) * RAD_TO_DEG_CONVERSION

    # convert negative bearings to positive by adding 360 degrees
    if target_bearing < 0:
        target_bearing += 360.00

    Current_Bearing = heading
    while Current_Bearing is None:
        Current_Bearing = heading

    Current_Bearing = float(Current_Bearing)
    # log_and_print(f"Current Bearing : {Current_Bearing:.1f} , Target Bearing : {target_bearing:.1f}")

    bearing_diff = Current_Bearing - target_bearing

    # normalize bearing difference to range between -180 and 180 degrees
    if bearing_diff < -180:
        bearing_diff += 360
    elif bearing_diff > 180:
        bearing_diff -= 360

    if abs(bearing_diff) < 1:  # Nullify the small the bearing difference
        temp = bearing_diff
        # bearing_diff = 0
        STEER_GAIN = 300
    elif abs(bearing_diff) > 20:
        STEER_GAIN = 900
    log_and_print(
        f"Bearing Difference : {bearing_diff:.1f} with {STEER_GAIN:.0f} Steer Gain"
    )

    steer_output = STEER_GAIN * np.arctan(
        -1 * 2 * 3.5 * np.sin(np.radians(bearing_diff)) / 8
    )
    return steer_output, bearing_diff


def calculate_bearing_difference_for_speed_reduction(currentLocation, Current_Bearing):
    global wp
    RAD_TO_DEG_CONVERSION = 57.2957795
    next_wp = 4  # changed from 3 to 4

    if wp + next_wp < wp_len:
        off_y = -currentLocation[0] + waypoints[wp + next_wp][0]
        off_x = -currentLocation[1] + waypoints[wp + next_wp][1]
    else:
        off_y = -currentLocation[0] + waypoints[wp][0]
        off_x = -currentLocation[1] + waypoints[wp][1]

    # calculate bearing based on position error
    target_bearing = 90.00 + math.atan2(-off_y, off_x) * RAD_TO_DEG_CONVERSION

    # convert negative bearings to positive by adding 360 degrees
    if target_bearing < 0:
        target_bearing += 360.00

    Current_Bearing = heading
    while Current_Bearing is None:
        Current_Bearing = heading

    Current_Bearing = float(Current_Bearing)
    future_bearing_diff = Current_Bearing - target_bearing

    # normalize bearing difference to range between -180 and 180 degrees
    if future_bearing_diff < -180:
        future_bearing_diff += 360
    elif future_bearing_diff > 180:
        future_bearing_diff -= 360

    # log_and_print(f"    Future Bearing Difference for ({next_wp}) waypoint : {future_bearing_diff:.1f}")
    return future_bearing_diff


def reduce_speed_for_collision_warning(speed, CW_flag):
    """Reduces the speed of the vehicle based on the collision warning flag.

    Args:
        speed: The current speed of the vehicle in km/h.
        CW_flag: The collision warning flag.

    Returns:
        The reduced speed of the vehicle in km/h.
    """
    if CW_flag == 1:  # Reduce speed by 20%
        log_and_print(f"Collision Warning Status : Caution")
        speed = speed * 0.8
    elif CW_flag == 2:  # Reduce speed to 0
        log_and_print(f"Collision Warning Status : Brake Signal")
        speed = 0
    else:
        log_and_print(f"Collision Warning Status : Safe")

    return speed


def reduce_speed_for_pothole_and_speedbump(
    speed, saw_pothole, frame_count, pothole_speed=5
):
    """Reduces the speed of the vehicle based on the pothole flag.

    Args:
        speed: The current speed of the vehicle in km/h.
        pothole_flag: current pothole flag.
        saw_pothole: last frame pothole flag.
        frame_count: The frame count.
        pothole_speed: The pothole speed.

    Returns:
        The reduced speed of the vehicle in km/h.
    """
    global pothole_flag
    pothole_flag = 0  # Only when pothole_flag is not connected
    if pothole_flag == 1:
        frame_count = 0
        veh_speed = pothole_speed
    elif saw_pothole == 1 and pothole_flag == 0:
        frame_count += 1
    if frame_count < 70 and saw_pothole == 1:
        veh_speed = pothole_speed
    else:
        saw_pothole = 0
        veh_speed = speed

    if pothole_flag == 1:
        saw_pothole = pothole_flag

    if pothole_flag == 1 or saw_pothole == 1:
        print(f"SpeedBump/Pothole Detected")

    return veh_speed, frame_count, saw_pothole


def navigation_output(latitude, longitude, Current_Bearing):
    global counter, initial_speed, wp, CW_flag, saw_pothole, frame_count, vehicle_speed, pothole_flag

    flasher_dict = {0: "None", 1: "Left", 2: "Right", 3: "Both"}
    flasher = 3

    counter = (counter + 1) % 256

    # log_and_print(f"Current :- Latitude: {latitude} , Longitude: {longitude}")
    log_and_print(f"2D Standard Deviation(in cms): {100*lat_delta:.2f} cm")

    currentLocation = [latitude, longitude]

    # Calculate the distance to each waypoint
    # distances_to_waypoints = np.linalg.norm(np.array(currentLocation) - waypoints, axis=1) * LAT_LNG_TO_METER          # Testing ---- Check this vectorrized code
    distances_to_waypoints = (
        np.linalg.norm(np.array(currentLocation) -
                       waypoints[-1]) * LAT_LNG_TO_METER
    )

    if (
        distances_to_waypoints > 1 and wp < wp_len
    ):  # to check if the final point is not less than 1m
        steer_output, bearing_diff = calculate_steer_output(
            currentLocation, Current_Bearing
        )
        steer_output *= -1.0
        future_bearing_diff = calculate_bearing_difference_for_speed_reduction(
            currentLocation, Current_Bearing
        )
        # next_bearing_diff = bearing_diff - future_bearing_diff
        # log_and_print(f"Future & Current Bearing diff : {next_bearing_diff:.1f}")

        vehicle_speed = initial_speed

        if wp < 10 or wp_len - 10 < wp:  # Slow start and end in the waypoints
            vehicle_speed = turning_factor * initial_speed

        if abs(bearing_diff) > 5:
            vehicle_speed = turning_factor * initial_speed
            log_and_print(
                f"Turning Speed from code : {vehicle_speed:.0f} kmph")
        elif abs(future_bearing_diff) > 5:
            vehicle_speed = 0.8 * initial_speed
            log_and_print(f"Curve Speed from code : {vehicle_speed:.0f} kmph")

        (
            vehicle_speed,
            frame_count,
            saw_pothole,
        ) = reduce_speed_for_pothole_and_speedbump(
            initial_speed, saw_pothole, frame_count
        )
        vehicle_speed = reduce_speed_for_collision_warning(
            vehicle_speed, CW_flag)
        distance_to_nextpoint = (
            np.linalg.norm(np.array(currentLocation) -
                           waypoints[wp]) * LAT_LNG_TO_METER
        )
        log_and_print(
            f"{wp} out of {wp_len} | Next Coordinate distance : {distance_to_nextpoint:.1f} m"
        )
        try:
            if wp < wp_len and distance_to_nextpoint < LOOK_AHEAD_DISTANCE:
                wp += 1
        except IndexError:
            log_and_print(
                f"Waypoint index out of range! - Seems like you are at wrong location or Inputted wrong waypoint"
            )
    else:
        log_and_print(f"----- FINISHED  -----")
        log_and_print(f"Brake Activated")
        steer_output = 0
        vehicle_speed = 0
        flasher = 0

    log_and_print(f"Vehicle Speed(set by Code): {int(vehicle_speed)} kmph")
    try:
        message = send_message_to_mabx(
            vehicle_speed, steer_output, 0, flasher, counter)
        mabx_socket.sendto(message, mabx_addr)
    except Exception as e:
        log_and_print(f"Error sending message to MABX: {e}")


def mainLoop():
    while not rospy.is_shutdown():
        try:
            log_and_print(f"Current Coordinate No. : {wp}")
            log_and_print(" ")
            # log_and_print(f"Velocity in kmph as per GNSS= {current_vel:.0f} kmph")

            latitude = float(lat)
            longitude = float(lng)
            Current_Bearing = float(heading)

            # time.sleep(SLEEP_INTERVAL/1000)
            navigation_output(latitude, longitude, Current_Bearing)
            time.sleep(SLEEP_INTERVAL / 1000)
        except ValueError as ve:
            log_and_print(f"ValueError occurred: {ve}")
        except IOError as ioe:
            log_and_print(f"IOError occurred: {ioe}")
        except KeyboardInterrupt:  # Currently not working
            log_and_print("Autonomous Mode is terminated manually!")
            message = send_message_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)
            raise SystemExit
        except Exception as e:
            log_and_print(f"An error occurred: {e}")
        # finally:
        #     log_and_print("Autonomous Mode is terminated manually in finally block!")
        #     message = send_message_to_mabx(0, 0, 0, 0, counter)     #0
        #     mabx_socket.sendto(message, mabx_addr)


if __name__ == "__main__":
    global initial_speed, reduction_factor, steer_output, counter, curr_time, wp, file_path, pothole_flag, vehicle_speed

    # Define the path(not relative path gives Error) to the waypoints file
    # file_path = '/home/intel-nuc/Desktop/Solio-ADAS/Solio-Suzuki/navigation/solio-waypoints/Solio1/waypoints-speedtest-reverse.txt'
    file_path = "/home/s186/Desktop/Solio-ADAS/Solio-Suzuki/navigation/Waypoints/Solio2/waypoints-maingate_TO_testbed.txt"

    logger = setup_logging(file_path)
    logger.info("Demo Code Starting")

    # Set sleep interval and lookahead distance
    SLEEP_INTERVAL = 100  # CHANGED FROM 5 TO 100
    LOOK_AHEAD_DISTANCE = 3

    # Define initial speeds, pothole_speed, turning_factor
    initial_speed = 16
    vehicle_speed = initial_speed
    turning_factor = 0.6
    wp = 0
    steer_output = 0
    counter = 0

    global saw_pothole, frame_count, CW_flag
    saw_pothole = 0
    frame_count = 0
    CW_flag = 0

    # Get the list of waypoints from the file
    waypoints = get_coordinates(file_path)
    wp_len = len(waypoints)

    logger.info(
        f" Speed : {vehicle_speed} , {SLEEP_INTERVAL}, Look ahead distance : {LOOK_AHEAD_DISTANCE}"
    )
    mainLoop()
navigation.py
Displaying navigation.py.