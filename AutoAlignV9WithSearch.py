import serial  # for serial port control of serial devices
import serial.tools.list_ports
import multiprocessing
import sys

sys.path.append("/home/enorda/.local/lib/python3.6/site-packages")  # get the missing packages on the python path
import pyzed.sl as sl  # for zed camera api
import time
from dronekit import connect, VehicleMode, mavutil, LocationGlobalRelative, LocationGlobal, \
    Command  # for the drone control interface
import argparse
import time
import cv2  # for video processing and handling
import cv2.aruco as aruco  # for aruco processing
import numpy as np
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
import queue

from simple_pid import PID  # library for pid controllers
import dataclasses
from dataclasses import dataclass
# import to do path gen stuff automatically
import folium
import csv
from folium import plugins
import webbrowser  # so folium can make a map display

search_speed = 3.0 # m/s speed to search for marker in waypoint stage
align_speed = 0.75 # m/s speed to align vehicle to marker
ALTITUDE = 3.0  # Altitude to take off to (meters)
ALIGN_VEHICLE_TO_MARKER_ALT = 1.0 # meters to align vehicle at eventually
FRAME_SIZE_OVERLAP = 0.9  # overlap of the frames in the search pattern
# pids for both axis
output_min = -40  # min output for pid loop
output_max = 40  # max output for pid loop
# initialize the pid loop for the gimbal pitch axis
pitch_pid = PID(3.0, 3.0, 0.27, setpoint=0, output_limits=(output_min, output_max))
# initialize the pid loop for the gimbal roll axis
roll_pid = PID(3.75, 3.5, 0.24, setpoint=0, output_limits=(output_min, output_max))


# angle error array initialize
error_window_length = 5  # length of the error window
error_init_number = 1000  # starting error value for the list
error_angle_data_points = [error_init_number] * error_window_length  # make the specified list with the init numbers

# xy error array initialize
error_init_number = 1000  # starting error value for the list
error_xy_data_points = [error_init_number] * error_window_length  # make the specified list with the init numbers
max_queue_size = 3  # max number of items in the queue
xy_error_threshold_stage1 = 0.4  # the threshold under which to begin dropping altitude while aligning
xy_error_threshold_stage2 = 0.25  # the threshold under which to stop aligning vehicle, and begin engaging

# Set angles for each servo and the valve (0 to 180 degrees), 90 is horizontal
servo1_angle = 0.0  # unused
servo2_angle = 90.0  # roll
servo3_angle = 90.0  # pitch
valve_output_state = 0
valve_open_time = 1300
max_pitch_angle = 125.0
max_roll_angle = 125.0
min_pitch_angle = 55.0
min_roll_angle = 55.0

# set targeting thresholds and Variables
marker_length = 0.187  # in meters
friendly_marker_id = 10  # id of the friendly aruco marker
marker_id_status = [False] * 1000  # create an array of 1000 marker ids, index = id, and set the status to false
marker_id_status[friendly_marker_id] = True  # Tell it we have already engaged the friendly marker so dont engage
max_engagement_angle = 5  # maximum angle error avg to shoot water
marker_to_engage = -1
ALLOWABLE_MARKER_MISSING_TIME = 8 # max time marker is allowed to be out of frame before returning to last known location

# define globals used
arduino_serial = None  # stores the arduino serial port object
zed = None  # stores the zed camera object
left_cam_distortion = None  # zed left camera distortion coefficients
left_cam_matrix = None  # zed left camera matrix
vehicle = None  # stores the vehcile object to acess and control the drone
roll = 90.0  # roll angle of the gimbal, will be changed from initial value of 90
pitch = 90.0  # pitch angle of the gimbal, will be changed from initial value of 90
run_initial_search = True  # are we in the search stage
align_vehicle_to_marker = False  # are we in the align stage
begin_final_targeting = False  # are we in the final targeting stage

# follium and search pattern Stuff
# Example bounding box coordinates (longitude, latitude) for sat tile manual
'''
32.0363918	-96.1977924
32.0363628	-96.1976919
32.0362321	-96.1977562
32.0362594	-96.1978501
'''
folium_bbox = [-96.1977924, 32.0363918, -96.1977562, 32.0362321]
minimum_search_row_dist = 1  # minimum spacing in meters between waypoint columns
minimum_search_column_dist = 1  # minimum spacing in meters between waypoint columns
home_location = None  # store the vehicle home location
home_latitude = None
home_longitude = None
home_altitude = None
fence_waypoint_array = []  # store the fence array waypoints


# functions for search algo
def generate_folium_map(waypoints, fence_waypoint_array):
    global home_location
    global home_latitude
    global home_longitude
    # Create a Folium map centered on home location
    my_map = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=20, max_zoom=25)

    """
    # add a manually acquired image overlay to the map
    folium.raster_layers.ImageOverlay(
        image="file:///C:/Users/Admin/PycharmProjects/TestUartUax/Stuff/lot.jpg",
        bounds=[[folium_bbox[1], folium_bbox[0]], [folium_bbox[3], folium_bbox[2]]],
        opacity=0.7,
        name='Landsat Image Overlay'
    ).add_to(my_map)
    """
    print(fence_waypoint_array)

    thefencepoint = (fence_waypoint_array[0][1], fence_waypoint_array[0][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {1}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[1][1], fence_waypoint_array[1][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {2}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[2][1], fence_waypoint_array[2][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {3}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[3][1], fence_waypoint_array[3][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {4}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    # Create a PolyLine
    polyline = folium.PolyLine(locations=waypoints, color='blue', weight=3, opacity=0.0).add_to(my_map)

    # Add arrows to the PolyLine
    plugins.PolyLineTextPath(
        polyline,
        '\u2192',  # Unicode arrow character (right arrow)
        repeat=True,
        offset=6,
        attributes={'fill': 'red', 'font-weight': 'bold', 'font-size': '35'}
    ).add_to(my_map)

    '''
    new_waypoint_index = 0
    # Add waypoints to the map
    for waypoint in waypoints:
        new_waypoint_index += 1
        print(f"waypoint: {waypoint}")
        folium.Marker(waypoint, popup=f'Waypoint No. {new_waypoint_index}: {waypoint}',
                      icon=folium.Icon(color='green')).add_to(my_map)
    '''
    # Save the map as a html file
    my_map.save('generated_search_pattern_waypoints.html')

    # Optionally, open the HTML file in a web browser
    open_in_browser = True
    if open_in_browser is True:
        webbrowser.open('generated_search_pattern_waypoints.html')

    return


def equirectangular_approximation(coord1, coord2):
    """
    Calculate the approximate straight-line distance between two GPS coordinates using the Equirectangular approximation.
    Parameters:
    - coord1: Tuple of (latitude, longitude) for the first point.
    - coord2: Tuple of (latitude, longitude) for the second point.
    Returns:
    - Distance in kilometers.
    """
    # Earth radius in kilometers
    R = 6371.0
    # Convert latitude and longitude from degrees to radians
    lat1, lon1 = map(radians, coord1)
    lat2, lon2 = map(radians, coord2)
    # Equirectangular approximation
    x = (lon2 - lon1) * cos((lat1 + lat2) / 2)
    y = lat2 - lat1
    # Return Distance in kilometers
    distance = R * math.sqrt(x ** 2 + y ** 2)
    return distance


def save_waypoints_to_csv(waypoints, csv_filename):
    waypoint_index = 0
    with open(csv_filename, 'w', newline='') as csvfile:
        # create and write the header
        fieldnames = ['latitude', 'longitude']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        # run thru each waypoint and add to the csv file
        for waypoint in waypoints:
            waypoint_index += 1
            writer.writerow({'latitude': waypoint[0], 'longitude': waypoint[1]})  # add waypoint to file


def generate_zigzag_waypoints(bottom_left, top_right, rows, cols):
    zig_waypoints = []
    # Calculate the step size for rows and columns
    row_step = (top_right[0] - bottom_left[0]) / (rows - 1)
    col_step = (top_right[1] - bottom_left[1]) / (cols - 1)

    for i in range(rows):
        # Adjust the column order for every other row to create a zigzag pattern
        col_range = range(cols) if i % 2 == 0 else reversed(range(cols))

        for j in col_range:
            lat = bottom_left[0] + i * row_step
            lon = bottom_left[1] + j * col_step
            zig_waypoints.append((lat, lon))

    return zig_waypoints


def load_waypoints_from_csv(file_path):
    global ALTITUDE
    csv_loaded_waypoints = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            latitude = float(row['latitude'])
            longitude = float(row['longitude'])
            altitude = float(ALTITUDE)
            csv_loaded_waypoints.append(LocationGlobalRelative(latitude, longitude, altitude))
    return csv_loaded_waypoints


def waypoint_distance(current_location, target_location):
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    return (dlat ** 2 + dlon ** 2) ** 0.5 * 1e5


@dataclass
class HomeClassLoc:  # define a class to handle the new home location
    lat: float
    lon: float


def generate_zig_zag_path_waypoints(point1, point2, point3, num_cols, num_rows):
    # Calculate the side vectors
    side_vector1 = np.array(point2) - np.array(point1)
    side_vector2 = np.array(point3) - np.array(point2)

    # Calculate the angle between the sides
    dot_product = np.dot(side_vector1, side_vector2)
    norm_product = np.linalg.norm(side_vector1) * np.linalg.norm(side_vector2)
    cosine_angle = dot_product / norm_product
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    # Choose the side with an angle greater than 180 degrees so the parallelogram doesnt cross itself
    if angle > 180:
        fourth_point = (point3[0] + side_vector1[0], point3[1] + side_vector1[1])
    else:
        fourth_point = (point1[0] + side_vector2[0], point1[1] + side_vector2[1])

    x = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)  # Width of the grid
    y = math.sqrt((point2[0] - point3[0]) ** 2 + (point2[1] - point3[1]) ** 2)  # Height of the grid

    print(x)
    print(y)

    # Define the corners of the rectangle
    rectangle = np.array([
        [0, 0],  # Bottom-left corner
        [x, 0],  # Bottom-right corner
        [x, y],  # Top-right corner
        [0, y]  # Top-left corner
    ])

    # Define the corners of the known parallelogram
    parallelogram = np.array([
        [point1[0], point1[1]],  # Bottom-left corner
        [point2[0], point2[1]],  # Bottom-right corner
        [point3[0], point3[1]],  # Top-right corner
        [fourth_point[0], fourth_point[1]]  # Top-left corner
    ])

    # Create coordinate vectors for the columns and rows
    cols = np.linspace(0, x, num_cols)
    rows = np.linspace(0, y, num_rows)

    # Create the meshgrid
    col_mesh, row_mesh = np.meshgrid(cols, rows, indexing='xy')

    # Flatten the meshgrid arrays and combine them into one array of xy pairs
    grid_points = np.column_stack((col_mesh.ravel(), row_mesh.ravel()))

    # reverse every other row in the array
    # n = int(math.floor(x) / col_spacing)
    n = num_cols
    result = grid_points.copy()
    for i in range(0, len(grid_points), n * 2):
        result[i:i + n] = np.flip(result[i:i + n], axis=0)

    grid_points = result

    # Convert rectangle and parallelogram to numpy arrays
    rectangle = np.array(rectangle)
    parallelogram = np.array(parallelogram)

    # Append a column of ones to the rectangle coordinates for the translation component
    rectangle_with_ones = np.hstack([rectangle, np.ones((4, 1))])

    # Find the transformation matrix that maps the rectangle onto the parallelogram
    transform_matrix, _, _, _ = np.linalg.lstsq(rectangle_with_ones, parallelogram, rcond=None)

    # Apply the transformation matrix to the rectangle
    skewed_rectangle = np.dot(rectangle_with_ones, transform_matrix)

    # Append a column of ones to the rectangle coordinates for the translation component
    grid_with_ones = np.hstack([grid_points, np.ones((len(grid_points), 1))])

    skewed_grid = np.dot(grid_with_ones, transform_matrix)
    print(skewed_grid)
    skewed_rectangle, skewed_grid_points = skewed_rectangle[:, :2], skewed_grid[:,
                                                                    :2]  # Exclude the last column of ones

    skewed_grid_points = np.flip(skewed_grid_points, axis=0)

    return skewed_grid_points


def run_path_generation(vehicle, frame_width_meters, frame_height_meters):
    global home_location
    global fence_waypoint_array
    global home_latitude
    global home_longitude
    global home_altitude
    # Wait for the vehicle to have a GPS fix

    while not vehicle.gps_0.fix_type:
        print("Waiting for the GPS to have a fix...")
        time.sleep(1)
    print("gpsfixed")

    # Download the vehicle waypoints (commands). Wait until download is complete.
    print("getting the cmds position")

    # Request the total number of mission items
    vehicle.commands.download()

    # Wait for the mission download to complete
    vehicle.commands.wait_ready()

    # Get the list of downloaded mission items
    cmds = vehicle.commands
    print(f"cmds: {cmds}")

    # Print waypoint data
    if len(cmds) > 0:
        print("Mission commands:")
        for cmd in cmds:
            print(f"{cmd.seq}, {cmd.x}, {cmd.y}, {cmd.z}")
            fence_waypoint_array.append([cmd.seq, cmd.x, cmd.y, cmd.z])
    else:
        print("No mission commands downloaded.")
        sys.exit("Exiting due to no waypoint data from cmds")

    # Wait for the home location to be set
    while vehicle.home_location is None:
        print("Waiting for the home location to be set...")
        time.sleep(1)

    # Retrieve home location
    home_location = vehicle.home_location

    # home location is not very useful because it changes to current loc every arming
    # Parse and print home location information
    if home_location is not None:
        home_latitude = home_location.lat
        home_longitude = home_location.lon
        home_altitude = home_location.alt
        print(f"Home Location: Latitude={home_latitude}, Longitude={home_longitude}, Altitude={home_altitude}")
    else:
        print("Home location is not available.")
        sys.exit("Exiting due to no home position")

    # print the fence data defined as 4 normal waypoints in mission planner

    # Specify the rectangular area with four corners defined
    top_left_corner = (fence_waypoint_array[0][1], fence_waypoint_array[0][2])  # Replace with your coordinates
    top_right_corner = (fence_waypoint_array[1][1], fence_waypoint_array[1][2])  # Replace with your coordinates
    bottom_right_corner = (fence_waypoint_array[2][1], fence_waypoint_array[2][2])  # Replace with your coordinates
    bottom_left_corner = (fence_waypoint_array[3][1], fence_waypoint_array[3][2])  # Replace with your coordinates

    # get width and length of the search area in meters
    horizontal_distance = equirectangular_approximation(top_left_corner, top_right_corner) * 1000
    vertical_dist = equirectangular_approximation(top_right_corner, bottom_right_corner) * 1000

    # Number of rows and columns in the zigzag grid based on the size of the field and radius of points
    try:
        cols = int(horizontal_distance // (frame_width_meters * FRAME_SIZE_OVERLAP))
        rows = int(vertical_dist // frame_height_meters * FRAME_SIZE_OVERLAP)
    except Exception as e:
        print("rows and cols too smol using 2 atleast")
        cols = 2
        rows = 2

    if rows < 2 or cols < 2:
        cols = 2
        rows = 2
    
    rows = 2

    # Generate zigzag waypoints
    # waypoints = generate_zigzag_waypoints(bottom_left_corner, top_right_corner, rows, cols)
    waypoints = generate_zig_zag_path_waypoints(top_left_corner, top_right_corner, bottom_right_corner, rows, cols)

    # Save waypoints to CSV
    csv_filename = 'generated_search_pattern_waypoints.csv'
    save_waypoints_to_csv(waypoints, csv_filename)

    print(f'Waypoints saved to {csv_filename}')

    # should the Code generate a HTML map also?
    genMap = False
    if genMap is True:
        generate_folium_map(waypoints, fence_waypoint_array)

    HomeClassLoc.lat = home_latitude
    HomeClassLoc.lon = home_longitude

    # Load waypoints from CSV file
    waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')

    return waypoints, top_left_corner, top_right_corner


# end of path gen functions


# Used to arm the drone
def arm_drone(vehicle):
    while not vehicle.is_armable:  # While the drone hasn't been armed
        print("Waiting for drone to become armable")
        time.sleep(1)  # Wait one second before checking if drone is armable
    print("The drone is now armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':  # While drone is not in guided mode
        print("The drone is not in guided mode yet")
        time.sleep(1)  # Wait one second before checking if drone is in guided mode
    print("The drone is now in guided mode")

    vehicle.armed = True

    while not vehicle.armed:  # While the vehicle has not been armed
        print(vehicle.armed)
        print("Waiting for drone to arm")
        time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")

    # Check if GPS is functioning
    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")


# Used to take off the drone to a specific altitude
def takeoff_drone(vehicle, targetAltitude: float):
    print("Taking off!")
    vehicle.simple_takeoff(targetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Used to land the drone, prints height every second while descending
def land_drone(vehicle):
    print("Setting copter into LAND mode")
    vehicle.mode = VehicleMode('LAND')
    while vehicle.mode != 'LAND':
        time.sleep(1)
    print("Initiating landing now...")

    while vehicle.armed:  # While the drone has not landed
        currDroneHeight = vehicle.location.global_relative_frame.alt
        print("Current drone elevation: ", currDroneHeight)
        time.sleep(1)

    print("The copter has landed!")


def zed_camera_calibration(camera_calibration):
    try:
        left = camera_calibration.left_cam
    except Exception as e:
        print("Error - specified config file does not contain valid ZED config.")
        exit(1)

    Lfx = camera_calibration.left_cam.fx
    Lfy = camera_calibration.left_cam.fy
    Lcx = camera_calibration.left_cam.cx
    Lcy = camera_calibration.left_cam.cy
    Lk1 = camera_calibration.left_cam.disto[0]
    Lk2 = camera_calibration.left_cam.disto[1]
    Lk3 = camera_calibration.left_cam.disto[2]
    Lp1 = camera_calibration.left_cam.disto[3]
    Lp2 = camera_calibration.left_cam.disto[4]

    # define intrinsic camera matrices, K for {left, right} caneras

    K_CameraMatrix_left = np.array([[Lfx, 0, Lcx], [0, Lfy, Lcy], [0, 0, 1]])

    # define intrinsic camera distortion coefficients for left
    # N.B. in ZED code last three values are zero by default

    # distCoeffsl = np.array([[Lk1], [Lk2], [Lk3], [Lp1], [Lp2]])
    # zed camera is already corrected for distortion, dont apply any correction
    dist_coeffs_left = np.array([[0], [0], [0], [0], [0]])

    return dist_coeffs_left, K_CameraMatrix_left


def connect_everything():
    global arduino_serial
    global vehicle
    print("connecting to arduino")
    arduino_serial = serial.Serial((connect_to_device(device_location='1-3:1.0'))[0], baudrate=115200)
    time.sleep(2.5)
    print("Arduino connected")
    write_arduino_serial(0, 90, 90, 0, 0)
    print("connecting to drone")
    device_id_stuff = (connect_to_device(device_location='1-1:1.0', baudrate=921600))[0]
    # vehicle = connect('/dev/ttyACM0', baud = 921600, wait_ready=True)
    vehicle = connect(device_id_stuff, baud=921600, wait_ready=True)
    print("drone connected")
    return


def write_arduino_serial(servo1_angle_f, servo2_angle_f, servo3_angle_f, valve_output_state_f, valve_open_time_f):
    global arduino_serial
    # Send commands to Arduino for the position of the gimbal and the output state and duration of the water valve
    # the angles sent to the gimbal are multiplied by 10 and divided in the arduino, to increase the precision of
    # the angle requests
    # say what is being sent
    # limit the max angle request
    if servo2_angle_f > max_roll_angle:
        servo2_angle_f = max_roll_angle
    if servo2_angle_f < min_roll_angle:
        servo2_angle_f = min_roll_angle
    if servo3_angle_f > max_pitch_angle:
        servo3_angle_f = max_pitch_angle
    if servo3_angle_f < min_pitch_angle:
        servo3_angle_f = min_pitch_angle

    print(
        f"Send Ard Cmds, N/A:{servo1_angle_f}, Roll:{servo2_angle_f}, Pitch:{servo3_angle_f}, ValveState:{valve_output_state_f}, OpenTime:{valve_open_time_f}/n")
    # send the serial command
    arduino_serial.write(
        f"A {int(servo1_angle_f * 10.0)} {int(servo2_angle_f * 10.0)} {int(servo3_angle_f * 10.0)} {valve_output_state_f} {valve_open_time_f}/n".encode())
    # command success
    print("Arduino Command Sent")
    return


def update_list_and_get_average(value, data_list):
    # function to get update the array for a running mean and return the mean
    # Add the new value to the end of the list
    data_list.append(value)
    # If the length of the list exceeds 5, remove the oldest value
    if len(data_list) > error_window_length:
        data_list.pop(0)
    # Calculate the average of the updated list
    average = sum(data_list) / len(data_list)
    return average


def calculate_heading(point1, point2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
    lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])

    # Calculate differences in longitude and latitude
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    # Calculate heading using arctangent
    heading = math.atan2(dlon, dlat)

    # Convert heading from radians to degrees
    heading_deg = math.degrees(heading)

    # Adjust heading to range from 0 to 360 degrees
    if heading_deg < 0:
        heading_deg += 360

    return heading_deg


def connect_to_device(device_location=None, baudrate=115200, timeout=4):
    attempts = 0
    while attempts < 3:
        attempts += 1
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            # Check if the device matches the specified criteria
            if port.location == device_location:
                try:
                    return port
                    break
                except serial.SerialException:
                    print(serial.SerialException)
                    # Ignore errors and continue to the next port
        print("Device not found. Retrying...")
        time.sleep(1)  # Retry every 1 second
    return None


def final_targeting_stage(marker_ids, marker_centers, all_distances, shot_marker, final_targeting_until_time):
    global zed
    global roll
    global pitch
    global marker_id_status
    global marker_to_engage
    global error_angle_data_points
    actuator_latency = 0.170
    adjusted_pitch_signal = None
    adjusted_roll_signal = None
    if marker_to_engage != -1 and all(
            element is not None for element in all_distances[marker_to_engage]):  # there is a marker to shoot
        # get the distance data from the que and parce for the specific marker id
        distance_x = all_distances[marker_to_engage][0]
        distance_y = all_distances[marker_to_engage][1]
        distance_z = all_distances[marker_to_engage][2]

        # Calculate the error for the roll and pitch axes
        roll_err = math.degrees(math.atan(distance_x / distance_z))
        pitch_err = math.degrees(math.atan(distance_y / distance_z))
        print(f"roll_errr: {roll_err}")
        print(f"pitch_errr: {pitch_err}")

        # Compute the PID output to drive the error to zero
        roll_output = roll_pid(roll_err) / 10.0
        pitch_output = pitch_pid(pitch_err) / 10.0
        print(f"RollOut:{roll_output}")
        print(f"pitchOut:{pitch_output}")

        p, i, roll_d = roll_pid.components  # The separate terms are now in p, i, d
        p, i, pitch_d = pitch_pid.components

        adjusted_roll_signal = roll_output + (actuator_latency * roll_d / 10.0)
        adjusted_pitch_signal = pitch_output + (actuator_latency * pitch_d / 10.0)

        # add the output the absolute value of the angle
        roll = roll + adjusted_roll_signal
        pitch = pitch - adjusted_pitch_signal

        # tell the gimbal to move
        write_arduino_serial(0, roll, pitch, 0, 0)

        # compute the current error and the moving average error
        total_error = math.sqrt((pitch_err * pitch_err) + (roll_err * roll_err))
        total_error_average = update_list_and_get_average(total_error, error_angle_data_points)
        print(f"total_error_avg:{total_error_average}")

        # check if we can shoot based on the error
        if total_error_average <= max_engagement_angle and shot_marker == 0:
            # don't look for this marker anymore
            print("shootingWater")
            write_arduino_serial(0, roll, pitch, 1, valve_open_time)  # shoot water
            final_targeting_until_time = time.time() + (int(valve_open_time / 1000) + 0.1) # how long to continue aligning camera after shot
            shot_marker = 1

        if shot_marker == 1 and time.time() >= final_targeting_until_time: # valve has finished shooting
            shot_marker = 2 # finished state
            marker_id_status[marker_to_engage] = True  # record that we shot this marker
            marker_to_engage = -1  # don't look for this marker anymore
            print("finished shooting Water, exiting stage, returning to search")
            # recenter the camera
            roll = 90.0
            pitch = 90.0
            write_arduino_serial(0, roll, pitch, 0, 0)



    return shot_marker, final_targeting_until_time


def align_vehicle_relative_to_marker(vehicle, all_distances, gain):
    global error_xy_data_points
    global marker_to_engage
    # using the video feed, with the camera still in the horizontal orientation, align the vehicle
    # directly over the marker when aligned within a certain degree of error, exit this stage
    # and begin final targeting

    # get relative distance of vehicle to marker
    distance_x = all_distances[marker_to_engage][0]
    distance_y = all_distances[marker_to_engage][1]
    distance_z = all_distances[marker_to_engage][2]

    # start of new stuff

    # Get current location and heading
    current_location = vehicle.location.global_frame
    current_heading = vehicle.heading  # Heading in degrees (0-360), where 0 is North

    print(f"current_location:{current_location}")
    # Define the movement (5 meters forward and 3 meters to the right)
    forward_distance = distance_y  # meters
    right_distance = -distance_x  # meters

    print(f"forward_distance:{forward_distance}")
    print(f"right_distance:{right_distance}")

    # Convert heading to radians
    heading_rad = math.radians(current_heading)

    # vector xy angle
    # xy_ang = math.atan(forward_distance/right_distance)

    # Calculate the xand y components of the movement based on the heading
    x_movement = right_distance * math.cos(heading_rad) + forward_distance * math.sin(heading_rad)
    y_movement = -right_distance * math.sin(heading_rad) + forward_distance * math.cos(heading_rad)

    # x_movement = right_distance
    # y_movement = forward_distance

    # Calculate new latitude and longitude based on current position and movement
    new_latitude = current_location.lat + (
                (y_movement * gain) / 111111)  # 1 degree latitude is approximately 111111 meters
    new_longitude = current_location.lon + ((x_movement * gain) / (
                111111 * math.cos(math.radians(current_location.lat))))  # Longitude varies with latitude

    # Create the new target location
    new_location = LocationGlobal(new_latitude, new_longitude, 100.25)

    # move the vehicle relative to the current location by the distance offset

    total_xy_error_average = 1000
    # determine the current and average error for straight line distance
    if distance_y is not None and distance_x is not None and current_location is not None:
        total_xy_error = math.sqrt((distance_x * distance_x) + (
                distance_y * distance_y))  # calculate the straight line error for the positional offset
        total_xy_error_average = update_list_and_get_average(total_xy_error, error_xy_data_points)
    print(f"total_xy_error_average:{total_xy_error_average}")

    return total_xy_error_average, new_location


def get_image_dimensions_meters(dimensions, camera_matrix, frame_altitude):
    """
    Calculates the Ground Sampling Distance (GSD) in meters per pixel for the ZED camera, then calculates the
    width and height of the image in meters.

    Parameters:
    - dimensions: Tuple of (height, width) for the image in pixels.
    - camera_matrix: Camera matrix for the ZED camera.

    Returns:
    - Tuple of (width, height) for the image in meters.
    """

    STANDARD_SENSOR_SIZE = 35  # mm
    fov_vert = 70  # degrees
    fov_horizontal = 110  # degrees
    frame_width = dimensions[1]
    frame_height = dimensions[0]
    sensor_width = 4.8e-3  # sensor width in meters
    sensor_height = 3.6e-3  # focal length in meters
    sensor_size = sqrt(sensor_width ** 2 + sensor_height ** 2)

    focal_length = (camera_matrix[0][0] * sensor_width) / frame_width

    magnification = (STANDARD_SENSOR_SIZE / sensor_size) * (focal_length / 43.26661531)

    frame_height = frame_altitude * tan(radians(fov_vert / 2.0)) * 2.0 * magnification
    frame_width = frame_altitude * tan(radians(fov_horizontal / 2.9)) * 2.0 * magnification

    return frame_width, frame_height

def send_nav_waypoint(wp_index, lat, lon, alt):
    msg = vehicle.message_factory.command_int_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL,# Target system, target component
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    # Command
        0,                                        # Confirmation
        0,                                        # Frame (1: Global, 2: Local)
        0,                                        # Current
        0,                                        # Autocontinue # Params 1-4 (not used)
        lat*1e7, lon*1e7, alt
        
                 # Latitude, Longitude, Altitude
    )
    vehicle.send_mavlink(msg)

def condition_yaw(vehicle, heading, relative=False): # set the yaw angle
	# Get the target system and component IDs
    target_system = vehicle._master.target_system
    target_component = vehicle._master.target_component
    
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        target_system, target_component,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        25,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    #vehicle.send_mavlink(msg)
    return msg
    


def capture_frames(queue):
    global zed
    # initiate the zed camera
    zed = sl.Camera()
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    # init_params.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP #change the coordinate system
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode

    print("moving Camera to get correct orientation on startup")
    time.sleep(2)
    print("attempting to connect")
    # Open the camera
    err = zed.open(init_params)
    while err != sl.ERROR_CODE.SUCCESS:
        print(f"Error while trying to open ZED camera: {err}, is it connected?")
        print("attempting to reconnect")
        err = zed.open(init_params)

    # store the calibration parameters
    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters

    # define the calibration parameters for the left lens and get the distortion and Camera Matrix
    global left_cam_distortion
    global left_cam_matrix
    left_cam_distortion, left_cam_matrix = zed_camera_calibration(calibration_params)

    image_size = zed.get_camera_information().camera_configuration.resolution
    image_size.width = image_size.width
    image_size.height = image_size.height
    # Create an RGBA sl.Mat object
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    # Retrieve data in a numpy array with get_data()

    # Define aruco dictionary
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image in sl.Mat
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            # Use get_data() to get the numpy array
            image_ocv = image_zed.get_data()
            frame = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)
            # Convert image_ocv to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)

            all_marker_ids = [False] * 1000  # store rotation vectors
            all_rvecs = [[None] * 3 for _ in range(1000)]  # store rotation vectors
            all_tvecs = [[None] * 3 for _ in range(1000)]  # store translation vectors
            all_marker_centers = [[None] * 2 for _ in range(1000)]  # store the centers of each marker
            all_marker_distances = [[None] * 5 for _ in range(1000)]

            if ids is not None and frame is not None:
                # outline the marker if its detected
                aruco.drawDetectedMarkers(frame, corners, ids)
                for single_id in range(len(ids)):
                    all_marker_ids[ids[single_id][0]] = True
                    # calculate the location of the marker in the frame
                    c = corners[single_id][0]

                    # get the translation and orientation of the marker relative to the camera
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[single_id],
                                                                          marker_length,
                                                                          left_cam_matrix,
                                                                          left_cam_distortion)

                    all_tvecs[single_id] = tvecs
                    all_rvecs[single_id] = rvecs

                    marker_center_x = int((c[0][0] + c[2][0]) / 2)
                    marker_center_y = int((c[0][1] + c[2][1]) / 2)
                    marker_center = (marker_center_x, marker_center_y)  # marker center location in pixels

                    all_marker_centers[ids[single_id][0]] = marker_center
                    # put an alignment mark on the marker center
                    cv2.circle(frame, marker_center, 5, (0, 255, 0), -1)

                    # Compute distance to marker
                    distance_x = tvecs[0][0][0]  # Distance along x-axis
                    distance_y = tvecs[0][0][1]  # Distance along y-axis
                    distance_z = tvecs[0][0][2]  # Distance along z-axis
                    # calculate the distance on the x axis angle
                    x_asymptote = math.sqrt((distance_x * distance_x) + (distance_z * distance_z))
                    # calculate the distance on the y axis angle
                    y_asymptote = math.sqrt((distance_y * distance_y) + (distance_z * distance_z))

                    distances = [distance_x, distance_y, distance_z, x_asymptote, y_asymptote]

                    all_marker_distances[ids[single_id][0]] = distances

            all_data = {'frame': frame,
                        'corners': corners,
                        'all_marker_ids': all_marker_ids,
                        'rejectedImgPoints': rejectedImgPoints,
                        'all_marker_centers': all_marker_centers,
                        'all_tvecs': all_tvecs,
                        'all_rvecs': all_rvecs,
                        'all_marker_distances': all_marker_distances,
                        'camera_matrix': left_cam_matrix}

            queue.put(all_data)


def process_frames(queue=None):
    global zed
    global roll
    global pitch
    global marker_id_status
    global marker_to_engage
    global vehicle
    global error_xy_data_points
    global frame_size_meters
    marker_to_engage = -1
    has_path_been_generated = False
    global run_initial_search
    run_initial_search = True
    global align_vehicle_to_marker
    align_vehicle_to_marker = False
    global begin_final_targeting
    target_location = None
    begin_final_targeting = False
    go_away_bad_guy = False
    search_waypoints = None
    search_waypoint_index = 0
    last_search_waypoint_index = -1
    final_search_waypoint_index = None
    align_xy_stage_1_complete = False
    last_known_marker_location = None
    last_time_marker_seen = None
    final_targeting_until_time = 0
    marker_not_in_view = False
    shot_marker = 0
    connect_everything()
    start_time = 0.0
    heading_deg = None
    new_loc = None
    end_time = 0.0
    # tell the camera gimbal to pitch up so that when the zed camera is initialized the frame orientation is correct
    write_arduino_serial(90, 90, 70, 0, 0)

    while True:

        if not queue.empty():
            # Record the end time
            end_time = time.time()

            # Calculate the duration in milliseconds
            duration_ms = (end_time - start_time) * 1000

            # print("Duration of operation:", duration_ms, "milliseconds")

            start_time = time.time()

            # print(f"run_initial_search:{run_initial_search}")
            # print(f"align_vehicle_to_marker:{align_vehicle_to_marker}")
            # print(f"begin_final_targeting:{begin_final_targeting}")

            all_data = queue.get()
            frame = all_data['frame']
            marker_ids = all_data['all_marker_ids']
            marker_centers = all_data['all_marker_centers']
            all_distances = all_data['all_marker_distances']
            camera_matrix = all_data['camera_matrix']

            # figure out which marker we are engaging store the first engage-able marker, and dont look again till its reset
            if marker_ids is not None and marker_to_engage == -1:
                for marker_id in range(len(marker_ids)):
                    if marker_id_status[marker_id] is False and marker_ids[marker_id] is True:
                        run_initial_search = False
                        align_vehicle_to_marker = True
                        # the marker hasnt been enaged or is friendly
                        marker_to_engage = marker_id
                        print(f"marker to engage:{marker_to_engage}")
                        # print(f"marker list:{marker_ids}")
                        # add break here for perf gains

            if run_initial_search is True:
                # run the search pattern to find the vehicle, when its found, stop searching and begin the align to marker stage
                # check if the search pattern has been generated and if it has not, call fn
                if has_path_been_generated is False:
                    # Finding frame size in meters
                    camera_frame_width, camera_frame_height = get_image_dimensions_meters(frame.shape, camera_matrix,
                                                                                          ALTITUDE)

                    write_arduino_serial(90, 90, 90, 0, 0)
                    time.sleep(2)
                    write_arduino_serial(90, 90, 90, 0, 0)
                    arm_drone(vehicle)  # arm the vehicle
                    search_waypoints, point1, point2 = run_path_generation(vehicle, camera_frame_width,
                                                                           camera_frame_height)  # generate the path
                    print("success")
                    has_path_been_generated = True  # dont run path gen again
                    time.sleep(5)
                    takeoff_drone(vehicle, ALTITUDE)  # take the vehicle off
                    anew_loc = vehicle.location.global_frame
                    
                    vehicle.simple_goto(anew_loc, groundspeed=search_speed)
                    time.sleep(1)
                    heading_deg = calculate_heading(point1, point2) # calculate the angle to search at
                    print(heading_deg)
                    message = condition_yaw(vehicle, heading_deg, relative=False) # set the yaw angle
                    vehicle.send_mavlink(message)
                    time.sleep(5)

                if marker_to_engage == -1:
                    if search_waypoints is not None:
                        current_waypoint = search_waypoints[search_waypoint_index]
                        if go_away_bad_guy is True: # when returning to the search, need to reinitalize the search waypoint
                            home_altitude = vehicle.home_location.alt
                            target_altitude = home_altitude + ALTITUDE
                            new_loc = LocationGlobal(current_waypoint.lat, current_waypoint.lon, target_altitude)
                            vehicle.simple_goto(new_loc, groundspeed=search_speed)
                            message = condition_yaw(vehicle, heading_deg, relative=False) # set the yaw angle
                            vehicle.send_mavlink(message)
                            go_away_bad_guy = False
                        if last_search_waypoint_index != search_waypoint_index:
                            print(f"Going to waypoint {search_waypoint_index}: {current_waypoint}")
                            vehicle.simple_goto(current_waypoint, groundspeed=search_speed)
                            message = condition_yaw(vehicle, heading_deg, relative=False) # set the yaw angle
                            vehicle.send_mavlink(message)
                            last_search_waypoint_index = search_waypoint_index  # reset the last waypoint
                            final_search_waypoint_index = len(search_waypoints) - 1

                        distance_to_waypoint = waypoint_distance(vehicle.location.global_relative_frame,
                                                                 current_waypoint)
                        if distance_to_waypoint < 0.75:  # has the vehicle reached the waypoint
                            print(f"Reached waypoint {search_waypoint_index}")
                            search_waypoint_index += 1

                        if search_waypoint_index >= final_search_waypoint_index:
                            print("Mission Un-Successful Aruco Marker Not Found")
                            vehicle.mode = VehicleMode("RTL")
                            while True:
                                dist_to_waypoint = waypoint_distance(vehicle.location.global_relative_frame,
                                                                     HomeClassLoc)
                                if dist_to_waypoint < 1:  # distance in meters?
                                    print("Reached RTL waypoint, Initiating Landing")
                                    break
                                time.sleep(0.5)
                            land_drone(vehicle)

            # align the vehicle relative to the maker using the camera to center the camera
            if align_vehicle_to_marker is True:
                # print("aligning vehicle to marker")
                total_xy_error_average = 100
                distance_x = all_distances[marker_to_engage][0]
                distance_y = all_distances[marker_to_engage][1]
                distance_z = all_distances[marker_to_engage][2]
                if distance_x is not None and distance_y is not None and distance_z is not None:
                    marker_not_in_view = False
                    total_xy_error_average, target_location = align_vehicle_relative_to_marker(vehicle, all_distances,
                                                                                               0.85)
                    print(f"target_location: {target_location}")
                    last_known_marker_location = new_loc
                    home_altitude = vehicle.home_location.alt
                    target_altitude = home_altitude + ALTITUDE
                    new_loc = LocationGlobal(target_location.lat, target_location.lon, target_altitude)
                    if align_xy_stage_1_complete is False:  # if the vehicle is close enough to engage
                        target_altitude = home_altitude + ALIGN_VEHICLE_TO_MARKER_ALT
                        new_loc = LocationGlobal(target_location.lat, target_location.lon, target_altitude)
                        if (vehicle.location.global_relative_frame.alt + .05) > ALIGN_VEHICLE_TO_MARKER_ALT > (
                                vehicle.location.global_relative_frame.alt - .05) and total_xy_error_average < xy_error_threshold_stage1:
                            align_xy_stage_1_complete = True
                    if align_xy_stage_1_complete is True:  # if the vehicle is close enough to engage
                        target_altitude = home_altitude + ALIGN_VEHICLE_TO_MARKER_ALT
                        new_loc = LocationGlobal(target_location.lat, target_location.lon, target_altitude)
                    vehicle.simple_goto(new_loc, groundspeed=align_speed)
                    # store the last known location of the marker
                elif distance_x is None: # no marker in frame
                    if marker_not_in_view is False:
                        # start Timer
                        # reset the flag to not cycle the timmer
                        last_time_marker_seen = time.time()
                        marker_not_in_view = True
                    elapsed_time = time.time() - last_time_marker_seen
                    if elapsed_time > ALLOWABLE_MARKER_MISSING_TIME and marker_not_in_view is True:
                        marker_not_in_view = False
                        if last_known_marker_location is not None:
                            last_known_marker_location.alt += 1
                            vehicle.simple_goto(last_known_marker_location, groundspeed=align_speed) # return to last known loc

                if align_xy_stage_1_complete is True and total_xy_error_average < xy_error_threshold_stage2:  # if the vehicle is close enough to engage
                    # the vehicle is close enough to engage, break out of this stage
                    align_xy_stage_1_complete = False
                    align_vehicle_to_marker = False  # stop alignment of vehicle
                    begin_final_targeting = True  # start targeting

            # use the gimbal to make the bore line of the camera aligned with the marker
            if begin_final_targeting is True:
                # print("final targeting vehicle to marker")
                # call the final targeting function to align the gimbal and shoot the water
                shot_marker, final_targeting_until_time = final_targeting_stage(marker_ids, marker_centers, all_distances, shot_marker, final_targeting_until_time)
                if shot_marker == 2:
                    begin_final_targeting = False
                    run_initial_search = True
                    go_away_bad_guy = True
                    shot_marker = 0


            resized_frame = cv2.resize(frame, None, fx=0.35, fy=0.35)  # create the display window

            # Display the resized frame in a window
            cv2.imshow('Resized Frame', resized_frame)
            # cv2.imshow('Processed Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # make a que for the data to be passed between processes
    frame_queue = multiprocessing.Queue(maxsize=max_queue_size)

    # create the capture and frame processing, processes so that they can run parallel
    capture_process = multiprocessing.Process(target=capture_frames, args=(frame_queue,))
    process_process = multiprocessing.Process(target=process_frames, args=(frame_queue,))

    # start the processes
    capture_process.start()
    process_process.start()

    # this probably should be deleted
    capture_process.join()  # Wait for the capture process to finish (which won't happen)
    process_process.join()
