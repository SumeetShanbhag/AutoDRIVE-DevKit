#!/usr/bin/env python

# Import libraries
import numpy as np
import base64
from io import BytesIO
from PIL import Image
import cv2
import time
import math
import matplotlib.pyplot as plt
################################################################################

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        derivative = error - self.prev_error
        self.integral += error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        return output

def detect_lanes(front_camera_image):
    # Step 1: Convert to grayscale
    gray_image = cv2.cvtColor(front_camera_image, cv2.COLOR_BGR2GRAY)

    # Step 2: Apply Gaussian blur
    blur_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

    # Step 3: Canny edge detection
    edges_image = cv2.Canny(blur_image, 50, 150)

    # Step 4: Define region of interest
    height, width = edges_image.shape
    mask = np.zeros_like(edges_image)
    polygon = np.array([[
        (0, height * 0.8),
        (width, height * 0.8),
        (width, height),
        (0, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges_image, mask)

    # Step 5: Hough transform for line detection
    lines = cv2.HoughLinesP(cropped_edges, 2, np.pi / 180, 50, np.array([]), minLineLength=40, maxLineGap=5)

    # Step 6: Process lines to output lane markers
    lane_lines = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                lane_lines.append((x1, y1, x2, y2))

    return lane_lines

def convert_path_to_commands(path, current_orientation, vehicle, max_steering_angle, max_throttle):
    if not path or len(path) < 2:
        return None, None  # No path or insufficient waypoints

    # Current position is the first point in the path
    current_position = path[0]
    next_waypoint = path[1]

    # Calculate angle to the next waypoint
    direction = math.atan2(next_waypoint[1] - current_position[1], next_waypoint[0] - current_position[0])
    steering_angle = direction - current_orientation
    steering_angle = max(-max_steering_angle, min(steering_angle, max_steering_angle))  # Limit the steering angle

    # Calculate distance to the next waypoint for throttle
    distance = math.sqrt((next_waypoint[0] - current_position[0]) ** 2 + (next_waypoint[1] - current_position[1]) ** 2)
    throttle = min(distance, max_throttle)  # Set throttle based on distance

    # Update vehicle's steering and throttle
    vehicle.steering_command = steering_angle
    vehicle.throttle_command = throttle

    return vehicle

# Vehicle class
class Vehicle:
    def __init__(self, grid_size=(100, 100)):
        # Vehicle data
        self.id                       = None
        self.throttle                 = None
        self.steering                 = None
        self.encoder_ticks            = None
        self.encoder_angles           = None
        self.position                 = None
        self.orientation_quaternion   = None
        self.orientation_euler_angles = None
        self.angular_velocity         = None
        self.linear_acceleration      = None
        self.lidar_scan_rate          = None
        self.lidar_range_array        = None
        self.lidar_intensity_array    = None
        self.front_camera_image       = None
        self.rear_camera_image        = None
        self.grid_map = None
        self.grid_size = grid_size
        # Vehicle commands
        self.throttle_command   = 0.0
        self.steering_command   = None
        self.headlights_command = None
        self.indicators_command = None

    # def update_grid_map(self, lidar_range_array, resolution=0.1):
    #     """
    #     Refines the grid map update process to align with calc_obstacle_map logic.
    #     """
    #     # Use vehicle's orientation and position
    #     vehicle_orientation = self.orientation_euler_angles[2]  # Assuming the yaw angle is the third element
    #     vehicle_position = self.position
    #     grid_size = self.grid_size  # Assuming this attribute is already defined

    #     obstacle_map = np.zeros(grid_size, dtype=int)  # Initialize the grid map

    #     # Calculate offset for the grid's center
    #     offset = np.array(grid_size) // 2
    #     vehicle_grid_x = int(vehicle_position[0] / resolution) + offset[0]
    #     vehicle_grid_y = int(vehicle_position[1] / resolution) + offset[1]

    #     # Mark the vehicle's position on the grid
    #     obstacle_map[vehicle_grid_x][vehicle_grid_y] = 2  # 2 represents the vehicle

    #     for index, distance in enumerate(lidar_range_array):
    #         if not np.isfinite(distance):  # Filter out invalid readings
    #             continue

    #         # Calculate angle for each LIDAR point
    #         angle = vehicle_orientation + np.deg2rad(index)  # Assuming 1 degree increment
    #         # Convert polar coordinates to Cartesian
    #         x_m = distance * np.cos(angle)
    #         y_m = distance * np.sin(angle)
    #         # Transform to grid coordinates
    #         grid_x = int((x_m / resolution) + vehicle_grid_x)
    #         grid_y = int((y_m / resolution) + vehicle_grid_y)

    #         # Mark obstacle position if within bounds
    #         if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
    #             obstacle_map[grid_x][grid_y] = 1  # 1 represents an obstacle

    #     self.grid_map = obstacle_map

    def update_grid_map(self, lidar_range_array, resolution=0.1, vehicle_radius=0.5):
        vehicle_orientation = self.orientation_euler_angles[2]  # Assuming this is the yaw angle
        vehicle_position = self.position  # Assuming this is in meters [x, y, z]
        grid_size = self.grid_size  # Assuming this is defined elsewhere [width, height]

        # Initialize the grid map with zeros (free space)
        obstacle_map = np.zeros(grid_size, dtype=int)

        # Calculate the grid position of the vehicle
        offset_x = int(vehicle_position[0] / resolution)
        offset_y = int(vehicle_position[1] / resolution)

        for index, distance in enumerate(lidar_range_array):
            if np.isfinite(distance):
                # Convert LIDAR data point to Cartesian coordinates
                angle = vehicle_orientation + index * np.deg2rad(1)  # Assuming 1-degree steps in LIDAR data
                dx = distance * np.cos(angle)
                dy = distance * np.sin(angle)

                # Convert to grid coordinates
                x = int((vehicle_position[0] + dx) / resolution)
                y = int((vehicle_position[1] + dy) / resolution)

                # Mark obstacles on the grid, considering vehicle's radius
                for ix in range(-math.ceil(vehicle_radius / resolution), math.ceil(vehicle_radius / resolution) + 1):
                    for iy in range(-math.ceil(vehicle_radius / resolution), math.ceil(vehicle_radius / resolution) + 1):
                        if ix**2 + iy**2 <= (vehicle_radius / resolution)**2:
                            grid_x, grid_y = x + ix, y + iy
                            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                                obstacle_map[grid_x, grid_y] = 1  # Mark as obstacle

        self.grid_map = obstacle_map

    def visualize_grid_map(self):
        """Visualizes the current grid map."""
        if self.grid_map is not None:
            # Flip the grid map vertically to align with matplotlib's coordinate system
            flipped_map = cv2.flip(self.grid_map, 0)
            plt.imshow(flipped_map, cmap='gray', origin='lower', extent=[-self.grid_size[0]/2, self.grid_size[0]/2, -self.grid_size[1]/2, self.grid_size[1]/2])
            plt.title(f"Grid Map for Vehicle {self.id}")
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.draw()
            plt.pause(0.001)  # Small pause to update the plot without blocking

    # Method to calculate distance to nearest obstacle using LIDAR data
    def distance_to_nearest_obstacle(self):
        # Calculate the minimum distance from the LIDAR data
        min_distance = np.min(self.lidar_range_array)
        return min_distance

    def is_dead_end(self, threshold=0.5):
        """Determine if the vehicle is in a dead-end situation."""
        # Check front, left, and right LIDAR data
        front_data = self.lidar_range_array[89:91]  # Front 2 degrees
        left_data = self.lidar_range_array[:45]     # Left 45 degrees
        right_data = self.lidar_range_array[135:]   # Right 45 degrees

        # Check if all three directions have obstacles within the threshold
        dead_end_detected = all(distance < threshold for distance in [np.min(front_data), np.min(left_data), np.min(right_data)])
        if dead_end_detected:
            print("Dead-end detected by the function!")
        return dead_end_detected

    # Method for obstacle avoidance
    def avoid_obstacle(self, min_distance):
        distance_to_obstacle = self.distance_to_nearest_obstacle()

        # Dead-end detection and reversal logic
        if self.is_dead_end():
            self.throttle_command = -0.5  # Reverse throttle
            print("Dead-end detected! Reversing...")
            return

        # If an obstacle is close, but not very close, reduce speed and steer away
        if distance_to_obstacle < min_distance * 0.75:
            self.throttle_command *= 0.5  # Reduce speed to 50% of current speed
            self.steering_command += 1 if np.argmin(self.lidar_range_array) < len(self.lidar_range_array) / 2 else -1  # Steer away from the obstacle

        # If an obstacle is very close, reduce speed even more and steer away aggressively
        elif distance_to_obstacle < min_distance / 2:
            self.throttle_command *= 0.3  # Reduce speed to 30% of current speed
            self.steering_command += 1 if np.argmin(self.lidar_range_array) < len(self.lidar_range_array) / 2 else -1  # Steer away from the obstacle

        # General obstacle avoidance
        elif distance_to_obstacle < min_distance:
            # Get the index of the minimum distance in the LIDAR array
            min_index = np.argmin(self.lidar_range_array)

            # Increase steering sensitivity: steer more aggressively
            steering_adjustment = 1 if min_index < len(self.lidar_range_array) / 2 else -1
            self.steering_command += steering_adjustment

            # Ensure the steering command is within a valid range (-1 to 1)
            self.steering_command = max(-1, min(self.steering_command, 1))

    # Parse vehicle sensor data
    def parse_data(self, data, verbose=False):
        # Parse the steering command data
        self.steering_command = data.get('steering', 0.0)  # Default to 0.0 if not provided

        # Actuator feedbacks
        self.throttle = float(data[self.id + " Throttle"])
        self.steering = float(data[self.id + " Steering"])
        # Wheel encoders
        self.encoder_ticks = np.fromstring(data[self.id + " Encoder Ticks"], dtype=int, sep=' ')
        self.encoder_angles = np.fromstring(data[self.id + " Encoder Angles"], dtype=float, sep=' ')
        # IPS
        self.position = np.fromstring(data[self.id + " Position"], dtype=float, sep=' ')
        # IMU
        self.orientation_quaternion = np.fromstring(data[self.id + " Orientation Quaternion"], dtype=float, sep=' ')
        self.orientation_euler_angles = np.fromstring(data[self.id + " Orientation Euler Angles"], dtype=float, sep=' ')
        self.angular_velocity = np.fromstring(data[self.id + " Angular Velocity"], dtype=float, sep=' ')
        self.linear_acceleration = np.fromstring(data[self.id + " Linear Acceleration"], dtype=float, sep=' ')
        # LIDAR
        self.lidar_scan_rate = float(data[self.id + " LIDAR Scan Rate"])
        self.lidar_range_array = np.fromstring(data[self.id + " LIDAR Range Array"], dtype=float, sep=' ')
        self.lidar_intensity_array = np.fromstring(data[self.id + " LIDAR Intensity Array"], dtype=float, sep=' ')
        # Cameras
        self.front_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data[self.id + " Front Camera Image"])))), cv2.COLOR_RGB2BGR)
        self.rear_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data[self.id + " Rear Camera Image"])))), cv2.COLOR_RGB2BGR)

        detected_lane_lines = detect_lanes(self.front_camera_image)

        # Draw the detected lanes on the image for visualization (optional)
        for line in detected_lane_lines:
            x1, y1, x2, y2 = line
            cv2.line(self.front_camera_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
        cv2.imshow(self.id + ' Front Camera Preview with Lanes', cv2.resize(self.front_camera_image, (640, 360)))
        if verbose:
            print('\n--------------------------------')
            print('Receive Data from Vehicle: ' + self.id)
            print('--------------------------------\n')
            # Monitor vehicle data
            print('Throttle: {}'.format(self.throttle))
            print('Steering: {}'.format(self.steering))
            print('Encoder Ticks:  {} {}'.format(self.encoder_ticks[0],self.encoder_ticks[1]))
            print('Encoder Angles: {} {}'.format(self.encoder_angles[0],self.encoder_angles[1]))
            print('Position: {} {} {}'.format(self.position[0],self.position[1],self.position[2]))
            print('Orientation [Quaternion]: {} {} {} {}'.format(self.orientation_quaternion[0],self.orientation_quaternion[1],self.orientation_quaternion[2],self.orientation_quaternion[3]))
            print('Orientation [Euler Angles]: {} {} {}'.format(self.orientation_euler_angles[0],self.orientation_euler_angles[1],self.orientation_euler_angles[2]))
            print('Angular Velocity: {} {} {}'.format(self.angular_velocity[0],self.angular_velocity[1],self.angular_velocity[2]))
            print('Linear Acceleration: {} {} {}'.format(self.linear_acceleration[0],self.linear_acceleration[1],self.linear_acceleration[2]))
            print('LIDAR Scan Rate: {}'.format(self.lidar_scan_rate))
            print('LIDAR Range Array: \n{}'.format(self.lidar_range_array))
            print('LIDAR Intensity Array: \n{}'.format(self.lidar_intensity_array))
            #cv2.imshow(self.id + ' Front Camera Preview', cv2.resize(self.front_camera_image, (640, 360)))
            #cv2.imshow(self.id + ' Rear Camera Preview', cv2.resize(self.rear_camera_image, (640, 360)))
            cv2.waitKey(1)

    def compute_lateral_error(self, left_line, right_line, image_width):
        left_line_center = (left_line[0] + left_line[2]) / 2
        right_line_center = (right_line[0] + right_line[2]) / 2

        lane_center = (left_line_center + right_line_center) / 2
        image_center = image_width / 2

        error = image_center - lane_center
        return error

    # Generate vehicle control commands
    def generate_commands(self, verbose=False):
        if verbose:
            print('\n-------------------------------')
            print('Transmit Data to Vehicle: ' + self.id)
            print('-------------------------------\n')
            # Monitor vehicle control commands
            print('Throttle Command: {}'.format(self.throttle_command))
            print('Steering Command: {}'.format(self.steering_command))
            if self.headlights_command == 0:
                headlights_cmd_str = 'Disabled'
            elif self.headlights_command == 1:
                headlights_cmd_str = 'Low Beam'
            elif self.headlights_command == 2:
                headlights_cmd_str = 'High Beam'
            else:
                headlights_cmd_str = 'Invalid'
            print('Headlights Command: {}'.format(headlights_cmd_str))
            if self.indicators_command == 0:
                indicators_cmd_str = 'Disabled'
            elif self.indicators_command == 1:
                indicators_cmd_str = 'Left Turn Indicator'
            elif self.indicators_command == 2:
                indicators_cmd_str = 'Right Turn Indicator'
            elif self.indicators_command == 3:
                indicators_cmd_str = 'Hazard Indicator'
            else:
                indicators_cmd_str = 'Invalid'
            print('Indicators Command: {}'.format(indicators_cmd_str))

                # Perform obstacle avoidance
        min_distance = 0.5 # Set the minimum distance for collision avoidance
        self.avoid_obstacle(min_distance)

        # Ensure that throttle and steering commands are within valid ranges
        self.throttle_command = max(-1, min(self.throttle_command, 1))
        self.steering_command = max(-1, min(self.steering_command, 1))

        # if verbose:
        #     # Print updated vehicle control command details
        #     print('\n-------------------------------')
        #     print('Updated Commands for Vehicle: ' + self.id)
        #     print('-------------------------------\n')
        #     print('Throttle Command: {}'.format(self.throttle_command))
        #     print('Steering Command: {}'.format(self.steering_command))
        #     print('Headlights Command: {}'.format(headlights_cmd_str))
        #     print('Indicators Command: {}'.format(indicators_cmd_str))

        return {str(self.id) + ' Throttle': str(self.throttle_command), str(self.id) + ' Steering': str(self.steering_command), str(self.id) + ' Headlights': str(self.headlights_command), str(self.id) + ' Indicators': str(self.indicators_command)}

################################################################################

# Traffic light class
class TrafficLight:
    def __init__(self):
        # Traffic light data
        self.id    = None
        self.state = None
        # Traffic light command
        self.command = None

    # Parse traffic light data
    def parse_data(self, data, verbose=False):
        # Traffic light state
        self.state = int(data[self.id + " State"])
        if verbose:
            print('\n--------------------------------------')
            print('Receive Data from Traffic Light: ' + self.id)
            print('--------------------------------------\n')
            # Monitor traffic light data
            if self.state == 0:
                state_str = 'Disabled'
            elif self.state == 1:
                state_str = 'Red'
            elif self.state == 2:
                state_str = 'Yellow'
            elif self.state == 3:
                state_str = 'Green'
            else:
                state_str = 'Invalid'
            print('Traffic Light State: {}'.format(state_str))

    # Generate traffic light control commands
    def generate_commands(self, verbose=False):
        if verbose:
            print('\n-------------------------------------')
            print('Transmit Data to Traffic Light: ' + self.id)
            print('-------------------------------------\n')
            # Monitor traffic light control commands
            if self.command == 0:
                command_str = 'Disabled'
            elif self.command == 1:
                command_str = 'Red'
            elif self.command == 2:
                command_str = 'Yellow'
            elif self.command == 3:
                command_str = 'Green'
            else:
                command_str = 'Invalid'
            print('Traffic Light Command: {}'.format(command_str))
        return {str(self.id) + ' State': str(self.command)}

# Add this function to handle coordinate transformation
def transform_to_world_coordinates(x_rel, y_rel, vehicle_position, vehicle_orientation):
    # Assuming vehicle_orientation is the yaw angle in radians
    x_world = vehicle_position[0] + x_rel * np.cos(vehicle_orientation) - y_rel * np.sin(vehicle_orientation)
    y_world = vehicle_position[1] + x_rel * np.sin(vehicle_orientation) + y_rel * np.cos(vehicle_orientation)
    return x_world, y_world

# Add this function to calculate the obstacle map
# def calc_obstacle_map(vehicle, grid_size, resolution):
#     lidar_data = vehicle.lidar_range_array
#     vehicle_position = vehicle.position
#     vehicle_orientation = vehicle.orientation_euler_angles[2]  # Assuming the yaw angle is the third element

#     obstacle_map = np.zeros(grid_size, dtype=int)  # Use int type to allow different values

#     # Offset to handle negative positions
#     offset = np.array(grid_size) // 2
#     vehicle_grid_x = int(vehicle_position[0] / resolution) + offset[0]
#     vehicle_grid_y = int(vehicle_position[1] / resolution) + offset[1]

#     # Mark the vehicle's position on the map if within bounds
#     if 0 <= vehicle_grid_x < grid_size[0] and 0 <= vehicle_grid_y < grid_size[1]:
#         obstacle_map[vehicle_grid_x][vehicle_grid_y] = 2  # 2 represents the vehicle

#     # Iterate over lidar data to populate the obstacle map
#     for index, distance in enumerate(lidar_data):
#         if not np.isfinite(distance):  # Skip infinite LIDAR readings
#             continue
#         # Calculate angle for each LIDAR point
#         angle = vehicle_orientation + index * (2 * np.pi / len(lidar_data))
#         # Convert polar coordinates to Cartesian
#         obstacle_x_rel = distance * np.cos(angle)
#         obstacle_y_rel = distance * np.sin(angle)
#         # Transform to grid coordinates
#         grid_x = int((vehicle_position[0] + obstacle_x_rel) / resolution) + offset[0]
#         grid_y = int((vehicle_position[1] + obstacle_y_rel) / resolution) + offset[1]
#         # Mark obstacle position if within bounds
#         if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
#             obstacle_map[grid_x][grid_y] = 1  # 1 represents an obstacle

#     return obstacle_map

# def calc_obstacle_map(vehicle, resolution=0.05, vehicle_radius=0.2):
#     """
#     Calculate an obstacle map based on LIDAR data from a vehicle.

#     Parameters:
#     vehicle: Vehicle object with LIDAR data and position attributes.
#     resolution: The grid resolution in meters.
#     vehicle_radius: Radius of the vehicle to consider for the obstacle buffer.

#     Returns:
#     obstacle_map: A 2D numpy array representing the obstacle grid.
#     min_x, min_y: Minimum x and y indices in the grid.
#     max_x, max_y: Maximum x and y indices in the grid.
#     x_w, y_w: Width and height of the grid.
#     """

#    # Extract LIDAR data and vehicle position
#     lidar_data = vehicle.lidar_range_array
#     vehicle_position = vehicle.position
#     vehicle_orientation = vehicle.orientation_euler_angles[2]  # Assuming yaw is the third element

#     ox, oy = [], []
#     for index, distance in enumerate(lidar_data):
#         if np.isfinite(distance):
#             angle = vehicle_orientation + np.deg2rad(index)  # Assuming 1-degree steps in LIDAR data
#             ox.append(vehicle_position[0] + distance * np.cos(angle))
#             oy.append(vehicle_position[1] + distance * np.sin(angle))

#     min_x, min_y = np.min(ox), np.min(oy)
#     max_x, max_y = np.max(ox), np.max(oy)

#     # Determine grid dimensions
#     x_w = int(np.round((max_x - min_x) / resolution))
#     y_w = int(np.round((max_y - min_y) / resolution))

#     obstacle_map = np.zeros((x_w + 1, y_w + 1), dtype=bool)

#     for x, y in zip(ox, oy):
#         ix = int(np.round((x - min_x) / resolution))
#         iy = int(np.round((y - min_y) / resolution))
#         obstacle_map[ix][iy] = True  # Mark as obstacle

#         # Mark surrounding cells as obstacles considering the vehicle's radius
#         for dx in np.arange(-vehicle_radius, vehicle_radius + resolution, resolution):
#             for dy in np.arange(-vehicle_radius, vehicle_radius + resolution, resolution):
#                 if dx**2 + dy**2 <= vehicle_radius**2:
#                     idx = int(np.round((x + dx - min_x) / resolution))
#                     idy = int(np.round((y + dy - min_y) / resolution))
#                     if 0 <= idx < x_w and 0 <= idy < y_w:
#                         obstacle_map[idx][idy] = True

#     return obstacle_map, min_x, min_y, max_x, max_y, x_w, y_w

def calc_obstacle_map(vehicle, grid_size, resolution):
    lidar_data = vehicle.lidar_range_array
    vehicle_position = vehicle.position
    vehicle_orientation = vehicle.orientation_euler_angles[2]  # Assuming the yaw angle is the third element

    obstacle_map = np.zeros(grid_size, dtype=int)  # Initialize obstacle map

    # Calculate extents based on vehicle position and LIDAR data
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')

    # Offset to center the vehicle in the grid
    offset = np.array(grid_size) // 2
    vehicle_grid_x = int(vehicle_position[0] / resolution) + offset[0]
    vehicle_grid_y = int(vehicle_position[1] / resolution) + offset[1]

    # Update min and max extents based on vehicle's position
    min_x, min_y = min(min_x, vehicle_grid_x), min(min_y, vehicle_grid_y)
    max_x, max_y = max(max_x, vehicle_grid_x), max(max_y, vehicle_grid_y)

    # Mark the vehicle's position on the map
    if 0 <= vehicle_grid_x < grid_size[0] and 0 <= vehicle_grid_y < grid_size[1]:
        obstacle_map[vehicle_grid_x][vehicle_grid_y] = 2  # 2 represents the vehicle

    # Process LIDAR data to populate the obstacle map and update extents
    for index, distance in enumerate(lidar_data):
        if not np.isfinite(distance):  # Skip invalid LIDAR readings
            continue
        angle = vehicle_orientation + index * (2 * np.pi / len(lidar_data))
        obstacle_x_rel = distance * np.cos(angle)
        obstacle_y_rel = distance * np.sin(angle)

        grid_x = int((vehicle_position[0] + obstacle_x_rel) / resolution) + offset[0]
        grid_y = int((vehicle_position[1] + obstacle_y_rel) / resolution) + offset[1]

        # Update grid extents based on obstacles
        min_x, min_y = min(min_x, grid_x), min(min_y, grid_y)
        max_x, max_y = max(max_x, grid_x), max(max_y, grid_y)

        # Mark obstacle positions on the map
        if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
            obstacle_map[grid_x][grid_y] = 1  # 1 represents an obstacle

    # Calculate grid width and height
    x_w = max_x - min_x + 1
    y_w = max_y - min_y + 1

    return obstacle_map, min_x, min_y, max_x, max_y, x_w, y_w


# def calc_distance_heuristic(vehicle, goal_x, goal_y, grid_size, resolution):
#     """
#     Calculate a distance heuristic map for A* path planning.

#     Parameters:
#     vehicle : Vehicle object with attributes necessary for generating an obstacle map.
#     goal_x, goal_y : Goal position in meters.
#     grid_size : Tuple (width, height) specifying the size of the grid map.
#     resolution : Grid resolution in meters.

#     Returns:
#     heuristic_map : A 2D numpy array where each cell contains the cost to reach the goal from that cell.
#     """

#     # Generate the obstacle map using the previously defined function
#     obstacle_map, min_x, min_y, max_x, max_y, x_w, y_w = calc_obstacle_map(vehicle, grid_size, resolution)

#     # Initialize the heuristic map with infinity values
#     heuristic_map = np.full((x_w, y_w), np.inf)

#     # Convert goal position to grid coordinates
#     goal_grid_x = int((goal_x - vehicle.position[0]) / resolution) + (grid_size[0] // 2)
#     goal_grid_y = int((goal_y - vehicle.position[1]) / resolution) + (grid_size[1] // 2)

#     # Compute the heuristic value (Euclidean distance to the goal) for each cell in the grid
#     for x in range(x_w):
#         for y in range(y_w):
#             if not obstacle_map[x][y]:  # Ignore cells with obstacles
#                 dx = goal_grid_x - x
#                 dy = goal_grid_y - y
#                 heuristic_map[x][y] = np.sqrt(dx**2 + dy**2) * resolution  # Distance in meters

#     return heuristic_map

def calc_distance_heuristic(vehicle, goal_x, goal_y, grid_size, resolution):
    """
    Calculate a heuristic for Hybrid A* path planning, considering both the distance and orientation.

    Parameters:
    - vehicle: Vehicle object with current state.
    - goal_position: Tuple (goal_x, goal_y) representing the goal's position in meters.
    - grid_size: Tuple (width, height) specifying the size of the grid map.
    - resolution: Grid resolution in meters.

    Returns:
    - heuristic_map: A 2D numpy array where each cell contains the heuristic value.
    """
    # Initialize heuristic map with high values
    heuristic_map = np.full(grid_size, np.inf)

    # Convert goal position to grid coordinates
    goal_grid_x = int((goal_x - vehicle.position[0]) / resolution) + (grid_size[0] // 2)
    goal_grid_y = int((goal_y - vehicle.position[1]) / resolution) + (grid_size[1] // 2)
    goal_orientation = math.atan2(goal_y - vehicle.position[1], goal_x - vehicle.position[0])

    for x in range(grid_size[0]):
        for y in range(grid_size[1]):
            # Calculate position in meters relative to vehicle
            pos_x = (x - grid_size[0] // 2) * resolution + vehicle.position[0]
            pos_y = (y - grid_size[1] // 2) * resolution + vehicle.position[1]
            # Distance component
            distance = math.sqrt((pos_x - goal_x)**2 + (pos_y - goal_y)**2)
            # Orientation component (simple approach: difference in orientation towards the goal)
            orientation_diff = abs(math.atan2(pos_y - goal_y, pos_x - goal_x) - goal_orientation)
            # Combine distance and orientation in the heuristic
            heuristic_map[x, y] = distance + orientation_diff

    return heuristic_map

class HybridANode:
    def __init__(self, position, orientation_euler_angles, g_cost, steering_angle, speed, parent=None):
        self.position = position[:2]
        self.yaw = orientation_euler_angles[2]
        self.g_cost = g_cost
        self.h_cost = None
        self.steering_angle = steering_angle
        self.speed = speed
        self.parent = parent

    def calculate_h_cost(self, goal):
        self.h_cost = np.linalg.norm(np.array(self.position) - np.array(goal.position))  # Update this line

    def calculate_total_cost(self):
        return self.g_cost + self.h_cost

def bicycle_model(node, delta, vehicle_dynamics):
    x, y = node.position
    theta = node.orientation_euler_angles
    L = vehicle_dynamics["wheel_base"]
    v = node.speed
    dt = vehicle_dynamics["time_step"]

    new_theta = theta + (v / L) * np.tan(delta) * dt
    new_x = x + v * np.cos(theta) * dt
    new_y = y + v * np.sin(theta) * dt

    return new_x, new_y, new_theta

def calculate_speed(steering_angle, vehicle_dynamics):
    if abs(steering_angle) > np.radians(15):
        return vehicle_dynamics["max_speed"] / 2
    return vehicle_dynamics["max_speed"]

def get_neighbors(node, vehicle_dynamics, lidar_data, goal_node):  # Add goal_node as a parameter
    neighbors = []
    for delta in np.linspace(-vehicle_dynamics["max_steering"], vehicle_dynamics["max_steering"], num=5):
        new_speed = calculate_speed(delta, vehicle_dynamics)
        new_x, new_y, new_theta = bicycle_model(node, delta, vehicle_dynamics)
        new_node = HybridANode((new_x, new_y), new_theta, node.g_cost + 1, delta, new_speed, node)
        new_node.calculate_h_cost(goal_node)  # Calculate h_cost for each neighbor
        if not is_collision(new_node, lidar_data, vehicle_dynamics):
            neighbors.append(new_node)
    return neighbors

def is_collision(node, lidar_data, vehicle_dynamics):
    vehicle = node.vehicle
    # Use vehicle's LIDAR data for collision detection
    vehicle.position = node.position  # Temporarily update vehicle's position for collision check
    vehicle.orientation_euler_angles[2] = node.orientation  # Update vehicle's orientation

    # Check if the vehicle is in a dead-end situation or too close to an obstacle
    if vehicle.is_dead_end() or vehicle.distance_to_nearest_obstacle() < 0.5:
        return True  # Collision detected

    return False  # No collision detected

def reconstruct_path(node):
    path = []
    while node:
        path.append(node)
        node = node.parent
    return path[::-1]

def distance_between(node1, node2):
    return np.linalg.norm(np.array(node1.position) - np.array(node2.position))

def hybrid_a_star(start, goal, lidar_data, vehicle_dynamics):
    open_set = set()
    closed_set = set()
    start_node = HybridANode(start[0], start[1], 0, 0, vehicle_dynamics["max_speed"])
    goal_node = HybridANode(goal[0], goal[1], 0, 0, vehicle_dynamics["max_speed"])
    start_node.calculate_h_cost(goal_node)  # Add this line
    open_set.add(start_node)

    while open_set:
        current_node = min(open_set, key=lambda o: o.calculate_total_cost())

        if (np.linalg.norm(np.array(current_node.position) - np.array(goal_node.position)) < vehicle_dynamics["position_tolerance"] and
            abs(current_node.orientation - goal_node.orientation) < vehicle_dynamics["orientation_tolerance"]):
            return reconstruct_path(current_node)

        open_set.remove(current_node)
        closed_set.add(current_node)

        for neighbor in get_neighbors(current_node, vehicle_dynamics, lidar_data, goal_node):  # Pass goal_node here
            if neighbor in closed_set:
                continue

            if neighbor in open_set:
                new_g_cost = current_node.g_cost + distance_between(current_node, neighbor)
                if neighbor.g_cost > new_g_cost:
                    neighbor.g_cost = new_g_cost
                    neighbor.parent = current_node
            else:
                neighbor.g_cost = current_node.g_cost + distance_between(current_node, neighbor)
                neighbor.calculate_h_cost(goal_node)
                open_set.add(neighbor)

    return None  # Path not found

def convert_pgm_to_grid(map_array, threshold=128):
    """
    Convert a .pgm map file to a grid usable for A* pathfinding.
    :param map_array: Numpy array representation of the .pgm map.
    :param threshold: Threshold to determine if a cell is occupied (default is 128 for grayscale).
    :return: A 2D grid for pathfinding.
    """
    grid = np.zeros_like(map_array, dtype=int)
    grid[map_array < threshold] = 1  # Mark cells as occupied if below the threshold
    return grid

def merge_grids(lidar_grid, map_grid):
    """
    Merge LIDAR-based grid and map-based grid.
    :param lidar_grid: Grid generated from LIDAR data.
    :param map_grid: Grid generated from the .pgm map.
    :return: Combined grid.
    """
    # Ensure both grids are of the same size
    if lidar_grid.shape != map_grid.shape:
        raise ValueError("Grid sizes do not match")

    # Combine the grids
    combined_grid = np.maximum(lidar_grid, map_grid)
    return combined_grid

import numpy as np
import cv2

def resize_grid(grid, new_size):
    print(f"Grid type: {type(grid)}")  # Debugging: Check grid type
    if not isinstance(grid, np.ndarray):
        raise ValueError("Grid is not a numpy array")
    if grid is None or grid.size == 0:
        raise ValueError("Grid is empty or None")
    return cv2.resize(grid, dsize=new_size, interpolation=cv2.INTER_NEAREST)

def visualize_grid(grid):
    """
    Visualize the grid.
    :param grid: The grid to visualize.
    """
    plt.imshow(grid, cmap='gray')
    plt.title('Merged Grid Visualization')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.draw()
    plt.pause(0.1)  # Short pause to update the plot
    plt.clf()

def merge_grids(lidar_grid, map_grid):
    # Ensure both grids are of the same size
    if lidar_grid.shape != map_grid.shape:
        # Resize the smaller grid to match the larger grid's size
        if lidar_grid.size < map_grid.size:
            lidar_grid = resize_grid(lidar_grid, map_grid.shape[::-1])
        else:
            map_grid = resize_grid(map_grid, lidar_grid.shape[::-1])

    # Combine the grids
    combined_grid = np.maximum(lidar_grid, map_grid)
    return combined_grid
