#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive
# Import the time module
import time
import matplotlib.pyplot as plt
from autodrive import PIDController, calc_obstacle_map, convert_pgm_to_grid, merge_grids, calc_distance_heuristic
#from autodrive import hybrid_a_star
import numpy as np
import yaml
from PIL import Image
import matplotlib.colors as mcolors
# Initialize a counter variable
data_counter = 0

# Initialize PID controller for steering control
Kp = 1000   # Proportional gain
Ki = 0   # Integral gain
Kd = 0  # Derivative gain

# previous_error = 0
# integral = 0

# Vehicle dynamics parameters
vehicle_dynamics = {
    "wheel_base": 2.5,
    "max_steering": np.radians(30),
    "max_speed": 5,
    "time_step": 0.1,
    "position_tolerance": 0.5,
    "orientation_tolerance": np.radians(10)
}

# Visualization function in new_sample.py
def visualize_obstacle_map(obstacle_map, grid_size, resolution):
    plt.imshow(obstacle_map, cmap='gray', origin='lower', extent=[-grid_size[1]*resolution/2, grid_size[1]*resolution/2, -grid_size[0]*resolution/2, grid_size[0]*resolution/2])

    # Highlight the vehicle's position (marked with value 2) differently
    vehicle_y, vehicle_x = np.where(obstacle_map == 2)
    # Translate grid coordinates back to visualization coordinates
    vehicle_x = (vehicle_x - grid_size[1]//2) * resolution
    vehicle_y = (vehicle_y - grid_size[0]//2) * resolution

    plt.scatter(vehicle_x, vehicle_y, c='red', s=10, label='Vehicle')  # s sets the size of the marker
    plt.xlim(-grid_size[1]*resolution/2, grid_size[1]*resolution/2)
    plt.ylim(-grid_size[0]*resolution/2, grid_size[0]*resolution/2)
    plt.title("Obstacle Map with Vehicle Position")
    plt.xlabel("X-axis (meters)")
    plt.ylabel("Y-axis (meters)")
    plt.legend()
    plt.pause(0.1)
    plt.clf()

def load_map(yaml_file):
    with open(yaml_file, 'r') as file:
        map_data = yaml.safe_load(file)
    
    # Use an absolute path for troubleshooting
    pgm_file = '/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_py/my_map.pgm'
    
    resolution = map_data['resolution']
    origin = map_data['origin']
    map_img = Image.open(pgm_file)
    map_array = np.array(map_img)
    return map_array, resolution, origin

# def world_to_map(x_world, y_world, resolution, origin):
#     x_map = int((x_world - origin[0]) / resolution)
#     y_map = int((y_world - origin[1]) / resolution)
#     return x_map, y_map

def world_to_map(x_world, y_world, resolution, origin, map_height):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = map_height - int((y_world - origin[1]) / resolution) - 1
    return x_map, y_map

def visualize_heuristic_map(heuristic_map, goal_position=None):
    plt.clf()  # Clear the current figure. This ensures that you start fresh each time this function is called.
    
    cmap = plt.cm.coolwarm
    norm = mcolors.Normalize(vmin=np.min(heuristic_map), vmax=np.max(heuristic_map))
    plt.imshow(heuristic_map, cmap=cmap, norm=norm)
    
    if goal_position is not None:
        plt.plot(goal_position[0], goal_position[1], 'go', markersize=10, label='Goal')
    
    plt.title('Heuristic Map Visualization')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend(loc='upper right')
    cbar = plt.colorbar()
    cbar.set_label('Heuristic Cost to Goal')
    
    plt.pause(0.1)  # Pause to update the plot without blocking.

# def plot_vehicle_on_map(map_array, vehicle_x, vehicle_y, resolution, origin):
#     vehicle_x_map, vehicle_y_map = world_to_map(vehicle_x, vehicle_y, resolution, origin)
#     plt.imshow(map_array, cmap='gray', origin='lower')
#     plt.scatter(vehicle_x_map, vehicle_y_map, c='red', s=10, label='Vehicle')
#     plt.title("Vehicle on Map")
#     plt.xlabel("X")
#     plt.ylabel("Y")
#     plt.legend()
#     plt.draw()
#     plt.pause(0.1)  # Short pause to update the plot
#     plt.clf()  # Clear the plot so it's ready for the next update

def plot_vehicle_on_map(map_array, vehicle_x, vehicle_y, resolution, origin):
    # Flip the map_array vertically
    flipped_map_array = map_array[::-1]
    
    # Get the height of the map to adjust y-coordinates accordingly
    map_height = flipped_map_array.shape[0]
    
    # Calculate the correct map coordinates for the vehicle
    vehicle_x_map, vehicle_y_map = world_to_map(vehicle_x, vehicle_y, resolution, origin, map_height)
    
    plt.imshow(flipped_map_array, cmap='gray', origin='lower')
    plt.scatter(vehicle_x_map, vehicle_y_map, c='red', s=10, label='Vehicle')
    plt.title("Vehicle on Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.pause(0.1)
    plt.clf()

def visualize_combined_grid_with_vehicle(combined_grid, vehicle_position, resolution, origin, vehicle_marker_size=10):
    """
    Visualize the combined grid and mark the vehicle's position on it.

    :param combined_grid: The grid to visualize.
    :param vehicle_position: The vehicle's position in world coordinates (x, y).
    :param resolution: The map resolution (meters per grid cell).
    :param origin: The map origin (world coordinates of the bottom-left corner of the grid).
    :param vehicle_marker_size: The size of the marker representing the vehicle.
    """
    # Convert vehicle world coordinates to grid coordinates
    vehicle_x, vehicle_y = vehicle_position
    vehicle_grid_x = int((vehicle_x - origin[0]) / resolution)
    vehicle_grid_y = int((vehicle_y - origin[1]) / resolution)

    # Visualize the combined grid
    plt.imshow(combined_grid, cmap='gray', origin='lower')
    plt.colorbar(label='Occupancy')

    # Overlay the vehicle's position
    plt.scatter(vehicle_grid_x, vehicle_grid_y, c='red', s=vehicle_marker_size, label='Vehicle')

    # Add labels and legend
    plt.title('Combined Grid Visualization with Vehicle')
    plt.xlabel('Grid X')
    plt.ylabel('Grid Y')
    plt.legend()
    plt.draw()
    plt.pause(0.1)  # Short pause to update the plot
    plt.clf()  # Clear the plot so it's ready for the next update

# Initialize a start time variable
start_time = time.time()
################################################################################

# Initialize vehicle(s)
v_1 = autodrive.Vehicle()
v_1.id = 'V1'

pid_controller = PIDController(Kp, Ki, Kd)

# Initialize traffic light(s)
tl_1 = autodrive.TrafficLight()
tl_2 = autodrive.TrafficLight()
tl_3 = autodrive.TrafficLight()
tl_4 = autodrive.TrafficLight()
tl_1.id = 'TL1'
tl_2.id = 'TL2'
tl_3.id = 'TL3'
tl_4.id = 'TL4'

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__)

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Global variable to track if the vehicle is reversing
is_reversing = False
reverse_duration = 3  # Duration (in seconds) to reverse
reverse_start_time = None

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    global data_counter, start_time, previous_error, integral, is_reversing, reverse_start_time, v_1

    # Increment the data counter
    data_counter += 1

    if data:

        ########################################################################
        # PERCEPTION
        ########################################################################

        # Vehicle data
        v_1.parse_data(data, verbose=True)

        # Traffic light data
        tl_1.parse_data(data, verbose=True)
        tl_2.parse_data(data, verbose=True)
        tl_3.parse_data(data, verbose=True)
        tl_4.parse_data(data, verbose=True)

        
        '''
        Implement perception stack here.
        '''
        # v_1.update_grid_map(v_1.lidar_range_array)  # Update the grid map based on the LIDAR data
        # v_1.visualize_grid_map()
        # # Obstacle avoidance logic
        # min_distance = 0.5  # Set the minimum distance for obstacle avoidance
        # distance_to_obstacle = v_1.distance_to_nearest_obstacle()

        # if distance_to_obstacle < min_distance:
        #     # Adjust steering to avoid the obstacle
        #     v_1.steering_command += 0.2  # For example, steer to the right

        #     # Make sure the steering command is within a valid range (-1 to 1)
        #     v_1.steering_command = max(-1, min(v_1.steering_command, 1))

        # Obstacle avoidance logic
        v_1.avoid_obstacle(min_distance= 1.0)  # Use the method from the Vehicle class       

        ########################################################################
        # PLANNING
        ########################################################################
        
        # Basic planner: Collision avoidance and lane-following behavior
        desired_lane = 0  # Stay in the center lane
        lane_center = desired_lane * 2.5  # Example lane center for each lane
        desired_steering = lane_center - v_1.position[1]  # Difference from lane center
        
        # Collision avoidance
        min_distance = 2.0  # Minimum distance to maintain from obstacles
        obstacle_detected = False

        # Check if any obstacle is too close
        if v_1.distance_to_nearest_obstacle() < min_distance:
            obstacle_detected = True

        if obstacle_detected:
            desired_steering *= -1.0  # Steer away from the obstacle
        
        # Parse vehicle data
        # Define the grid size and resolution for the obstacle map
        goal_x, goal_y = -0.119, 1.375
        grid_size = (80, 80)  # Example values, adjust according to your simulation environment
        resolution = 0.05  # Example value in meters

        # Update the obstacle map
        # obstacle_map = calc_obstacle_map(v_1, grid_size, resolution)
        obstacle_map, min_x, min_y, max_x, max_y, x_w, y_w = calc_obstacle_map(v_1, grid_size, resolution)
        # visualize_obstacle_map(obstacle_map, grid_size, resolution)
        heuristic_map = calc_distance_heuristic(v_1, goal_x, goal_y, grid_size, resolution)
        goal_grid_x = int(goal_x / resolution) + grid_size[0] // 2
        goal_grid_y = int(goal_y / resolution) + grid_size[1] // 2

        # Visualize the heuristic map
        # visualize_heuristic_map(heuristic_map, goal_position=(goal_grid_x, goal_grid_y))

        #obstacle_map, min_x, min_y, max_x, max_y, x_w, y_w = calc_obstacle_map(v_1, grid_size, resolution)
        # visualize_obstacle_map(obstacle_map, grid_size, resolution)
        # Load the map data and convert it to a grid
        yaml_path = '/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_py/my_map.yaml'
        pgm_file = '/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_py/my_map.pgm'
        map_array, resolution, origin = load_map(yaml_path)
        map_grid = convert_pgm_to_grid(map_array)

        # Merge LIDAR-based grid and map-based grid
        combined_grid = merge_grids(obstacle_map, map_grid)

        # Define start and goal states for Hybrid A*
        # start = (v_1.position[:2], v_1.orientation_euler_angles[2])
        # goal = ((-0.913, 0.117), 0)  # Define goal position and orientation
        # print("Start orientation data:", start[1])
        # print("Goal orientation data:", goal[1])
        # Run Hybrid A* planner
        #path = hybrid_a_star(start, goal, v_1.lidar_range_array, vehicle_dynamics)

        # # Convert the path to steering and throttle commands
        # if path:
        #     for node in path:
        #         # Example: You can convert each node to steering and throttle commands
        #         # vehicle.steering_command = ...
        #         # vehicle.throttle_command = ...
        #         pass
        # else:
        #     print("Path not found")

        # Assuming v_1.position[0] and v_1.position[1] gives the X, Y coordinates of the vehicle
        vehicle_x = v_1.position[0]
        vehicle_y = v_1.position[1]

        # Plot the vehicle on the map
        plot_vehicle_on_map(map_array, vehicle_x, vehicle_y, resolution, origin)
        #visualize_combined_grid_with_vehicle(combined_grid, (vehicle_x, vehicle_y), resolution, origin)
        ########################################################################
        # CONTROL
        ########################################################################

        # # Check for dead-end situation
        # if v_1.is_dead_end():
        #     # Start reversing
        #     is_reversing = True
        #     reverse_start_time = time.time()
        #     v_1.throttle_command = -0.5  # Reverse throttle
        #     print("Dead-end detected! Reversing...")  # Print statement to confirm dead-end detection

        # # If the vehicle is reversing, check the duration
        # if is_reversing:
        #     if time.time() - reverse_start_time > reverse_duration:
        #         is_reversing = False  # Stop reversing after the duration
        #     else:
        #         v_1.throttle_command = -0.5  # Continue reversing
        #         return  # Ensure the reversing command is not overridden
        # else:
        #     v_1.throttle_command = 0.5  # Apply throttle to move forward only if not reversing

        print("Final throttle command:", v_1.throttle_command)
        # Compute steering error
        # error = desired_steering - v_1.orientation_euler_angles[0]

        # # Compute PID terms
        # proportional = Kp * error
        # integral += Ki * error
        # derivative = Kd * (error - previous_error)

        # # Compute steering command using PID terms
        # steering_cmd = proportional + integral + derivative

        # # Apply steering limits
        # steering_cmd = max(min(steering_cmd, 0.5), -0.5)  # Adjust the steering limits

        # # Set the computed steering command
        # v_1.steering_command = steering_cmd

        # # Update previous error for the next iteration
        # previous_error = error

        # Compute lateral error using detected lane lines
        # NOTE: You'll need to modify this to get the actual detected left and right lane lines
        left_detected_line = [0, 0, 0, 0]  # Placeholder, replace with actual detected line
        right_detected_line = [0, 0, 0, 0]  # Placeholder, replace with actual detected line

        lateral_error = v_1.compute_lateral_error(left_detected_line, right_detected_line, v_1.front_camera_image.shape[1])

        # Compute steering command using the PID controller
        steering_cmd = pid_controller.compute(lateral_error)

        # Apply steering limits
        steering_cmd = max(min(steering_cmd, 0.5), -0.5)  # Adjust the steering limits if necessary

        # Set the computed steering command
        v_1.steering_command = steering_cmd
        '''
        Implement control stack here.
        '''

        # Vehicle control
        v_1.throttle_command = 0.5  # Apply throttle to move forward
        v_1.headlights_command = 1  # Enable headlights
        v_1.indicators_command = 0  # Disable indicators

        # Traffic light control
        tl_1.command = 1  # Red state
        tl_2.command = 2  # Yellow state
        tl_3.command = 3  # Green state
        tl_4.command = 3  # Green state

        ########################################################################

        json_msg = v_1.generate_commands(verbose=True)  # Generate vehicle 1 message
        json_msg.update(tl_1.generate_commands(verbose=True))  # Append traffic light 1 message
        json_msg.update(tl_2.generate_commands(verbose=True))  # Append traffic light 2 message
        json_msg.update(tl_3.generate_commands(verbose=True))  # Append traffic light 3 message
        json_msg.update(tl_4.generate_commands(verbose=True))  # Append traffic light 4 message

        try:
            sio.emit('Bridge', data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app)  # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)  # Deploy as an eventlet WSGI server
