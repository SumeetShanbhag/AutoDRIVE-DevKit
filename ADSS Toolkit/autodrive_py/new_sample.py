#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive
# Import the time module
import time

# Initialize a counter variable
data_counter = 0

# Initialize PID controller for steering control
Kp = 1.0   # Proportional gain
Ki = 0.1   # Integral gain
Kd = 0.01  # Derivative gain

previous_error = 0
integral = 0

# Initialize a start time variable
start_time = time.time()
################################################################################

# Initialize vehicle(s)
v_1 = autodrive.Vehicle()
v_1.id = 'V1'

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

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    global data_counter, start_time, previous_error, integral

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

        # Obstacle avoidance logic
        min_distance = 0.5  # Set the minimum distance for obstacle avoidance
        distance_to_obstacle = v_1.distance_to_nearest_obstacle()

        if distance_to_obstacle < min_distance:
            # Adjust steering to avoid the obstacle
            v_1.steering_command += 0.2  # For example, steer to the right

            # Make sure the steering command is within a valid range (-1 to 1)
            v_1.steering_command = max(-1, min(v_1.steering_command, 1))
            
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
        
        ########################################################################
        # CONTROL
        ########################################################################

        # Compute steering error
        error = desired_steering - v_1.orientation_euler_angles[0]

        # Compute PID terms
        proportional = Kp * error
        integral += Ki * error
        derivative = Kd * (error - previous_error)

        # Compute steering command using PID terms
        steering_cmd = proportional + integral + derivative

        # Apply steering limits
        steering_cmd = max(min(steering_cmd, 0.5), -0.5)  # Adjust the steering limits

        # Set the computed steering command
        v_1.steering_command = steering_cmd

        # Update previous error for the next iteration
        previous_error = error

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
