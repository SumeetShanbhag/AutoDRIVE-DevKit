#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive

################################################################################

# Initialize vehicle(s)
v_1 = autodrive.Vehicle()
v_2 = autodrive.Vehicle()
v_3 = autodrive.Vehicle()
v_4 = autodrive.Vehicle()
v_1.id = 'V1'
v_2.id = 'V2'
v_3.id = 'V3'
v_4.id = 'V4'

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

# Initialize PID controller for steering control
Kp = 1.0   # Proportional gain
Ki = 0.1   # Integral gain
Kd = 0.01  # Derivative gain

previous_error = 0
integral = 0

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    global previous_error, integral

    if data:

        ########################################################################
        # PERCEPTION
        ########################################################################

        # Vehicle data
        v_1.parse_data(data, verbose=True)
        v_2.parse_data(data, verbose=True)
        v_3.parse_data(data, verbose=True)
        v_4.parse_data(data, verbose=True)

        # Traffic light data
        tl_1.parse_data(data, verbose=True)
        tl_2.parse_data(data, verbose=True)
        tl_3.parse_data(data, verbose=True)
        tl_4.parse_data(data, verbose=True)

        '''
        Implement perception stack here.
        '''

        ########################################################################
        # PLANNING
        ########################################################################

        '''
        Implement planning stack here.
        '''

        ########################################################################
        # CONTROL
        ########################################################################

        # Compute desired steering angle based on your logic
        desired_steering = 0.0  # Example desired steering angle

        # Compute steering error
        error = desired_steering - v_1.orientation_euler_angles[0]

        # Compute PID terms
        proportional = Kp * error
        integral += Ki * error
        derivative = Kd * (error - previous_error)

        # Compute steering command using PID terms
        steering_cmd = proportional + integral + derivative

        # Apply steering limits
        steering_cmd = max(min(steering_cmd, 1.0), -1.0)

        # Set the computed steering command for all vehicles
        v_1.steering_command = steering_cmd
        v_2.steering_command = steering_cmd
        v_3.steering_command = steering_cmd
        v_4.steering_command = steering_cmd

        # Update previous error for the next iteration
        previous_error = error

        '''
        Implement control stack here.
        '''

        # Vehicle control
        v_1.throttle_command = 0.5  # Apply throttle to move forward
        v_1.headlights_command = 1  # Enable headlights
        v_1.indicators_command = 0  # Disable indicators

        v_2.throttle_command = 0.5  # Apply throttle to move forward
        v_2.headlights_command = 1  # Enable headlights
        v_2.indicators_command = 0  # Disable indicators

        v_3.throttle_command = 0.5  # Apply throttle to move forward
        v_3.headlights_command = 1  # Enable headlights
        v_3.indicators_command = 0  # Disable indicators

        v_4.throttle_command = 0.5  # Apply throttle to move forward
        v_4.headlights_command = 1  # Enable headlights
        v_4.indicators_command = 0  # Disable indicators

        # Traffic light control
        tl_1.command = 1  # Red state
        tl_2.command = 2  # Yellow state
        tl_3.command = 3  # Green state
        tl_4.command = 3  # Green state

        ########################################################################

        json_msg = v_1.generate_commands(verbose=True)  # Generate vehicle 1 message
        json_msg.update(v_2.generate_commands(verbose=True))  # Append vehicle 2 message
        json_msg.update(v_3.generate_commands(verbose=True))  # Append vehicle 3 message
        json_msg.update(v_4.generate_commands(verbose=True))  # Append vehicle 4 message
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
