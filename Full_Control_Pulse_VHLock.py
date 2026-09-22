# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 12:23:55 2024

@author: Aiden Hussey
"""

# Import mavutil
from pymavlink import mavutil
import sys
import time
import math
from datetime import datetime
import csv
import numpy as np
import os

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.prev_time = None
        self.integral = 0
        self.P = 0
        self.I = 0
        self.D = 0
        self.prev_measurement = None
        self.filtered_derivative = 0.0
        self.d_tau = 0.3

    def compute(self, target, current, is_yaw=False):

        # Calculate error
        error = target - current

        if is_yaw:
            # Wrapping of error to compute fastest direction to target
            error = (error + 180) % 360 - 180

        # Read current time
        current_time = datetime.now()

        # Compute time difference
        if self.prev_time is None:
            delta_time = 0
        else:
            delta_time = (current_time - self.prev_time).total_seconds()

        # ---------------- P TERM ----------------
        self.P = self.kp * error

        # ---------------- I TERM ----------------
        # Use stored integral first
        self.I = self.ki * self.integral

        # ---------------- D TERM ----------------
        if delta_time > 0 and self.prev_measurement is not None:

            # Raw derivative of measurement
            if is_yaw:
                # Proper unwrap for yaw measurement change
                delta_measurement = (current - self.prev_measurement + 180) % 360 - 180
                measurement_rate = delta_measurement / delta_time
            else:
                measurement_rate = (current - self.prev_measurement) / delta_time

            # Low-pass filter on measurement rate
            alpha = self.d_tau / (self.d_tau + delta_time)
            self.filtered_derivative = (
                alpha * self.filtered_derivative
                + (1 - alpha) * measurement_rate
            )

            # Derivative on error = - derivative on measurement
            self.D = self.kd * (-self.filtered_derivative)

        else:
            self.D = 0

        # ---------------- OUTPUT ----------------
        control_speed = self.P + self.I + self.D

        # --------------- Clamp Output ---------------
        if control_speed > 1000:
            clamped_control_speed = 1000
        elif control_speed < -1000:
            clamped_control_speed = -1000
        else:
            clamped_control_speed = control_speed

        # --------------- Anti-Windup ---------------
        if delta_time > 0:
            saturated = (clamped_control_speed != control_speed)

            # Integrate if not saturated,
            # or if the error would pull the output back out of saturation
            if (not saturated) or (error * clamped_control_speed < 0):
                self.integral += error * delta_time

        # Update I term after possible integral change
        self.I = self.ki * self.integral

        # Save values for next iteration
        self.prev_error = error
        self.prev_time = current_time
        self.prev_measurement = current

        return clamped_control_speed
    
    def compute_hor(self, target_N, target_E, current_N, current_E):
        # Calculate error
        error = np.sqrt((target_N-current_N)**2 + (target_E-current_E)**2)
        
        angle = (math.atan2((target_E - current_E), (target_N - current_N))) * 180 / np.pi
        
        # Read current time
        current_time = datetime.now()
        # Compute time difference between measurements
        if self.prev_time is None:
            delta_time = 0
        else:
            delta_time = (current_time - self.prev_time).total_seconds()
        
        # Proportional term
        P = self.kp * error
        
        # Integral term
        if delta_time > 0:
            self.integral += error * delta_time  # Accumulate the integral
        I = self.ki * self.integral
        
        # Derivative term
        if delta_time > 0:
            D = self.kd * (error - self.prev_error) / delta_time
        else:
            D = 0  # No derivative calculation on the first call
        
        # Calculate the PID output (Control level)
        control_speed = P + I + D
        if control_speed > 1000:
            control_speed = 1000
        if control_speed < -1000:
            control_speed = -1000
        # Save error for the next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        return control_speed, angle
    
class PositionIMU:
    def __init__(self, timestamp):
        # Position is starting at (0, 0)
        self.pos_N = 0
        self.pos_E = 0
        # Start the system with no previous acceleration (drone frame)
        self.prev_a_x = 0
        self.prev_a_y = 0
        # Current acceleration from the IMU (drone frame)
        self.current_a_x = 0
        self.current_a_y = 0
        # Velocity of the drone (global frame)
        self.vel_N = 0
        self.vel_E = 0
        # Previous heading reading
        self.prev_heading_rad = 0
        # When the reading was taken
        self.prev_time = timestamp

    def update(self, a_x, a_y, timestamp, heading):

        heading_rad = math.radians(heading)

        delta_t = (timestamp - self.prev_time).total_seconds()
        self.prev_time = timestamp

        # Current acceleration in global North-East frame
        a_N = (
            a_x * math.cos(heading_rad)
            - a_y * math.sin(heading_rad)
            )

        a_E = (
            a_x * math.sin(heading_rad)
            + a_y * math.cos(heading_rad)
            )

        # Previous acceleration using previous heading
        prev_a_N = (
            self.prev_a_x * math.cos(self.prev_heading_rad)
            - self.prev_a_y * math.sin(self.prev_heading_rad)
            )

        prev_a_E = (
            self.prev_a_x * math.sin(self.prev_heading_rad)
            + self.prev_a_y * math.cos(self.prev_heading_rad)
            )

        # Save previous velocity
        prev_vel_N = self.vel_N
        prev_vel_E = self.vel_E

            # Acceleration -> velocity
        self.vel_N += 0.5 * (prev_a_N + a_N) * delta_t
        self.vel_E += 0.5 * (prev_a_E + a_E) * delta_t

        # Velocity -> position
        self.pos_N += 0.5 * (prev_vel_N + self.vel_N) * delta_t
        self.pos_E += 0.5 * (prev_vel_E + self.vel_E) * delta_t

        # Save current values for next iteration
        self.prev_a_x = a_x
        self.prev_a_y = a_y
        self.prev_heading_rad = heading_rad

        return self.pos_N, self.pos_E

    def get_position(self):
        return self.pos_N, self.pos_E

    def get_velocity(self):
        return self.vel_N, self.vel_E
    
    def get_acceleration(self):
        return self.current_a_x, self.current_a_y
          
def get_latest_AHRS2_imu(master):
    latest_AHRS2_message = None
    latest_imu_message = None
    
    while True:
        # Attempt to retrieve the next available message
        # blocking=False do not wait for a message if queue empty
        message = master.recv_match(blocking=False)
        if message is None:
            break  # No more messages in the queue, exit the loop
        
        # Check if message is one we want to read
        if message.get_type() == 'AHRS2':
            latest_AHRS2_message = message
        if message.get_type() == 'RAW_IMU':
            latest_imu_message = message

    return latest_AHRS2_message, latest_imu_message
 
def send_movement(master, hor_control_speed, vert_control_speed, rot_control_speed):
    #Horizontal control speed clamping
    if hor_control_speed>500:
        hor_control_speed = 500
    elif hor_control_speed<-500:
        hor_control_speed = -500           
        
    #Vertical control speed clamping
    if vert_control_speed>500:
        vert_control_speed = 500
    elif vert_control_speed<-500:
        vert_control_speed = -500
        
    #Rotational control speed clamping
    #Clockwise rotation
    if rot_control_speed>1000:
        rot_control_speed = 1000
    #Counter-clockwise rotation
    elif rot_control_speed<-1000:
        rot_control_speed = -1000
    elif 0<rot_control_speed<130:
        rot_control_speed = 130
    elif -130<rot_control_speed<0:
        rot_control_speed = -130
    
    #Send control command to drone
    master.mav.manual_control_send(
        master.target_system,
        0, #Controls forward and backward thrust motion
        int(hor_control_speed),
        int(500+vert_control_speed),
        int(rot_control_speed),
        0)

def angle_difference(angle1, angle2):
    # Compute the smallest difference between two angles, considering wrapping
    diff = (angle1 - angle2 + 180) % 360 - 180
    return diff

def pulse_plan(target_distance):
    # Function used to compute the optimal plan for movement using pulses based off of 
    # strength and distance profiles generated using SITL simulation
    pulse_distances = {
        500: 1.1,
        400: 0.95,
        300: 0.8,
        200: 0.6
    }

    # Pulse plan speed: # of pulses
    plan = {500: 0, 400: 0, 300: 0, 200: 0}
    remaining = target_distance

    # Try higher speeds first
    for speed in sorted(pulse_distances.keys(), reverse=True):
        pulse_dist = pulse_distances[speed]
        count = int(remaining // pulse_dist)
        if count > 0:
            plan[speed] = count
            remaining -= count * pulse_dist

    # Try to close any remaining gap using smaller pulses
    for speed in sorted(pulse_distances.keys(), reverse=False):
        pulse_dist = pulse_distances[speed]
        if remaining <= 0.01:
            break  # Close enough
        if remaining >= pulse_dist - 0.01:  # allow rounding margin
            plan[speed] += 1
            remaining -= pulse_dist

    # Compute final distance achieved
    total_distance = sum(plan[speed] * pulse_distances[speed] for speed in plan)
    final_leftover = round(target_distance - total_distance, 3)

    return plan, round(total_distance, 3), final_leftover

# Main Program Starts

# Define gravity constant used to convert scaled IMU measurements (m/s^2)
gravity_constant = 9.80665

# Define epsilon for acceleration control methods (this is the limit of acceleration concidered to be noise)
epsilon = 0

# Target N and E distances from starting position
target_N = 0
target_E = 0

# Ask how user wants to input mission data
input_method = input("Would you like to read mission data from a CSV file or enter it manually? (Enter 'csv' or 'manual'): ").strip().lower()

# Ask for custom file name for logging (this is needed in both modes)
file_name = input("Enter the custom file name for data logging (without extension): ").strip()
if file_name.lower().endswith(".csv"):
    file_name = file_name[:-4]
file_name += ".csv"

if input_method == "csv":
    csv_input_path = input("Enter the full path to the CSV file with mission data: ").strip()

    # Add .csv if missing
    if not csv_input_path.lower().endswith('.csv'):
        csv_input_path += '.csv'

    if not os.path.isfile(csv_input_path):
        print(f"File not found: {csv_input_path}")
        sys.exit(1)

    with open(csv_input_path, 'r', encoding='utf-8-sig') as mission_file:
        reader = csv.DictReader(mission_file)
        target_depths = []
        yaw_distances = []
        surface_flags = []
    
        for row in reader:
            depth = float(row['Depth (m)'].strip())
            heading = float(row['Heading (°)'].strip())
            distance = float(row['Distance (m)'].strip())
            surface = row['Surface (T/F)'].strip().lower() == 'true'
    
            target_depths.append(depth)
            yaw_distances.append((heading, distance))
            surface_flags.append(surface)
    
        yaw_iter = iter([y[0] for y in yaw_distances])
        distance_iter = iter([y[1] for y in yaw_distances])
        surface_iter = iter(surface_flags)

else:
    # Manual input mode
    depths_input = input("Enter the target depths for the mission (separated by spaces i.e. -2.5 -3 -3.5): ")
    target_depths = [float(depth) for depth in depths_input.split()]

    print('Heading is measured from -180 to 180 where 0 is North')
    yaw_distance_input = input("Enter a target heading followed by target distance (separated by spaces, i.e. 0 3 90 2.5 -180 -5.5): ")
    parse_yaw_distance = [float(data) for data in yaw_distance_input.split()]
    yaw_iter = iter(parse_yaw_distance[::2])
    distance_iter = iter(parse_yaw_distance[1::2])

    surface_input = input("Enter if you wish for the drone to surface after achieving depth, yaw and distance (separated by spaces, i.e. False True False): ")
    surface_iter = iter([value == "True" for value in surface_input.split()])


# Open a CSV file to record the depths
with open(file_name, mode='w', newline='') as file:
    csv_writer = csv.writer(file)
    # Write the header row
    csv_writer.writerow([
    'Timestamp', 'Latitude', 'Longitude','Roll [°])', 'Pitch [°]',  'Yaw [°]', 'Depth [m]', 
    'Hor control speed', 'Vert control speed', 'Rot control speed', 
    'Position North [m]', 'Position East [m]', 'Velocity North [m/s]', 'Velocity East [m/s]', 
    'Acceleration X [m/s^2]', 'Acceleration Y [m/s^2]', 
    'Hor integral', 'Hor prev_error', 
    'Vert integral', 'Vert prev_error', 
    'Rot integral', 'Rot prev_error', 
    'Hor Proportional Cont', 'Hor Integral Cont', 'Hor Derivative Cont',
    'Vert Proportional Cont', 'Vert Integral Cont', 'Vert Derivative Cont',
    'Rot Proportional Cont', 'Rot Integral Cont', 'Rot Derivative Cont',
    'Target Depth [m]', 'Target Yaw [°]'
])

    # Create the connection
    master = mavutil.mavlink_connection('udpin:172.27.144.1:14551')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    
    # Pick mode from below list:
    # mode_mapping_sub = {
    # 0: 'STABILIZE',
    # 1: 'ACRO',
    # 2: 'ALT_HOLD',
    # 3: 'AUTO',
    # 4: 'GUIDED',
    # 7: 'CIRCLE',
    # 9: 'SURFACE',
    # 16: 'POSHOLD',
    # 19: 'MANUAL',
    # }
    mode = 'MANUAL'

    # Check to see if guided exists as a valid mode
    if mode not in master.mode_mapping():
        print('Unknown mode: {}'.format(mode))
        print('Try:', list(master.mode_mapping.keys()))
        sys.exit(1)
    
    # Get mode ID from preset list
    mode_id = master.mode_mapping()[mode]
    print('mode ID is: ', mode_id)

    # Set new mode
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1, 19, 0, 0, 0, 0, 0)

    # Wait for mode change rersponse from system
    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
    
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break
    
    # Arm the ROV
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    # Wait for the motors to be armed
    master.motors_armed_wait()
    
    # Request position data stream to communication port
    master.mav.request_data_stream_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 
        10, 1)
    time.sleep(2)
    
    # Initialize the IMU position computation
    pos_imu = PositionIMU(datetime.now())
    
    # PID Control main loop
    for target_depth in target_depths:
        # Check if there is another target yaw if not set to 0
        try:
            target_yaw = next(yaw_iter)
        except StopIteration:
            target_yaw = 0
        
        # Check if there is another target distance otherwise stay in position
        try:
            target_distance = next(distance_iter)
            target_N += target_distance * math.cos(math.radians(target_yaw+90))
            target_E += target_distance * math.sin(math.radians(target_yaw+90))
            print('target distance is at: ({}, {})'.format(target_N, target_E))
        except StopIteration:
            target_distance = 0
            target_N += target_distance * math.cos(math.radians(target_yaw))
            target_E += target_distance * math.sin(math.radians(target_yaw))
            print('target distance is at: ({}, {})'.format(target_N, target_E))
            
        # Check if we have another surface instance if not we do surface
        try:
            target_surface = next(surface_iter)
        except StopIteration:
            target_surface = True
        
        
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # Initialize Horizontal PID controller N and E
        hor_pid = PIDController(kp=100, ki=0.25, kd=4)
        
        # Initialize Vertical PID controller
        vert_pid = PIDController(kp=600, ki=60, kd=50)
        
        # Initialize Rotational PID controller
        rot_pid = PIDController(kp=1000/180, ki=0, kd=0)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        hor_control_speed = 0
        hor_target_reached = False
        heading_counter = 0
            
        # Dive loop
        depth_hold_active = True
        depth_hold_counter = 0
        vert_target_reached = False
        sign_distance = (target_distance > 0) - (target_distance < 0)
        
        
        # Compute pulse plan for given distance
        if target_distance>0 or target_distance<0:
            plan, achieved_dist, leftover = pulse_plan(abs(target_distance))
        elif target_distance ==0:
            achieved_dist = 0
            leftover = 0
            plan = {500: 0, 400: 0, 300: 0, 200: 0}
            hor_target_reached = True
        
        # Ordered list of speeds and their pulse counts
        pulse_speeds = [500, 400, 300, 200]
        pulse_counts = [plan[speed] for speed in pulse_speeds]
        
        # Initialize pulse tracking
        pulse_index = 0  # Which speed we're on
        pulses_applied = 0  # How many complete pulses applied at current speed
        total_pulses = sum(pulse_counts) # Total number of pulses needed to reach target
        pulse_duration_iterations = 4  # 1 second of pulse control at 4 Hz
        pulse_active = False # Determines if we are currnetly in a pulse
        pulse_counter = 0 # Counts the number of times weve sent a control message to the drone
        pulse_pause_duration = 16 # Controls the length between pulses that we send
        pulse_pause_counter = 0 #Counts the iterations of the code that we have paused control
        pulse_pause = False # Determines if we are currently pausing our control
        
        # Wait for vehicle heartbeat before sending commands
        ensure_autopilot_heartbeat = (
            f'HEARTBEAT.get_srcSystem() == {master.target_system} and '
            f'HEARTBEAT.get_srcComponent() == {mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1}'
        )
        master.recv_match(type='HEARTBEAT', condition=ensure_autopilot_heartbeat, blocking=True)
        
        while depth_hold_active:
            
            latest_AHRS2_message, latest_imu_message = get_latest_AHRS2_imu(master)
            
            # Read the newest depth reading
            if latest_AHRS2_message:
                current_depth = latest_AHRS2_message.altitude
                current_roll = (latest_AHRS2_message.roll) - (-0.75*np.pi/180) #account for natural offset of IMU inside electronics enclosure
                current_pitch = (latest_AHRS2_message.pitch)
                current_yaw = (latest_AHRS2_message.yaw)
            else:
                print('No new AHRS2 message received')
            
            # Read current acceleration information
            if latest_imu_message:
                current_a_x = float(latest_imu_message.xacc) * gravity_constant /1000
                current_a_y = float(latest_imu_message.yacc) * gravity_constant /1000
                current_a_z = float(latest_imu_message.zacc) * gravity_constant /1000
            else:
                print('No new SCALED_IMU2 message received')
                
            # Convert accelerations to the world frame
            current_a_x = current_a_x*np.cos(current_pitch) + current_a_y*np.sin(current_pitch)*np.sin(current_roll)+current_a_z*np.sin(current_pitch)*np.cos(current_roll)
            if abs(current_a_x) <= epsilon: # If our current acceleration less then a certin value disregard it
                current_a_x = 0
            current_a_y = current_a_y*np.cos(current_roll)-current_a_z*np.sin(current_roll)
            if abs(current_a_y) <= epsilon: # If our current acceleration less then a certin value disregard it
                current_a_y = 0
            current_a_z = -current_a_x*np.sin(current_pitch)+current_a_y*np.cos(current_pitch)*np.sin(current_roll)+current_a_z*np.cos(current_pitch)*np.cos(current_roll)
            #print('Acceleration is: ', current_a_x, current_a_y, current_a_z)
            
            # Convert Euler angles to degrees
            current_roll = (current_roll*180/np.pi)
            current_pitch = (current_pitch*180/np.pi)
            current_yaw = (current_yaw*180/np.pi)
            
            #Wrap degrees to ensure within range of [-180, 180)
            current_roll = (current_roll + 180) % 360 - 180
            current_pitch = (current_pitch + 180) % 360 - 180
            current_yaw = (current_yaw + 180) % 360 - 180
            
            #Update the position calculation using the yaw and accelerations
            pos_imu.update(current_a_x, current_a_y, datetime.now(), current_yaw)
            
            #Check if we are within target range of our depth
            if current_depth <= (target_depth+0.05) and current_depth >= (target_depth-0.05):
                vert_target_reached = True
                depth_hold_counter += 1
                if depth_hold_counter >= 60 and hor_target_reached == True and vert_target_reached == True:
                    depth_hold_active = False
                    
            #Compute the vertical PID control value 
            vert_control_speed = vert_pid.compute(target_depth, current_depth)
            #print('Vertical control speed is: ', vert_control_speed)
            
            #Compute the rotation PID control value both target and current are in degrees
            rot_control_speed = rot_pid.compute(target_yaw, current_yaw, True)
            #print('Rotational control speed is: ', rot_control_speed)
            
            # Check heading within targeting range +/- 5 deg
            if abs(angle_difference(current_yaw, target_yaw)) <= 5 and vert_target_reached:
                heading_counter += 1
                print('Heading locked: ',heading_counter)
                if heading_counter >= 4 and not hor_target_reached:

                    # Handle pulse pause between pulses
                    if pulse_pause:
                        pulse_pause_counter += 1
                        hor_control_speed = 0
                        if pulse_pause_counter >= pulse_pause_duration:
                            pulse_pause = False
                            pulse_pause_counter = 0
                            print("Pause between pulses completed")
        
                    # If not in pause and not in a pulse, start a new pulse
                    elif not pulse_active and pulse_index < len(pulse_speeds):
                        if pulses_applied < pulse_counts[pulse_index]:
                            pulse_active = True
                            pulse_counter = 0
                            current_pulse_speed = pulse_speeds[pulse_index]
                            hor_control_speed = current_pulse_speed * sign_distance
                            rot_control_speed = 0
                            print(f"Pulse started at speed {current_pulse_speed}")
                        else:
                            # Move to next speed group
                            pulse_index += 1
                            pulses_applied = 0

                    elif pulse_active:
                        if pulse_counter < pulse_duration_iterations:
                            hor_control_speed = current_pulse_speed * sign_distance
                            rot_control_speed = 0
                            pulse_counter += 1
                            print(f"Pulse ongoing at {current_pulse_speed}... ({pulse_counter}/{pulse_duration_iterations})")
                        else:
                            hor_control_speed = 0
                            pulse_active = False
                            pulse_counter = 0
                            pulses_applied += 1
                            pulse_pause = True
                            print(f"Pulse ended at speed {current_pulse_speed} — starting pause")

                    if pulses_applied >= pulse_counts[pulse_index]:
                        pulse_index += 1
                        pulses_applied = 0

                    if pulse_index >= len(pulse_speeds):
                        hor_target_reached = True
                        print("All pulses completed — target reached")

            else:
                hor_control_speed = 0
                heading_counter = 0
            
            
            send_movement(master, hor_control_speed, vert_control_speed, rot_control_speed)
            
            if latest_AHRS2_message and latest_imu_message:
                csv_writer.writerow([
                    vert_pid.prev_time, latest_AHRS2_message.lat, latest_AHRS2_message.lng,
                    current_roll, current_pitch, current_yaw, current_depth, 
                    hor_control_speed, vert_control_speed, 
                    rot_control_speed, pos_imu.pos_N, pos_imu.pos_E, pos_imu.vel_N, pos_imu.vel_E, 
                    current_a_x, current_a_y, 
                    hor_pid.integral, hor_pid.prev_error, 
                    vert_pid.integral, vert_pid.prev_error, 
                    rot_pid.integral, rot_pid.prev_error,
                    hor_pid.P, hor_pid.I, hor_pid.D,
                    vert_pid.P, vert_pid.I, vert_pid.D,
                    rot_pid.P, rot_pid.I, rot_pid.D, target_depth, target_yaw
                    ])
            else:
                csv_writer.writerow([
                    vert_pid.prev_time, 0, 0, current_roll, current_pitch, current_yaw, 
                    current_depth, hor_control_speed, vert_control_speed, rot_control_speed, 
                    pos_imu.pos_N, pos_imu.pos_E, pos_imu.vel_N, pos_imu.vel_E, 
                    current_a_x, current_a_y, 
                    hor_pid.integral, hor_pid.prev_error, 
                    vert_pid.integral, vert_pid.prev_error, 
                    rot_pid.integral, rot_pid.prev_error,
                    hor_pid.P, hor_pid.I, hor_pid.D,
                    vert_pid.P, vert_pid.I, vert_pid.D,
                    rot_pid.P, rot_pid.I, rot_pid.D, target_depth, target_yaw
                    ])
            time.sleep(1/4)
            
        # Surface loop
        surface_active = True
        # Target 10cm from surface
        surface_depth = -0.5
        
        # wait for vehicle heartbeat before sending commands
        ensure_autopilot_heartbeat = (
            f'HEARTBEAT.get_srcSystem() == {master.target_system} and '
            f'HEARTBEAT.get_srcComponent() == {mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1}'
        )
        master.recv_match(type='HEARTBEAT', condition=ensure_autopilot_heartbeat, blocking=True)
            
        while surface_active and target_surface == True:
            target_depth = 0
            
            latest_AHRS2_message, latest_imu_message = get_latest_AHRS2_imu(master)
            
            # Read the newest depth reading
            if latest_AHRS2_message:
                current_depth = latest_AHRS2_message.altitude
                current_roll = (latest_AHRS2_message.roll) - (-0.75*np.pi/180)
                current_pitch = (latest_AHRS2_message.pitch)
                current_yaw = (latest_AHRS2_message.yaw)
            else:
                print('No new AHRS2 message received')
            
            # Read current acceleration information
            if latest_imu_message:
                current_a_x = float(latest_imu_message.xacc) * gravity_constant /1000
                current_a_y = float(latest_imu_message.yacc) * gravity_constant /1000
                current_a_z = float(latest_imu_message.zacc) * gravity_constant /1000
            else:
                print('No new SCALED_IMU2 message received')
                
            # Convert accelerations to the world frame
            current_a_x = current_a_x*np.cos(current_pitch) + current_a_y*np.sin(current_pitch)*np.sin(current_roll)+current_a_z*np.sin(current_pitch)*np.cos(current_roll)
            if abs(current_a_x) <= epsilon:
                current_a_x = 0
            current_a_y = current_a_y*np.cos(current_roll)-current_a_z*np.sin(current_roll)
            if abs(current_a_y) <= epsilon:
                current_a_y = 0
            current_a_z = -current_a_x*np.sin(current_pitch)+current_a_y*np.cos(current_pitch)*np.sin(current_roll)+current_a_z*np.cos(current_pitch)*np.cos(current_roll)
            #print('Acceleration is: ', current_a_x, current_a_y, current_a_z)
            
            # Convert Euler angles to degrees
            current_roll = (current_roll*180/np.pi)
            current_pitch = (current_pitch*180/np.pi)
            current_yaw = (current_yaw*180/np.pi)
            
            pos_imu.update(current_a_x, current_a_y, datetime.now(), current_yaw)
            #print('Current position is: ', pos_imu.pos_N, pos_imu.pos_E)
           
            hor_control_speed = 0
            #print('Horizontal control speed is: ', hor_control_speed)
            
            vert_control_speed = vert_pid.compute(surface_depth, current_depth)
            #print('Vertical control speed is: ', vert_control_speed)
            
            rot_control_speed = rot_pid.compute(target_yaw, current_yaw, True)
            #print('Rotational control speed is: ', rot_control_speed)
            
            send_movement(master, hor_control_speed, vert_control_speed, rot_control_speed)
                
            if current_depth >= surface_depth:
                surface_active = False
            
            if latest_AHRS2_message and latest_imu_message:
                csv_writer.writerow([
                    vert_pid.prev_time, latest_AHRS2_message.lat, latest_AHRS2_message.lng,
                    current_roll, current_pitch, current_yaw, current_depth, 
                    hor_control_speed, vert_control_speed, 
                    rot_control_speed, pos_imu.pos_N, pos_imu.pos_E, pos_imu.vel_N, pos_imu.vel_E, 
                    current_a_x, current_a_y, 
                    hor_pid.integral, hor_pid.prev_error, 
                    vert_pid.integral, vert_pid.prev_error, 
                    rot_pid.integral, rot_pid.prev_error,
                    hor_pid.P, hor_pid.I, hor_pid.D,
                    vert_pid.P, vert_pid.I, vert_pid.D,
                    rot_pid.P, rot_pid.I, rot_pid.D, target_depth, target_yaw
                    ])
            else:
                csv_writer.writerow([
                    vert_pid.prev_time, 0, 0, current_roll, current_pitch, current_yaw, 
                    current_depth, hor_control_speed, vert_control_speed, rot_control_speed, 
                    pos_imu.pos_N, pos_imu.pos_E, pos_imu.vel_N, pos_imu.vel_E, 
                    current_a_x, current_a_y, 
                    hor_pid.integral, hor_pid.prev_error, 
                    vert_pid.integral, vert_pid.prev_error, 
                    rot_pid.integral, rot_pid.prev_error,
                    hor_pid.P, hor_pid.I, hor_pid.D,
                    vert_pid.P, vert_pid.I, vert_pid.D,
                    rot_pid.P, rot_pid.I, rot_pid.D, target_depth, target_yaw
                    ])
            time.sleep(1/4)
            
            #reset the conditions of each pid controller at the end of each target waypoint
            hor_pid.integral = 0
            hor_pid.prev_error = 0
            rot_pid.integral = 0
            rot_pid.prev_error = 0
            vert_pid.integral = 0
            vert_pid.prev_error = 0
            
        time.sleep(1/4)
                
    
    
# Mission complete exiting control loop
print('Control system demonstration complete')
print('Motors turning off')

# Turn off motors command
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    500,
    0,
    0)

print('Waiting to disarm ROV')

# Disarm the ROV when mission is complete
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
# Wait for the motors to be disarmed
master.motors_disarmed_wait()
print('Motors are disarmed')
print(f'Depth readings saved to {file_name}')

constants_file_name = file_name.replace(".csv", "_constants.txt")


with open(constants_file_name, mode='w') as const_file:

    const_file.write("=== RUN CONSTANTS ===\n\n")

    const_file.write("Horizontal PID Gains:\n")
    const_file.write(f"Kp: {hor_pid.kp}\n")
    const_file.write(f"Ki: {hor_pid.ki}\n")
    const_file.write(f"Kd: {hor_pid.kd}\n\n")

    const_file.write("Vertical PID Gains:\n")
    const_file.write(f"Kp: {vert_pid.kp}\n")
    const_file.write(f"Ki: {vert_pid.ki}\n")
    const_file.write(f"Kd: {vert_pid.kd}\n\n")

    const_file.write("Rotational PID Gains:\n")
    const_file.write(f"Kp: {rot_pid.kp}\n")
    const_file.write(f"Ki: {rot_pid.ki}\n")
    const_file.write(f"Kd: {rot_pid.kd}\n\n")

    const_file.write("Other Parameters:\n")
    const_file.write(f"Gravity Constant: {gravity_constant} m/s^2\n")
    const_file.write("Loop Delay: 4 Hz\n")
    const_file.write(f"Target Depths: {target_depths} m\n")
    const_file.write(f"Target Heading and Distance: {parse_yaw_distance}")
    

print(f'Constants saved to {constants_file_name}')
