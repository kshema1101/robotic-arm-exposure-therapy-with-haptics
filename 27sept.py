import os
from import_ultrasonic import read_stored_value
import serial as ser
import numpy as np
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library
import socket
import time


MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430



# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_GOAL_VELOCITY          = 1
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    ADDR_PRESENT_POSITION       = 611
    ADDR_GOAL_VELOCITY          = 1
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_PRESENT_POSITION       = 580
    ADDR_GOAL_VELOCITY          = 1
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'XL320':
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_PRESENT_POSITION       = 37
    ADDR_GOAL_VELOCITY          = 1
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
    BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 30    # Dynamixel moving status threshold
DEVICENAME                  = '/dev/tty.usbserial-FT7W92VO'
index = 0
# Factory default ID of the first DYNAMIXEL
DXL_ID_1                    = 1

# Factory default ID of the second DYNAMIXEL
DXL_ID_2                    = 2
DXL_ID_3                    = 3
GOAL_VELOCITY = 8 
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Open port
waypoints=[]
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Define waypoints (motor positions)

#Set up the socket server
host = '127.0.0.1'  # Replace with your desired IP address
port = 57207  # Choose an available port

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the server address and port
server_socket.bind((host, port))

# Listen for incoming connections (you can specify the maximum number of connections)
server_socket.listen(5)

print(f"Server is listening on {host}:{port}")

# Accept incoming connections
client_socket, client_address = server_socket.accept()
print(f"Accepted connection from {client_address}")
# Set the current positions as the starting waypoint

trajectory_duration = 0.1 # Total duration of the trajectory in seconds
num_steps = 100
time_step = trajectory_duration / num_steps
# Number of interpolation steps between each pair of waypoints
interpolation_steps = 20  # Adjust this value for smoother movement

def interpolate(start, end, steps):
    interpolated_points = []
    for i in range(steps + 1):
        fraction = i / steps
        interpolated_point = start + fraction * (end - start)
        #interpolated_point = [s + fraction * (e - s) for s, e in zip(start, end)]
        
        interpolated_points.append(interpolated_point)
    return np.array(interpolated_points)

def emergency_stop_and_interpolate(portHandler, packetHandler):
    global emergency_stop_requested
    #message_to_send = "4"
    #client_socket.send(message_to_send.encode('utf-8'))
    #print(f"Sent to client: {message_to_send}")
    print("Emergency stop requested. Entering emergency stop mode...")

    # Define the target waypoint
    target_waypoint = (current_positions[0], 1634,current_positions[2]) 

    # Read the current positions of the motors
    
        # Set the position for each motor
    for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, target_waypoint[motor_id - 1])
                if dxl_comm_result != COMM_SUCCESS:
                    print("Failed to write goal position for motor ID:", motor_id)
                    quit()
        # Delay before the next step
        

    print("Emergency stop completed. Motors at the target waypoint.")
    quit()
    # Reset the flag after completing the emergency stop
    emergency_stop_requested = False

start_bit=0

# Disable the motors when 'd' is pressed
if  getch() == 'd':
    for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("Failed to disable torque for motor ID:", motor_id)
            quit()
    print("Motors disabled based on VR data (5).")
getch()

# Read current position and add it to waypoints when 'o' is pressed
if  getch() == 'o':
    start_bit = 1
    current_positions = [
        packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)[0]
        for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]
    ]
    current_positions = [
    packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)[0]
    for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]
]

# Set the current positions as the starting waypoint
    #waypoints = [tuple(current_positions), (1100, current_positions[1], current_positions[2])]
    waypoints = [tuple(current_positions), (current_positions[0]+300, current_positions[1], current_positions[2])]


print("wp1",waypoints[0])


print("Press  start in vr...")
#k_start=getch()
start_bit = client_socket.recv(1024).decode('utf-8')
print("start_bit from vr",start_bit)
while start_bit=='s' :

    last_adjusted_pos = list(waypoints[0])  # Convert tuple to list
    last_adjusted_pos_second_motor = last_adjusted_pos[1]  # Initialize with the initial position of the second motor

    # Loop through each pair of consecutive waypoints
    for i in range(len(waypoints) - 1):
        start_pos = np.array(waypoints[i])
        end_pos = np.array(waypoints[i + 1])

        

        # Calculate intermediate positions using interpolation
        intermediate_positions = interpolate(start_pos, end_pos, interpolation_steps)

        # Calculate velocity for the trajectory segment
        velocity = np.abs((end_pos - start_pos) / trajectory_duration)
        for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("Failed to enable torque for motor ID:", motor_id)
                quit()
            else:
                print("Enabled torque for motor ID:", motor_id)
        # Execute the trajectory

        for pos in intermediate_positions:
            stored_value = read_stored_value()
            print("Stored Value:", stored_value)  # Print the stored value

            # Adjust the position of the second motor based on the stored value
            if stored_value == 1:
                last_adjusted_pos_second_motor -= 4  # Update the adjusted position
                pos[1] = last_adjusted_pos_second_motor  # Decrease the second motor's position
                
    
                if stored_value == '8':
                    emergency_stop_and_interpolate(portHandler, packetHandler)
            elif stored_value == 3:
                last_adjusted_pos_second_motor += 10  # Update the adjusted position
                pos[1] = last_adjusted_pos_second_motor 
                if stored_value == '8':
                    
        # Trigger the emergency_stop function
                    emergency_stop_and_interpolate(portHandler, packetHandler)
            elif stored_value == 2:
                pos[1] = last_adjusted_pos_second_motor  # Stay at the last adjusted position
                
    
                if stored_value == '8' :
                    
        # Trigger the emergency_stop function
                    emergency_stop_and_interpolate(portHandler, packetHandler)
            # Limit the adjusted positions within the allowed range
            for j in range(3):
                pos[j] = np.clip(pos[j], DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)

            # Update the last adjusted position for the second motor
            last_adjusted_pos_second_motor = pos[1]
            print("Adjusted Position:", pos)
            if stored_value == '8':
                    
        # Trigger the emergency_stop function
                emergency_stop_and_interpolate(portHandler, packetHandler)
            for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, int(pos[motor_id - 1]))
                if dxl_comm_result != COMM_SUCCESS:
                    print("Failed to write goal position for motor ID:", motor_id)
                    
                    quit()
                    
                    

        while True:
                

                last_waypoint = waypoints[-1]  # Get the last waypoint from the list
                
                #print("last_waypoint",last_waypoint)
                pos_motor1, pos_motor2, pos_motor3 = [
                packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)[0]
                for motor_id in [DXL_ID_1, DXL_ID_2, DXL_ID_3]
                    ]
                #print(pos_motor1,pos_motor2,pos_motor3)
                #if (pos_motor1 == last_waypoint[0] and int(pos_motor3) == last_waypoint[2]):
                if (abs(pos_motor1 - last_waypoint[0]) <= DXL_MOVING_STATUS_THRESHOLD
                and abs(pos_motor3 - last_waypoint[2]) <= DXL_MOVING_STATUS_THRESHOLD
                and abs(pos_motor2 - pos[2])<= DXL_MOVING_STATUS_THRESHOLD):
                
            # Motors 1 and 3 have reached their positions and match the last waypoint
                    
                    start_bit=0
                    
                    quit()
                    break
        time.sleep(0.001)
client_socket.close()
server_socket.close()