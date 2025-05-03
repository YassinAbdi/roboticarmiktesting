import network
import socket
import machine
from machine import Pin
import time
import math

# Wi-Fi credentials
SSID = "the realest"
PASSWORD = "assflAt1-"

# Setup Wi-Fi connection
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect(SSID, PASSWORD)

# Define constants for Motor 1
DIR1 = 19
STEP1 = 18
STEPS_PER_REV1 = 100 # Example value, adjust as needed

# Initialize pins for Motor 1
dir_pin1 = Pin(DIR1, Pin.OUT)
step_pin1 = Pin(STEP1, Pin.OUT)

# Define constants for Motor 2
DIR2 = 23
STEP2 = 22
STEPS_PER_REV2 = 100 # Example value, adjust as needed

# Initialize pins for Motor 2
dir_pin2 = Pin(DIR2, Pin.OUT)
step_pin2 = Pin(STEP2, Pin.OUT)

# Define constants for Motor 3 (Gripper)
DIR3 = 17
STEP3 = 16
STEPS_PER_REV3 = 100 # Example value, adjust as needed

# Initialize pins for Motor 3
dir_pin3 = Pin(DIR3, Pin.OUT)
step_pin3 = Pin(STEP3, Pin.OUT)

# Track current positions
current_position_1 = 0  # Start at step 0
current_position_2 = 0  # Start at step 0
current_position_3 = 0  # Start at step 0

# Generic step motor function (can be used for any motor by passing pins)
def step_motor_generic(step_pin, dir_pin, delay_us, direction, steps_to_take):
    dir_pin.value(direction)  # Set direction
    # print(f"Spinning {'Clockwise' if direction else 'Anti-Clockwise'} for {steps_to_take} steps...")

    for _ in range(steps_to_take):
        step_pin.value(1)
        time.sleep_us(delay_us)
        step_pin.value(0)
        time.sleep_us(delay_us)

# Specific step functions for clarity (can call generic)
def step_motor1(delay_us, direction, steps_to_take=STEPS_PER_REV1):
    step_motor_generic(step_pin1, dir_pin1, delay_us, direction, steps_to_take)

def step_motor2(delay_us, direction, steps_to_take=STEPS_PER_REV2):
    step_motor_generic(step_pin2, dir_pin2, delay_us, direction, steps_to_take)

def step_motor3(delay_us, direction, steps_to_take=STEPS_PER_REV3):
    step_motor_generic(step_pin3, dir_pin3, delay_us, direction, steps_to_take)


# Functions to move to a specific absolute position for each motor
def move_to_position1(target_position, step_delay_us=2000):
    """Move motor 1 to a specific absolute step position."""
    global current_position_1
    steps_needed = target_position - current_position_1
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    steps_needed = abs(steps_needed)

    print(f"Motor 1: Moving to position {target_position} ({'CW' if direction else 'CCW'}) {steps_needed} steps")
    step_motor1(step_delay_us, direction, steps_needed)

    current_position_1 = target_position
    print(f"Motor 1: Reached position: {current_position_1}")

def move_to_position2(target_position, step_delay_us=2000):
    """Move motor 2 to a specific absolute step position."""
    global current_position_2
    steps_needed = target_position - current_position_2
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    steps_needed = abs(steps_needed)

    print(f"Motor 2: Moving to position {target_position} ({'CW' if direction else 'CCW'}) {steps_needed} steps")
    step_motor2(step_delay_us, direction, steps_needed)

    current_position_2 = target_position
    print(f"Motor 2: Reached position: {current_position_2}")

def move_to_position3(target_position, step_delay_us=2000):
    """Move motor 3 to a specific absolute step position."""
    global current_position_3
    steps_needed = target_position - current_position_3
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    steps_needed = abs(steps_needed)

    print(f"Motor 3: Moving to position {target_position} ({'CW' if direction else 'CCW'}) {steps_needed} steps")
    step_motor3(step_delay_us, direction, steps_needed)

    current_position_3 = target_position
    print(f"Motor 3: Reached position: {current_position_3}")


# Motor and gear setup for arm (assuming Motors 1 and 2 control arm joints)
steps_per_rev = 200  # 1.8° stepper motor base resolution
microsteps = 16       # Microstepping factor
gear_ratio_base = 2.6
gear_ratio_arm1 = 2.6
gear_ratio_arm2 = 2.6
# Add gear ratio for gripper if needed
gear_ratio_gripper = 1 # Example, adjust if gripper has a gear

# Track the current target position of the arm (for inverse kinematics)
current_target_pos = [0, 0, 0]  # x, y, z

def angle_to_steps(angle, gear_ratio, motor_steps_per_rev=steps_per_rev, microstepping_factor=microsteps):
    """
    Convert angle (degrees) to stepper motor steps based on gear ratio and microstepping.
    """
    total_steps_per_rev_geared = motor_steps_per_rev * microstepping_factor * gear_ratio
    total_steps_per_deg = total_steps_per_rev_geared / 360
    steps = angle * total_steps_per_deg
    return int(steps)

def move_to_angle(b, a1, a2, g, step_delay_us=2000):
    """
    Convert target angles (b, a1, a2) and gripper value (g) to stepper motor steps
    and move motors simultaneously.
    Assumes b, a1, a2 are absolute angles in degrees.
    Assumes g is a target step position for the gripper motor (Motor 3).
    """
    global current_position_1, current_position_2, current_position_3

    # Calculate target step positions based on angles
    # These calculations assume a specific arm configuration and homing position
    # You may need to adjust the angle offsets (+90, -90, +101) based on your robot's geometry
    target_base_steps = angle_to_steps(b + 90, gear_ratio_base)
    target_arm1_steps = angle_to_steps(a1 - 90, gear_ratio_arm1)
    target_arm2_steps = angle_to_steps(a2 + 101, gear_ratio_arm2)
    target_gripper_steps = g # Assuming 'g' is already a target step position

    # Calculate steps needed from current position to target position
    steps_needed_1 = target_base_steps - current_position_1
    steps_needed_2 = target_arm1_steps - current_position_2
    steps_needed_3 = target_gripper_steps - current_position_3

    # Determine direction for each motor
    dir1 = 1 if steps_needed_1 > 0 else 0 # 1 for CW, 0 for CCW
    dir2 = 1 if steps_needed_2 > 0 else 0
    dir3 = 1 if steps_needed_3 > 0 else 0

    # Get absolute number of steps needed
    abs_steps_1 = abs(steps_needed_1)
    abs_steps_2 = abs(steps_needed_2)
    abs_steps_3 = abs(steps_needed_3)

    # Determine the maximum number of steps to move for simultaneous motion
    max_steps = max(abs_steps_1, abs_steps_2, abs_steps_3)

    print("Moving motors simultaneously:")
    print(f"  Motor 1 steps: {steps_needed_1} ({abs_steps_1} abs, {'CW' if dir1 else 'CCW'})")
    print(f"  Motor 2 steps: {steps_needed_2} ({abs_steps_2} abs, {'CW' if dir2 else 'CCW'})")
    print(f"  Motor 3 steps: {steps_needed_3} ({abs_steps_3} abs, {'CW' if dir3 else 'CCW'})")
    print(f"  Max steps for synchronization: {max_steps}")

    # Set directions before starting steps
    dir_pin1.value(dir1)
    dir_pin2.value(dir2)
    dir_pin3.value(dir3)

    # Perform simultaneous stepping
    for i in range(max_steps):
        if i < abs_steps_1:
            step_pin1.value(1)
        if i < abs_steps_2:
            step_pin2.value(1)
        if i < abs_steps_3:
            step_pin3.value(1)

        time.sleep_us(step_delay_us)

        step_pin1.value(0)
        step_pin2.value(0)
        step_pin3.value(0)

        time.sleep_us(step_delay_us)

    # Update current positions
    current_position_1 = target_base_steps
    current_position_2 = target_arm1_steps
    current_position_3 = target_gripper_steps

    print("Simultaneous move complete.")
    print(f"Reached positions: Motor 1: {current_position_1}, Motor 2: {current_position_2}, Motor 3: {current_position_3}")


def move_to_pos(x, y, z, g):
    """
    Compute joint angles to reach target position (x, y, z) and convert to motor steps.
    Uses inverse kinematics. Controls gripper with value 'g'.
    'g' is assumed to be a target step position for the gripper motor (Motor 3).
    """
    global current_target_pos

    print(f"Attempting to move to target position: x={x}, y={y}, z={z}, gripper={g}")

    # Use target pos to compute angles (Inverse Kinematics)
    # This IK assumes a 2-DOF arm in a vertical plane, with a base rotation.
    # Arm segment lengths are assumed to be 75 units each based on the original code's calculation.
    arm_length = 75 # Length of each arm segment

    # Calculate base angle (b)
    b = math.atan2(y, x) * (180 / math.pi) if x != 0 or y != 0 else current_target_pos[0] # Handle straight up case

    # Calculate horizontal distance from base to target projection
    l = math.sqrt(x**2 + y**2)

    # Calculate the height of the target relative to the base joint
    h = z # Assuming z is height above the base joint

    # Use the law of cosines to find the angles of the two arm segments
    # We have a triangle with sides arm_length, arm_length, and distance_to_target_projection
    distance_to_target_projection = math.sqrt(l**2 + h**2)

    # Check if target is reachable
    if distance_to_target_projection > (2 * arm_length):
        print("Warning: Target position is out of reach!")
        # Optionally, move to the closest reachable point or do nothing
        # For now, let's print a warning and proceed with potentially incorrect angles
        pass # Proceeding with calculations might result in invalid angles (NaN)

    # Angles within the arm triangle
    # Using Law of Cosines: c^2 = a^2 + b^2 - 2ab * cos(C)
    # cos(angle_between_arm_segments) = (arm_length^2 + arm_length^2 - distance_to_target_projection^2) / (2 * arm_length * arm_length)
    cos_theta = (arm_length**2 + arm_length**2 - distance_to_target_projection**2) / (2 * arm_length * arm_length)
    # Clamp the value to the valid range [-1, 1] to avoid math domain errors for acos
    cos_theta = max(min(cos_theta, 1), -1)
    theta = math.acos(cos_theta) * (180 / math.pi) # Angle between the two arm segments

    # Angle of the line from base to target projection relative to the horizontal
    phi = math.atan2(h, l) * (180 / math.pi) if l != 0 else (90 if h > 0 else (-90 if h < 0 else 0)) # Handle vertical case

    # Calculate joint angles (a1 and a2) relative to the horizontal
    a1 = phi + (theta / 2) # Angle of the first arm segment relative to horizontal
    a2 = phi - (theta / 2) # Angle of the second arm segment relative to horizontal

    print(f"Calculated angles (degrees):")
    print(f"  Base (Motor 1): {b:.2f}")
    print(f"  Arm1 (Motor 2, relative to horizontal): {a1:.2f}")
    print(f"  Arm2 (Motor ?, relative to horizontal): {a2:.2f}") # Note: Your current setup uses Motor 2 and 3 for arm segments, Base is Motor 1.
                                                                # Need to clarify which physical joint each motor controls.
                                                                # Assuming Motor 1 = Base, Motor 2 = Arm1, Motor 3 = Arm 2 for now based on pin assignments and order in previous code.

    # --- IMPORTANT ASSUMPTION ---
    # Based on your original code's `move_to_angle` and `move_to_pos`
    # It seems Motor 1 controls the base (angle b)
    # Motor 2 controls the first arm joint (angle a1)
    # Motor 3 is intended for the gripper or a third joint (angle a2, or the gripper value g)
    # The original `move_to_angle` calculated steps for base (b), arm1 (a1), and arm2 (a2) but only moved motors 1 and 2.
    # Let's correct this: Motor 1 -> Base (b), Motor 2 -> Arm1 (a1), Motor 3 -> Arm2 (a2)
    # The 'g' parameter will *override* the calculated a2 angle and be used for Motor 3.
    # If you want a2 to control the third motor, then the 'g' parameter should control something else (like a separate gripper motor if you have one).
    # I will proceed with using Motor 3 for the a2 joint angle calculated by IK, and ignore the 'g' parameter in `move_to_angle` when called from `move_to_pos`.
    # If you want 'g' to control a *separate* gripper motor (Motor 3), then `move_to_angle` needs to be adjusted.

    # Let's stick to the original intention of using 'g' for the gripper and using the calculated angles for the arm joints.
    # So, Motor 1 (Base) gets angle b, Motor 2 (Arm1) gets angle a1, and Motor 3 (Gripper) gets value g.
    # The `move_to_angle` function will be updated to handle this.

    # Call the function to move the motors based on the calculated angles and gripper value
    # The calculated a2 angle is NOT used if 'g' controls the gripper motor (Motor 3)
    move_to_angle(b, a1, a2, g) # Pass the calculated a2, but move_to_angle will use 'g' for motor 3

    # Update current target position after moving (for future IK calculations based on delta)
    current_target_pos[0] = x
    current_target_pos[1] = y
    current_target_pos[2] = z

# Wait for Wi-Fi connection
while not sta.isconnected():
    pass

print("Connected to Wi-Fi:", sta.ifconfig())

# Define GPIO pin (LED or other output device)
LED_PIN = 2  # Change this based on your board (GPIO2 = Onboard LED for ESP8266)
try:
    led = machine.Pin(LED_PIN, machine.Pin.OUT)
except ValueError:
    print(f"Warning: Pin {LED_PIN} is not available or invalid for output.")
    led = None # Set led to None if pin initialization fails
led_state = 0  # Track LED state

# State variables for web control
clockwise_state = 0
counterclockwise_state = 0
selected_motor = 1 # Default to Motor 1
x_pos = 0
y_pos = 0
z_pos = 0
gripper_value = 0 # Default gripper value

# Function to return HTML page with controls
def web_page():
    btn_state = "ON" if led_state else "OFF"
    btn2_state = "ON" if clockwise_state else "OFF"
    btn3_state = "ON" if counterclockwise_state else "OFF"

    html = f"""<!DOCTYPE html>
    <html>
    <head>
    <title>MicroPython Robot Arm Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ text-align: center; font-family: Arial, sans-serif; }}
        .button {{ padding: 10px 20px; font-size: 18px; margin: 5px; cursor: pointer; border-radius: 5px; }}
        .on {{ background-color: green; color: white; }}
        .off {{ background-color: red; color: white; }}
        .selected {{ background-color: blue; color: white; }}
        .section {{ margin-bottom: 20px; border: 1px solid #ccc; padding: 15px; border-radius: 8px; display: inline-block; }}
        .section h2 {{ margin-top: 0; }}
        input[type="number"] {{ padding: 8px; margin: 5px; border-radius: 4px; border: 1px solid #ccc; }}
        input[type="submit"] {{ padding: 10px 20px; font-size: 18px; margin: 5px; cursor: pointer; border-radius: 5px; background-color: #007bff; color: white; border: none; }}
        input[type="submit"]:hover {{ background-color: #0056b3; }}
    </style>
    </head>
    <body>
    <h1>ESP Robot Arm Control</h1>

    <div class="section">
        <h2>System</h2>
        <p>LED is <strong>{btn_state}</strong></p>
        <a href="/toggle"><button class="button {'on' if led_state else 'off'}">Toggle LED</button></a>
        <p>Current IK Target Pos: ({current_target_pos[0]:.2f}, {current_target_pos[1]:.2f}, {current_target_pos[2]:.2f})</p>
        <a href="/reset_ik_pos"><button class="button off">Reset IK Target Pos</button></a>
    </div>

    <div class="section">
        <h2>Motor Selection</h2>
        <p>Select motor for individual control:</p>
        <a href="/motor1"><button class="button {'selected' if selected_motor == 1 else ''}">Motor 1</button></a>
        <a href="/motor2"><button class="button {'selected' if selected_motor == 2 else ''}">Motor 2</button></a>
        <a href="/motor3"><button class="button {'selected' if selected_motor == 3 else ''}">Motor 3 (Gripper)</button></a>
        <p>Selected: <strong>Motor {selected_motor}</strong></p>
    </div>

    <div class="section">
        <h2>Individual Motor Control (Selected Motor {selected_motor})</h2>
        <a href="/clockwise"><button class="button">Step Clockwise</button></a>
        <a href="/antiwise"><button class="button">Step Anti-Clockwise</button></a>
        <p>Move selected motor to specific absolute step position:</p>
        <form action="/move_to" method="get">
            Target Step Position: <input type="number" name="position" min="0" max="10000" value="{current_position_1 if selected_motor == 1 else (current_position_2 if selected_motor == 2 else current_position_3)}" required>
            <input type="hidden" name="motor" value="{selected_motor}">
            <input type="submit" value="Move Motor {selected_motor}">
        </form>
        <p>Current Position (Motor 1): {current_position_1}</p>
        <p>Current Position (Motor 2): {current_position_2}</p>
        <p>Current Position (Motor 3): {current_position_3}</p>
    </div>

    <div class="section">
        <h2>Inverse Kinematics (Move to X, Y, Z)</h2>
        <p>Move arm to position (X, Y, Z) and set gripper (G - steps for Motor 3):</p>
        <form action="/smove_to_pos" method="get">
            X: <input type="number" name="x" placeholder="X" value="{current_target_pos[0]:.2f}" step="0.1" required>
            Y: <input type="number" name="y" placeholder="Y" value="{current_target_pos[1]:.2f}" step="0.1" required>
            Z: <input type="number" name="z" placeholder="Z" value="{current_target_pos[2]:.2f}" step="0.1" required>
            Gripper (Steps): <input type="number" name="g" placeholder="G" value="{current_position_3}" min="0" max="1000" required>
            <input type="submit" value="Move Arm">
        </form>
         <a href="/reset_arm_angles"><button class="button off">Reset Arm to 0/0/0 (angles)</button></a>
    </div>

     <div class="section">
        <h2>G-code Upload (Experimental)</h2>
        <p>Upload G-code file for execution:</p>
        <form method="POST" enctype="multipart/form-data" action="/upload">
            <input type="file" name="file">
            <input type="submit" value="Upload and Run">
        </form>
    </div>

    </body>
    </html>"""
    return html

# Setup the socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 80))
server_socket.listen(5)

print("Web server started...")

while True:
    conn, addr = server_socket.accept()
    print("Client connected from", addr)

    request = conn.recv(1024).decode()
    print("Request:", request)

    # Handle GET requests
    if "GET " in request:
        request_line = request.split('\r\n')[0]
        path = request_line.split(' ')[1]

        if path == "/toggle":
            print("Toggling LED")
            if led: # Check if led pin was initialized successfully
                led_state = not led_state  # Toggle state
                led.value(led_state)  # Update GPIO

        elif path == "/motor1":
            print("Selected Motor 1")
            selected_motor = 1
        elif path == "/motor2":
            print("Selected Motor 2")
            selected_motor = 2
        elif path == "/motor3":
            print("Selected Motor 3")
            selected_motor = 3

        elif path == "/clockwise":
            print(f"Stepping Motor {selected_motor} Clockwise")
            if selected_motor == 1:
                step_motor1(2000, 1, STEPS_PER_REV1 // 4) # Step a quarter revolution
            elif selected_motor == 2:
                step_motor2(2000, 1, STEPS_PER_REV2 // 4) # Step a quarter revolution
            elif selected_motor == 3:
                 step_motor3(2000, 1, STEPS_PER_REV3 // 4) # Step a quarter revolution


        elif path == "/antiwise":
            print(f"Stepping Motor {selected_motor} Anti-Clockwise")
            if selected_motor == 1:
                step_motor1(2000, 0, STEPS_PER_REV1 // 4) # Step a quarter revolution
            elif selected_motor == 2:
                step_motor2(2000, 0, STEPS_PER_REV2 // 4) # Step a quarter revolution
            elif selected_motor == 3:
                 step_motor3(2000, 0, STEPS_PER_REV3 // 4) # Step a quarter revolution


        elif path.startswith("/move_to"):
            print("Moving selected motor to specific position")
            # Extract position from request
            try:
                query_params = path.split('?')[1].split('&')
                position = int(query_params[0].split('=')[1])
                motor_param = [p for p in query_params if p.startswith('motor=')]
                selected_motor_for_move = int(motor_param[0].split('=')[1]) if motor_param else selected_motor # Use submitted motor or current selected

                print(f"Moving Motor {selected_motor_for_move} to position {position}")
                if selected_motor_for_move == 1:
                    move_to_position1(position)
                elif selected_motor_for_move == 2:
                    move_to_position2(position)
                elif selected_motor_for_move == 3:
                    move_to_position3(position)
                else:
                    print("Invalid motor specified for move_to")
            except (IndexError, ValueError) as e:
                print("Error parsing /move_to request:", e)


        elif path.startswith("/smove_to_pos"):
            print("Moving to specific x, y, z, g position (Inverse Kinematics)")
            try:
                # Extract x, y, z, g from request
                params = path.split('?')[1].split('&')
                x = float([p.split('=')[1] for p in params if p.startswith('x=')][0])
                y = float([p.split('=')[1] for p in params if p.startswith('y=')][0])
                z = float([p.split('=')[1] for p in params if p.startswith('z=')][0])
                g = int([p.split('=')[1] for p in params if p.startswith('g=')][0])

                print(f"Moving to position x={x}, y={y}, z={z}, gripper (steps)={g}")
                move_to_pos(x, y, z, g)

            except (IndexError, ValueError) as e:
                print("Error parsing /smove_to_pos request:", e)
                # Optionally send an error response
                conn.send("HTTP/1.1 400 Bad Request\n")
                conn.send("Content-Type: text/plain\n")
                conn.send("Connection: close\n\n")
                conn.sendall(f"Error parsing request: {e}".encode())
                conn.close()
                continue # Go to the next iteration

        elif path == "/current_pos":
             print("Returning current motor positions")
             response = f"Motor 1: {current_position_1}, Motor 2: {current_position_2}, Motor 3: {current_position_3}, IK Target: ({current_target_pos[0]:.2f}, {current_target_pos[1]:.2f}, {current_target_pos[2]:.2f})"
             conn.send("HTTP/1.1 200 OK\n")
             conn.send("Content-Type: text/plain\n")
             conn.send("Connection: close\n\n")
             conn.sendall(response.encode())
             conn.close()
             continue # Go to the next iteration

        elif path == "/reset_ik_pos":
            print("Resetting IK Target Position")
            current_target_pos = [0, 0, 0]

        elif path == "/reset_arm_angles":
             print("Attempting to reset arm to 0 angles (joint space)")
             # Move motors back to what corresponds to 0 angles for each joint
             # This requires knowing the step position that corresponds to 0 degrees for each motor.
             # This is often determined during a homing sequence.
             # For now, let's move them to step position 0 as a simple "reset" which might not be 0 degrees.
             # A proper homing function is needed for accurate angle resets.
             move_to_position1(0)
             move_to_position2(0)
             move_to_position3(0) # Reset gripper to 0 steps too


        # For any other GET request, serve the main page
        response = web_page()
        conn.send("HTTP/1.1 200 OK\n")
        conn.send("Content-Type: text/html\n")
        conn.send("Connection: close\n\n")
        conn.sendall(response.encode()) # Encode HTML response

    # Handle POST requests (specifically for file upload)
    elif "POST /upload" in request:
        print("Receiving file upload...")

        try:
            # Read until we hit the end of headers to find the boundary
            request_bytes = request.encode()
            body_start_index = request_bytes.find(b"\r\n\r\n") + 4
            header_part = request_bytes[:body_start_index].decode()

            content_type_header = [line for line in header_part.split('\r\n') if line.lower().startswith('content-type:')][0]
            boundary = content_type_header.split('boundary=')[1].strip()
            boundary_bytes = b"--" + boundary.encode()

            # Check if the file data is already in the initial request buffer
            file_data_start_in_buffer = request_bytes.find(b"\r\n\r\n") + 4
            initial_file_data = request_bytes[file_data_start_in_buffer:]

            # Find the start of the actual file content within the multipart data
            # Look for Content-Disposition header
            file_content_start_index = initial_file_data.find(b"\r\n\r\n") + 4
            if file_content_start_index > 3: # Ensure find was successful
                 initial_file_data = initial_file_data[file_content_start_index:]
            else:
                 print("Warning: Could not find file content start in initial buffer.")
                 # Try to read more data if content start wasn't in the first chunk
                 while b"\r\n\r\n" not in initial_file_data and len(initial_file_data) < 4096: # Read up to 4KB more to find headers
                      chunk = conn.recv(1024)
                      if not chunk: break
                      initial_file_data += chunk
                 file_content_start_index = initial_file_data.find(b"\r\n\r\n") + 4
                 if file_content_start_index > 3:
                      initial_file_data = initial_file_data[file_content_start_index:]
                 else:
                      print("Error: Could not find file content start.")
                      raise Exception("Could not parse file upload data.")


            # Define file path - create a directory if it doesn't exist
            try:
                 import os
                 if "gcode" not in os.listdir():
                      os.mkdir("gcode")
            except Exception as e:
                 print("Error creating gcode directory:", e)


            file_path = "/gcode/uploaded.gcode" # Save as a fixed name for simplicity

            with open(file_path, "wb") as f:
                 # Write the initial chunk of file data
                 # We need to be careful not to write the boundary at the end if it's in the initial chunk
                 end_boundary_index = initial_file_data.find(boundary_bytes)
                 if end_boundary_index != -1:
                      f.write(initial_file_data[:end_boundary_index - 2]) # -2 to remove trailing \r\n
                      print("File upload complete (single chunk).")
                 else:
                      f.write(initial_file_data)
                      print("Continuing to read file data...")
                      # Read remaining chunks until the boundary is found
                      while True:
                          chunk = conn.recv(1024)
                          if not chunk: break
                          end_boundary_index = chunk.find(boundary_bytes)
                          if end_boundary_index != -1:
                              f.write(chunk[:end_boundary_index - 2]) # -2 to remove trailing \r\n
                              print("File upload complete.")
                              break
                          f.write(chunk)

            print(f"File saved to {file_path}")

            # Optionally, add code here to process the uploaded G-code file
            # You would open the file, read line by line, parse commands (G0, G1, etc.)
            # and translate them into motor movements using move_to_pos or similar functions.
            # This is a complex task and not fully implemented here.

            # Respond to client
            conn.send("HTTP/1.1 200 OK\n")
            conn.send("Content-Type: text/html\n")
            conn.send("Connection: close\n\n")
            conn.sendall(b"<html><body><h1>File Upload Complete!</h1><p>G-code saved to /gcode/uploaded.gcode</p></body></html>")


        except Exception as e:
            print("Error during file upload:", e)
            conn.send("HTTP/1.1 500 Internal Server Error\n")
            conn.send("Content-Type: text/plain\n")
            conn.send("Connection: close\n\n")
            conn.sendall(f"Error processing file upload: {e}".encode())

    # Close the connection after handling the request
    conn.close()