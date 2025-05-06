import network
import socket
import machine
from machine import Pin
import time
import math
import os # Import the os module for file operations

# Wi-Fi credentials
SSID = "the realest"
PASSWORD = "assflAt1-"

# Setup Wi-Fi connection
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect(SSID, PASSWORD)

# Define constants for Motor 1 (Base)
DIR1 = 19
STEP1 = 18
STEPS_PER_REV1 = 100 # Example value, adjust as needed

# Initialize pins for Motor 1
dir_pin1 = Pin(DIR1, Pin.OUT)
step_pin1 = Pin(STEP1, Pin.OUT)

# Define constants for Motor 2 (Arm 1)
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

# Track current positions (in steps)
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
def step_motor1(delay_us, direction, steps_to_take):
    step_motor_generic(step_pin1, dir_pin1, delay_us, direction, steps_to_take)

def step_motor2(delay_us, direction, steps_to_take):
    step_motor_generic(step_pin2, dir_pin2, delay_us, direction, steps_to_take)

def step_motor3(delay_us, direction, steps_to_take):
    step_motor_generic(step_pin3, dir_pin3, delay_us, direction, steps_to_take)


# Functions to move to a specific absolute position for each motor
def move_to_position1(target_position, step_delay_us=2000):
    """Move motor 1 (Base) to a specific absolute step position."""
    global current_position_1
    steps_needed = target_position - current_position_1
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    steps_needed = abs(steps_needed)

    if steps_needed > 0:
        print(f"Motor 1: Moving to position {target_position} ({'CW' if direction else 'CCW'}) {steps_needed} steps")
        step_motor1(step_delay_us, direction, steps_needed)
        current_position_1 = target_position
        print(f"Motor 1: Reached position: {current_position_1}")
    else:
        print(f"Motor 1: Already at target position {target_position}")


def move_to_position2(target_position, step_delay_us=2000):
    """Move motor 2 (Arm 1) to a specific absolute step position."""
    global current_position_2
    steps_needed = target_position - current_position_2
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    steps_needed = abs(steps_needed)

    if steps_needed > 0:
        print(f"Motor 2: Moving to position {target_position} ({'CW' if direction else 'CCW'}) {steps_needed} steps")
        step_motor2(step_delay_us, direction, steps_needed)
        current_position_2 = target_position
        print(f"Motor 2: Reached position: {current_position_2}")
    else:
         print(f"Motor 2: Already at target position {target_position}")


def move_to_position3(target_position, step_delay_us=2000):
    """Move motor 3 (Gripper) to a specific absolute step position."""
    global current_position_3
    steps_needed = target_position - current_position_3
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    steps_needed = abs(steps_needed)

    if steps_needed > 0:
        print(f"Motor 3: Moving to position {target_position} ({'CW' if direction else 'CCW'}) {steps_needed} steps")
        step_motor3(step_delay_us, direction, steps_needed)
        current_position_3 = target_position
        print(f"Motor 3: Reached position: {current_position_3}")
    else:
        print(f"Motor 3: Already at target position {target_position}")


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

def move_to_angle(b_deg, a1_deg, a2_deg, g_steps, step_delay_us=2000):
    """
    Convert target angles (b, a1, a2) to stepper motor steps for motors 1 and 2,
    and move motor 3 to the specified gripper step position (g_steps).
    Assumes b, a1, a2 are absolute angles in degrees.
    Assumes g_steps is a target step position for the gripper motor (Motor 3).
    """
    global current_position_1, current_position_2, current_position_3

    # Calculate target step positions based on angles
    # These calculations assume a specific arm configuration and homing position
    # You may need to adjust the angle offsets (+90, -90, +101) based on your robot's geometry
    target_base_steps = angle_to_steps(b_deg + 90, gear_ratio_base)
    target_arm1_steps = angle_to_steps(a1_deg - 90, gear_ratio_arm1)
    target_arm2_steps = angle_to_steps(a2_deg + 101, gear_ratio_arm2)
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
    print(f"  Motor 1 (Base) steps: {steps_needed_1} ({abs_steps_1} abs, {'CW' if dir1 else 'CCW'})")
    print(f"  Motor 2 (Arm1) steps: {steps_needed_2} ({abs_steps_2} abs, {'CW' if dir2 else 'CCW'})")
    print(f"  Motor 3 (Gripper) steps: {steps_needed_3} ({abs_steps_3} abs, {'CW' if dir3 else 'CCW'})")
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
    Uses inverse kinematics. Controls gripper with value 'g' (steps for Motor 3).
    """
    global current_target_pos

    print(f"Attempting to move to target position: x={x}, y={y}, z={z}, gripper={g}")

    # Use target pos to compute angles (Inverse Kinematics)
    # This IK assumes a 2-DOF arm in a vertical plane, with a base rotation.
    # Arm segment lengths are assumed to be 75 units each based on the original code's calculation.
    arm_length = 75 # Length of each arm segment

    # Calculate base angle (b)
    b = math.atan2(y, x) * (180 / math.pi) if x != 0 or y != 0 else 0 # Handle straight up case, default to 0 base angle

    # Calculate horizontal distance from base to target projection
    l = math.sqrt(x**2 + y**2)

    # Calculate the height of the target relative to the base joint
    h = z # Assuming z is height above the base joint

    # Calculate the distance from the base joint to the target point
    distance_to_target = math.sqrt(l**2 + h**2)

    # Check if target is reachable
    if distance_to_target > (2 * arm_length):
        print(f"Warning: Target position ({x}, {y}, {z}) is out of reach! Max reach is {2 * arm_length:.2f}")
        # Optionally, move to the closest reachable point or do nothing
        # For now, we can print a warning and proceed, which might result in invalid angles.
        # A better approach would be to clamp the distance or adjust the target.
        # For demonstration, we proceed, but be aware of potential math domain errors if distance > 2 * arm_length.
        distance_to_target = min(distance_to_target, 2 * arm_length - 0.001) # Clamp to max reach minus a small epsilon for safety

    # Use the law of cosines to find the angles within the arm triangle
    # We have a triangle with sides arm_length, arm_length, and distance_to_target
    # Let alpha be the angle between the two arm segments
    # distance_to_target^2 = arm_length^2 + arm_length^2 - 2 * arm_length * arm_length * cos(alpha)
    # cos(alpha) = (arm_length^2 + arm_length^2 - distance_to_target^2) / (2 * arm_length * arm_length)
    cos_alpha = (arm_length**2 + arm_length**2 - distance_to_target**2) / (2 * arm_length * arm_length)
    # Clamp the value to the valid range [-1, 1] for acos, in case of floating point inaccuracies near limits
    cos_alpha = max(min(cos_alpha, 1.0), -1.0)
    alpha = math.acos(cos_alpha) * (180 / math.pi) # Angle between the two arm segments in degrees

    # Angle of the line from base to target point relative to the horizontal
    gamma = math.atan2(h, l) * (180 / math.pi) if l != 0 else (90 if h > 0 else (-90 if h < 0 else 0)) # Handle vertical case

    # Calculate joint angles (a1 and a2) relative to the horizontal or base
    # Assuming a setup where a1 is the angle of the first arm relative to the base
    # and a2 is the angle of the second arm relative to the first arm.
    # This requires a different IK approach than the phi + theta/2 method used previously.

    # Let's use a standard 2-link planar arm IK approach relative to the base joint.
    # Assuming the first arm segment's angle (a1) is relative to the horizontal, and the second arm segment's angle (a2) is relative to the first arm segment.
    # However, the previous `move_to_angle` implies a1 and a2 are potentially relative to the horizontal.
    # Let's revert to the previous calculation method based on the structure of `move_to_angle`.
    # This calculation assumes a1 and a2 are angles relative to the base or horizontal, not relative to each other. This is likely incorrect for a standard robotic arm.

    # --- Correction based on typical robotic arm IK ---
    # Let theta1 be the angle of the first arm segment relative to the horizontal.
    # Let theta2 be the angle of the second arm segment relative to the first arm segment.
    # x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    # z = L1 * sin(theta1) + L2 * sin(theta1 + theta2) # Using z for vertical axis

    # We have distance_to_target = sqrt(l^2 + h^2). Let's use 'r' for this distance.
    # In the triangle formed by the two arm segments and the line to the target, we have sides L1, L2, and r.
    # Using Law of Cosines: r^2 = L1^2 + L2^2 - 2 * L1 * L2 * cos(theta2)
    # cos(theta2) = (L1^2 + L2^2 - r^2) / (2 * L1 * L2)
    # theta2 = acos((L1^2 + L2^2 - r^2) / (2 * L1 * L2))

    # Now find theta1. Use the angle of the target relative to the base (gamma)
    # and the angles within the arm triangle.
    # Using Law of Sines or a vector approach:
    # r / sin(alpha) = L2 / sin(angle_at_base_in_arm_triangle)
    # sin(angle_at_base_in_arm_triangle) = (L2 * sin(alpha)) / r
    # angle_at_base_in_arm_triangle = asin((L2 * sin(alpha)) / r)

    # This approach is more complex and requires careful handling of angle signs and quadrants.

    # Let's try to make the original approach in `move_to_pos` work with the existing `move_to_angle` structure.
    # The original `move_to_angle` seems to expect three absolute angles (base, arm1, arm2) which are then converted to steps.
    # It then uses these steps for motors 1 (base), 2 (arm1), and 3 (gripper).
    # The `move_to_pos` calculates `b`, `a1`, and `a2` using a simplified IK that might not match a standard arm.

    # Let's refine the IK based on the assumption that Motor 1 controls the base rotation (b),
    # Motor 2 controls the angle of the first arm segment relative to the base (a1),
    # and Motor 3 controls the angle of the second arm segment relative to the first (a2).
    # The 'g' parameter is then only for the gripper mechanism, which is separate from the arm's IK.

    # Recalculating IK assuming:
    # L1 = 75 (length of first arm segment)
    # L2 = 75 (length of second arm segment)
    # (x, y, z) is the target position relative to the base pivot.
    # Motor 1 controls base rotation (angle b).
    # Motor 2 controls the angle of L1 relative to the horizontal (let's call this angle `theta1`).
    # Motor 3 controls the angle of L2 relative to L1 (let's call this angle `theta2`).

    # Calculate base angle (b) - remains the same
    b = math.atan2(y, x) * (180 / math.pi) if x != 0 or y != 0 else 0

    # Project the target point onto the vertical plane containing the arm
    r_proj = math.sqrt(x**2 + y**2) # Horizontal distance from base
    z_target = z # Vertical distance from base

    # Calculate the angle of the line from base to target projection relative to the horizontal
    gamma = math.atan2(z_target, r_proj) * (180 / math.pi) if r_proj != 0 else (90 if z_target > 0 else (-90 if z_target < 0 else 0))

    # Calculate the distance from the base joint to the target point in the arm's plane
    distance_to_target = math.sqrt(r_proj**2 + z_target**2)

    # Check reachability again with the actual target distance
    if distance_to_target > (arm_length + arm_length):
         print(f"Warning: Target position ({x}, {y}, {z}) is out of reach! Max reach is {arm_length + arm_length:.2f}")
         # Clamp the distance for calculation purposes, but the physical arm won't reach
         distance_to_target = arm_length + arm_length - 0.001

    # Use the Law of Cosines to find theta2 (angle between arm segments)
    # distance_to_target^2 = L1^2 + L2^2 - 2 * L1 * L2 * cos(180 - theta2_internal)
    # The internal angle of the triangle at the second joint is 180 - theta2.
    # cos(180 - theta2) = (L1^2 + L2^2 - distance_to_target^2) / (2 * L1 * L2)
    # Since cos(180 - x) = -cos(x), we have:
    # -cos(theta2) = (L1^2 + L2^2 - distance_to_target^2) / (2 * L1 * L2)
    # cos(theta2) = -(L1^2 + L2^2 - distance_to_target^2) / (2 * L1 * L2)
    # cos_theta2_val = (arm_length**2 + arm_length**2 - distance_to_target**2) / (2 * arm_length * arm_length) # This was for the angle INSIDE the triangle
    # Let's calculate the internal angle first:
    cos_internal_angle_at_joint2 = (arm_length**2 + arm_length**2 - distance_to_target**2) / (2 * arm_length * arm_length)
    cos_internal_angle_at_joint2 = max(min(cos_internal_angle_at_joint2, 1.0), -1.0)
    internal_angle_at_joint2 = math.acos(cos_internal_angle_at_joint2) * (180 / math.pi)

    # If theta2 is the angle of the second arm segment relative to the first, and 0 is straight out,
    # then theta2 might be 180 - internal_angle_at_joint2 (for an elbow-down configuration).
    # Or it might be the internal angle itself depending on the mechanical setup and how the angle is defined.
    # Let's assume a simple elbow-down configuration where the internal angle is theta2.
    theta2 = internal_angle_at_joint2

    # Use the Law of Sines to find the angle at the base within the arm triangle
    # distance_to_target / sin(internal_angle_at_joint2) = L2 / sin(angle_at_base_in_triangle)
    # sin(angle_at_base_in_triangle) = (L2 * sin(internal_angle_at_joint2 * math.pi / 180)) / distance_to_target
    # Clamp the value for asin
    sin_angle_at_base = (arm_length * math.sin(internal_angle_at_joint2 * math.pi / 180)) / distance_to_target
    sin_angle_at_base = max(min(sin_angle_at_base, 1.0), -1.0)
    angle_at_base_in_triangle = math.asin(sin_angle_at_base) * (180 / math.pi)

    # Calculate theta1 (angle of the first arm segment relative to the horizontal)
    # theta1 = gamma + angle_at_base_in_triangle (for elbow up)
    # theta1 = gamma - angle_at_base_in_triangle (for elbow down)
    # Let's assume elbow-down configuration for now.
    theta1 = gamma - angle_at_base_in_triangle

    # a1 corresponds to theta1, a2 corresponds to theta2
    a1 = theta1
    a2 = theta2

    # NOTE: The calculated a2 here (angle between arm segments) is likely NOT
    # what the original `move_to_angle` was designed to use for Motor 3.
    # The original `move_to_angle` used a2 for calculating steps for motor 3,
    # implying motor 3 might be controlling a third joint or another degree of freedom,
    # not the angle between arm 1 and arm 2 in a standard 2-DOF arm.
    # Given the request to send angles (b, a1, a2) and a gripper value (g),
    # and the structure of `move_to_angle`, it's most probable that:
    # Motor 1 -> Base (b)
    # Motor 2 -> Arm 1 angle relative to horizontal (a1)
    # Motor 3 -> Gripper (g - step value)
    # The calculated 'a2' from IK in this function is *not* used for motor 3 movement
    # when called from `move_to_pos`. The 'g' parameter is used for motor 3.

    print(f"Calculated angles (degrees) using updated IK:")
    print(f"  Base (Motor 1): {b:.2f}")
    print(f"  Arm1 relative to horizontal (Motor 2): {a1:.2f}")
    # print(f"  Arm2 relative to Arm1 (calculated, not used for motor 3): {a2:.2f}")

    # Call the function to move the motors based on the calculated angles and gripper value
    # Use the calculated b and a1, and the provided g for the gripper.
    # The calculated a2 from IK is not used here.
    move_to_angle(b, a1, 0, g) # Pass 0 for a2 as it's not used for motor 3 in this interpretation

    # Update current target position after moving (for future IK calculations based on delta)
    current_target_pos[0] = x
    current_target_pos[1] = y
    current_target_pos[2] = z


# Wait for Wi-Fi connection
print("Connecting to Wi-Fi...")
while not sta.isconnected():
    time.sleep(0.1) # Wait a bit before checking again
print("Connected to Wi-Fi:", sta.ifconfig())

# Define GPIO pin (LED or other output device)
LED_PIN = 2  # Change this based on your board (GPIO2 = Onboard LED for ESP8266)
try:
    led = machine.Pin(LED_PIN, machine.Pin.OUT)
    print(f"Initialized LED pin on GPIO {LED_PIN}")
except ValueError:
    print(f"Warning: Pin {LED_PIN} is not available or invalid for output. LED control disabled.")
    led = None # Set led to None if pin initialization fails
led_state = 0  # Track LED state

# State variables for web control
clockwise_state = 0
counterclockwise_state = 0
selected_motor = 1 # Default to Motor 1
x_pos = 0
y_pos = 0
z_pos = 0
gripper_value = 0 # Default gripper value (steps)


# Function to return HTML page with controls
def web_page():
    btn_state = "ON" if led_state else "OFF"

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
        .section {{ margin-bottom: 20px; border: 1px solid #ccc; padding: 15px; border-radius: 8px; display: inline-block; min-width: 300px; }}
        .section h2 {{ margin-top: 0; }}
        input[type="number"], input[type="text"] {{ padding: 8px; margin: 5px; border-radius: 4px; border: 1px solid #ccc; width: 80px; }}
        input[type="submit"] {{ padding: 10px 20px; font-size: 18px; margin: 5px; cursor: pointer; border-radius: 5px; background-color: #007bff; color: white; border: none; }}
        input[type="submit"]:hover {{ background-color: #0056b3; }}
        label {{ margin-right: 10px; }}
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
        <a href="/motor1"><button class="button {'selected' if selected_motor == 1 else ''}">Motor 1 (Base)</button></a>
        <a href="/motor2"><button class="button {'selected' if selected_motor == 2 else ''}">Motor 2 (Arm 1)</button></a>
        <a href="/motor3"><button class="button {'selected' if selected_motor == 3 else ''}">Motor 3 (Gripper)</button></a>
        <p>Selected: <strong>Motor {selected_motor}</strong></p>
    </div>

    <div class="section">
        <h2>Individual Motor Control (Selected Motor {selected_motor})</h2>
        <p>Step selected motor (approx. 1/4 rev):</p>
        <a href="/clockwise"><button class="button">Step Clockwise</button></a>
        <a href="/antiwise"><button class="button">Step Anti-Clockwise</button></a>
        <p>Move selected motor to specific absolute step position:</p>
        <form action="/move_to" method="get">
            <label for="position">Target Step Position:</label>
            <input type="number" id="position" name="position" min="-10000" max="10000" value="{current_position_1 if selected_motor == 1 else (current_position_2 if selected_motor == 2 else current_position_3)}" required>
            <input type="hidden" name="motor" value="{selected_motor}">
            <input type="submit" value="Move Motor {selected_motor}">
        </form>
        <p>Current Position (Motor 1 Base): {current_position_1}</p>
        <p>Current Position (Motor 2 Arm 1): {current_position_2}</p>
        <p>Current Position (Motor 3 Gripper): {current_position_3}</p>
    </div>

    <div class="section">
        <h2>Move to Angle</h2>
        <p>Move arm to specific joint angles and set gripper step position:</p>
        <form action="/tmove_to_angle_cmd" method="get">
            <label for="base_angle">Base Angle (°):</label>
            <input type="number" id="base_angle" name="b" placeholder="Base (°)" value="0" step="0.1" required><br>
            <label for="arm1_angle">Arm 1 Angle (°):</label>
            <input type="number" id="arm1_angle" name="a1" placeholder="Arm 1 (°)" value="0" step="0.1" required><br>
            <label for="arm2_angle">Arm 2 Angle (°):</label>
            <input type="number" id="arm2_angle" name="a2" placeholder="Arm 2 (°)" value="0" step="0.1" required><br>
            <label for="gripper_steps">Gripper (Steps):</label>
            <input type="number" id="gripper_steps" name="g" placeholder="Gripper Steps" value="{current_position_3}" min="0" max="1000" required><br>
            <input type="submit" value="Move to Angles">
        </form>
         <a href="/reset_arm_angles"><button class="button off">Reset Arm Joint Steps (to 0)</button></a>
    </div>


    <div class="section">
        <h2>Inverse Kinematics (Move to X, Y, Z)</h2>
        <p>Move arm to position (X, Y, Z) relative to base, and set gripper (G - steps for Motor 3):</p>
        <form action="/smove_to_pos" method="get">
            <label for="x_pos">X:</label>
            <input type="number" id="x_pos" name="x" placeholder="X" value="{current_target_pos[0]:.2f}" step="0.1" required><br>
            <label for="y_pos">Y:</label>
            <input type="number" id="y_pos" name="y" placeholder="Y" value="{current_target_pos[1]:.2f}" step="0.1" required><br>
            <label for="z_pos">Z:</label>
            <input type="number" id="z_pos" name="z" placeholder="Z" value="{current_target_pos[2]:.2f}" step="0.1" required><br>
            <label for="gripper_ik">Gripper (Steps):</label>
            <input type="number" id="gripper_ik" name="g" placeholder="Gripper Steps" value="{current_position_3}" min="0" max="1000" required><br>
            <input type="submit" value="Move Arm (IK)">
        </form>
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

print("Web server started on port 80...")

# Ensure gcode directory exists
try:
    if "gcode" not in os.listdir():
        os.mkdir("gcode")
        print("Created directory: /gcode")
except Exception as e:
    print("Error ensuring /gcode directory exists:", e)


while True:
    try:
        conn, addr = server_socket.accept()
        print("Client connected from", addr)

        request = conn.recv(1024).decode()
        # print("Request:", request) # Uncomment for detailed request debugging

        # Handle GET requests
        if "GET " in request:
            request_line = request.split('\r\n')[0]
            method, path, _ = request_line.split(' ')

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
                steps_to_step = 50 # Example: step 50 steps for manual control
                if selected_motor == 1:
                    step_motor1(2000, 1, steps_to_step)
                    current_position_1 += steps_to_step
                elif selected_motor == 2:
                    step_motor2(2000, 1, steps_to_step)
                    current_position_2 += steps_to_step
                elif selected_motor == 3:
                     step_motor3(2000, 1, steps_to_step)
                     current_position_3 += steps_to_step


            elif path == "/antiwise":
                print(f"Stepping Motor {selected_motor} Anti-Clockwise")
                steps_to_step = 50 # Example: step 50 steps for manual control
                if selected_motor == 1:
                    step_motor1(2000, 0, steps_to_step)
                    current_position_1 -= steps_to_step
                elif selected_motor == 2:
                    step_motor2(2000, 0, steps_to_step)
                    current_position_2 -= steps_to_step
                elif selected_motor == 3:
                     step_motor3(2000, 0, steps_to_step)
                     current_position_3 -= steps_to_step


            elif path.startswith("/move_to"):
                print("Moving selected motor to specific position")
                try:
                    query_params_str = path.split('?')[1]
                    query_params = {}
                    for param in query_params_str.split('&'):
                        key_value = param.split('=')
                        if len(key_value) == 2:
                            query_params[key_value[0]] = key_value[1]

                    position = int(query_params.get('position', 0)) # Default to 0 if not found
                    selected_motor_for_move = int(query_params.get('motor', selected_motor)) # Use submitted motor or current selected

                    print(f"Moving Motor {selected_motor_for_move} to position {position}")
                    if selected_motor_for_move == 1:
                        move_to_position1(position)
                    elif selected_motor_for_move == 2:
                        move_to_position2(position)
                    elif selected_motor_for_move == 3:
                        move_to_position3(position)
                    else:
                        print("Invalid motor specified for move_to")
                except (IndexError, ValueError, AttributeError) as e:
                    print("Error parsing /move_to request:", e)

            elif path.startswith("/tmove_to_angle_cmd"):
                 print("Received Move to Angle command")
                 try:
                     query_params_str = path.split('?')[1]
                     query_params = {}
                     for param in query_params_str.split('&'):
                         key_value = param.split('=')
                         if len(key_value) == 2:
                             query_params[key_value[0]] = key_value[1]

                     # Extract and convert parameters
                     b_deg = float(query_params.get('b', 0))
                     a1_deg = float(query_params.get('a1', 0))
                     a2_deg = float(query_params.get('a2', 0)) # This value is received but not used for motor 3 control in move_to_angle
                     g_steps = int(query_params.get('g', 0))

                     print(f"Parsed angles: Base={b_deg}, Arm1={a1_deg}, Arm2(ignored)={a2_deg}, Gripper Steps={g_steps}")

                     # Call the move_to_angle function
                     move_to_angle(b_deg, a1_deg, a2_deg, g_steps)

                 except (IndexError, ValueError, AttributeError) as e:
                     print("Error parsing /move_to_angle_cmd request:", e)
                     # Optionally send an error response
                     conn.send("HTTP/1.1 400 Bad Request\n")
                     conn.send("Content-Type: text/plain\n")
                     conn.send("Connection: close\n\n")
                     conn.sendall(f"Error parsing request: {e}".encode())
                     conn.close()
                     continue # Go to the next iteration


            elif path.startswith("/smove_to_pos"):
                print("Moving to specific x, y, z, g position (Inverse Kinematics)")
                try:
                    query_params_str = path.split('?')[1]
                    query_params = {}
                    for param in query_params_str.split('&'):
                        key_value = param.split('=')
                        if len(key_value) == 2:
                            query_params[key_value[0]] = key_value[1]

                    # Extract and convert parameters
                    x = float(query_params.get('x', 0))
                    y = float(query_params.get('y', 0))
                    z = float(query_params.get('z', 0))
                    g = int(query_params.get('g', 0))

                    print(f"Moving to position x={x}, y={y}, z={z}, gripper (steps)={g}")
                    move_to_pos(x, y, z, g)

                except (IndexError, ValueError, AttributeError) as e:
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
                 response = f"Motor 1 (Base): {current_position_1}, Motor 2 (Arm 1): {current_position_2}, Motor 3 (Gripper): {current_position_3}, IK Target: ({current_target_pos[0]:.2f}, {current_target_pos[1]:.2f}, {current_target_pos[2]:.2f})"
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
                 print("Attempting to reset arm joint steps (to 0)")
                 # Move motors back to step position 0. This requires a homing sequence for true angle 0.
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
                # Read until the end of headers to find the boundary
                request_bytes = request.encode('utf-8') # Encode the initial request string
                header_end_index = request_bytes.find(b"\r\n\r\n")
                if header_end_index == -1:
                     raise Exception("Could not find end of headers in POST request.")

                header_part = request_bytes[:header_end_index].decode('utf-8')

                # Extract boundary from Content-Type header
                content_type_header = [line for line in header_part.split('\r\n') if line.lower().startswith('content-type:')][0]
                boundary = content_type_header.split('boundary=')[1].strip()
                boundary_bytes = b"--" + boundary.encode('utf-8')
                end_boundary_bytes = boundary_bytes + b"--" # The final boundary

                # Find the start of the file content body
                body_start_index = header_end_index + 4 # +4 for the two \r\n pairs

                # Define file path
                file_path = "/gcode/uploaded.gcode" # Save as a fixed name for simplicity

                print(f"Saving uploaded file to {file_path}")

                # Write the file data
                with open(file_path, "wb") as f:
                    # Write the initial chunk of the body that was already read
                    file_content_part = request_bytes[body_start_index:]

                    # We need to find the start of the actual file content within the multipart part
                    # Look for Content-Disposition and Content-Type headers for the file part
                    part_headers_end_index = file_content_part.find(b"\r\n\r\n")
                    if part_headers_end_index != -1:
                         file_content_part = file_content_part[part_headers_end_index + 4:] # Skip part headers

                    # Find the end of the file content within the initial chunk
                    end_of_file_in_initial_chunk = file_content_part.find(boundary_bytes)

                    if end_of_file_in_initial_chunk != -1:
                        # The entire file content is in the initial chunk
                        f.write(file_content_part[:end_of_file_in_initial_chunk - 2]) # -2 to remove trailing \r\n
                        print("File upload complete (single chunk).")
                    else:
                        # File content spans multiple chunks, write the initial part and read more
                        f.write(file_content_part)
                        print("Continuing to read file data...")
                        # Read remaining chunks until the boundary is found
                        while True:
                            chunk = conn.recv(1024)
                            if not chunk: break

                            # Check if this chunk contains the end boundary
                            end_index = chunk.find(end_boundary_bytes)
                            if end_index != -1:
                                # Write data up to the end boundary marker (minus the preceding \r\n)
                                f.write(chunk[:end_index - 2]) # -2 for the \r\n before the final boundary
                                print("File upload complete.")
                                break # Exit the reading loop

                            # Check if this chunk contains a boundary that's not the end boundary
                            # This would happen if there were multiple form fields
                            boundary_index = chunk.find(boundary_bytes)
                            if boundary_index != -1:
                                # Write data up to this boundary (minus the preceding \r\n)
                                f.write(chunk[:boundary_index - 2])
                                print("File upload part complete, found next boundary.")
                                break # Exit the reading loop (assuming only one file part)

                            f.write(chunk) # Write the whole chunk if no boundary found

                print(f"File saved to {file_path}")

                # --- G-code Parsing and Execution (Basic Example) ---
                print("Attempting to parse and execute G-code...")
                try:
                    with open(file_path, "r") as gcode_file:
                        for line in gcode_file:
                            line = line.strip()
                            if not line or line.startswith(';'):
                                continue # Skip empty lines and comments

                            print(f"Processing G-code line: {line}")

                            # Basic G-code parsing (simplified)
                            # Looks for G0 or G1 commands with X, Y, Z, and G parameters
                            parts = line.split()
                            command = parts[0].upper()

                            if command == 'G0' or command == 'G1':
                                target_x = current_target_pos[0]
                                target_y = current_target_pos[1]
                                target_z = current_target_pos[2]
                                target_g = current_position_3

                                for part in parts[1:]:
                                    if part.startswith('X'):
                                        try:
                                            target_x = float(part[1:])
                                        except ValueError:
                                            print(f"Warning: Invalid X value in G-code line: {line}")
                                    elif part.startswith('Y'):
                                         try:
                                            target_y = float(part[1:])
                                         except ValueError:
                                            print(f"Warning: Invalid Y value in G-code line: {line}")
                                    elif part.startswith('Z'):
                                         try:
                                            target_z = float(part[1:])
                                         except ValueError:
                                            print(f"Warning: Invalid Z value in G-code line: {line}")
                                    elif part.startswith('G'): # Assuming G parameter controls gripper steps
                                         try:
                                            # Check if it's a coordinate 'G' or a command 'G'
                                            if len(part) > 1 and part[1:].isdigit():
                                                # This is likely a command like G28, G90 etc. Handle these separately if needed.
                                                # For now, assuming a 'G' parameter for gripper steps is different.
                                                # Let's use a custom M-code or a dedicated parameter if possible in the G-code source.
                                                # For this example, let's assume a custom parameter like P or S for gripper steps
                                                # Or reinterpret Gx where x is not 0 or 1 as gripper steps (hacky)
                                                # A better approach is a custom M-code or specific G-code dialect support.
                                                pass # Ignore G commands here, handle X, Y, Z
                                         except ValueError:
                                            print(f"Warning: Invalid G value in G-code line: {line}")

                                print(f"Executing G-code move to X={target_x}, Y={target_y}, Z={target_z}, Gripper (last)={target_g}")
                                # Need to call move_to_pos with the extracted target values
                                # Note: The 'g' parameter in move_to_pos controls the gripper steps.
                                # The G-code standard doesn't typically have a 'G' coordinate for gripper steps.
                                # If your G-code generator uses a custom parameter (like E, S, or P) for gripper,
                                # you would parse that instead of 'G'.
                                # For this example, let's assume the G-code provides X, Y, Z, and we'll just use the *last known* gripper value.
                                # A more robust solution would be to look for a specific M-code or parameter for gripper.
                                # Let's add a placeholder for gripper control from G-code.
                                # For now, we only use X, Y, Z from G-code for IK and keep the current gripper position.
                                # If the G-code explicitly contains a gripper command (e.g., M command), parse that.

                                # Let's re-evaluate the G-code parsing. A common way to control grippers is via M-codes.
                                # Example: M10 command with parameters for gripper position.
                                # If your G-code uses M codes for the gripper, you would parse them here.
                                # Since the user provided 'g' in the IK form, let's assume G-code doesn't control the gripper via X,Y,Z,G.
                                # If the G-code is expected to control the gripper, we need to know the specific command/syntax.

                                # For now, let's just move based on X, Y, Z from G0/G1 and ignore gripper commands in G-code.
                                # If a gripper command is needed from G-code, define its format.

                                # Re-parsing for X, Y, Z only
                                move_x, move_y, move_z = current_target_pos # Start with current position

                                for part in parts[1:]:
                                    if part.startswith('X'):
                                        try:
                                            move_x = float(part[1:])
                                        except ValueError: pass
                                    elif part.startswith('Y'):
                                         try:
                                            move_y = float(part[1:])
                                         except ValueError: pass
                                    elif part.startswith('Z'):
                                         try:
                                            move_z = float(part[1:])
                                         except ValueError: pass
                                    # Add parsing for gripper command if known (e.g., M10 P<steps>)
                                    # elif part.startswith('M10 P'):
                                    #      try:
                                    #           gripper_steps_from_gcode = int(part[4:])
                                    #           target_g = gripper_steps_from_gcode
                                    #      except ValueError: pass


                                # Call move_to_pos with extracted X, Y, Z and the *current* gripper position
                                move_to_pos(move_x, move_y, move_z, current_position_3)

                                # Add a delay between G-code commands if needed
                                time.sleep(0.1) # Small delay between moves

                            # Add handling for other G-code commands like G28 (Homing), G90/G91 (Absolute/Relative), etc.
                            # This requires a full G-code interpreter implementation.

                except Exception as gcode_e:
                    print("Error processing G-code file:", gcode_e)


                # Respond to client after processing
                conn.send("HTTP/1.1 200 OK\n")
                conn.send("Content-Type: text/html\n")
                conn.send("Connection: close\n\n")
                conn.sendall(b"<html><body><h1>File Upload Complete!</h1><p>G-code saved to /gcode/uploaded.gcode</p><p>Attempted to run G-code.</p><a href=\"/\">Go Home</a></body></html>")


            except Exception as e:
                print("Error during file upload or G-code processing:", e)
                conn.send("HTTP/1.1 500 Internal Server Error\n")
                conn.send("Content-Type: text/plain\n")
                conn.send("Connection: close\n\n")
                conn.sendall(f"Error processing file upload or G-code: {e}".encode())


        # Close the connection after handling the request
        conn.close()

    except OSError as e:
        if e.args[0] == 110: # errno.ETIMEDOUT
            print("Connection timed out. Retrying accept()...")
            # Continue the loop to call accept() again
        else:
            print("Error in socket operation:", e)
            # Attempt to close and reopen socket if possible
            try:
                server_socket.close()
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.bind(('', 80))
                server_socket.listen(5)
                print("Socket reopened.")
            except Exception as reconnect_e:
                print("Failed to reopen socket:", reconnect_e)
                # Fatal error, break the loop or reset the device
                # machine.reset() # Uncomment to reset the microcontroller on fatal error
                break # Exit the loop

    except Exception as e:
        print("An unexpected error occurred:", e)
        # Handle other unexpected errors
        # machine.reset() # Uncomment to reset the microcontroller on unexpected error
        break # Exit the loop