# Common imports available in both environments
import socket
import time
import math
import sys
import os # Added for CPython path handling

# --- Environment Detection and Conditional Setup ---
IS_MICROPYTHON = hasattr(sys, 'implementation') and sys.implementation.name == 'micropython'

if IS_MICROPYTHON:
    import network
    from machine import Pin
    # MicroPython specific sleep_us
    sleep_us = time.sleep_us
    # MicroPython might have a specific filesystem structure
    GCODE_UPLOAD_PATH = "/gcode" 
else:
    # --- Dummy Implementations for CPython ---
    print("Running in CPython mode. Hardware interactions will be simulated.")

    class DummyPin:
        """Simulates machine.Pin for CPython."""
        OUT = 1 # Dummy constant
        IN = 0  # Dummy constant
        _value = 0
        _pin_num = -1
        _mode = -1

        def __init__(self, pin_num, mode=OUT, *args, **kwargs):
            self._pin_num = pin_num
            self._mode = mode
            print(f"Simulated Pin({pin_num}, mode={'OUT' if mode == self.OUT else 'IN'}) created.")

        def value(self, val=None):
            if val is None:
                # Getting value
                return self._value
            else:
                # Setting value
                if val != self._value:
                    self._value = val
                    # print(f"Simulated Pin({self._pin_num}) set to {val}")
                return None # Match MicroPython's return type on set

        def on(self):
            self.value(1)

        def off(self):
            self.value(0)

    # Assign the dummy class to the name expected by the rest of the code
    Pin = DummyPin

    # CPython compatible sleep_us (takes microseconds, sleeps in seconds)
    def sleep_us(us):
        time.sleep(us / 1_000_000.0)

    # Dummy network module for CPython (only need constants)
    class DummyNetwork:
        STA_IF = 1
        AP_IF = 2
        def WLAN(self, mode):
             # Return a dummy WLAN object that does nothing for connect/active
            class DummyWLAN:
                def active(self, *args): pass
                def connect(self, *args): pass
                def isconnected(self): return True # Assume connected on CPython
                def ifconfig(self): return ('127.0.0.1', '255.255.255.0', '192.168.1.1', '8.8.8.8') # Dummy IP
            return DummyWLAN()

    network = DummyNetwork()
    
    # Use current directory for uploads on CPython
    GCODE_UPLOAD_PATH = os.path.join(os.getcwd(), "gcode_uploads") 
    if not os.path.exists(GCODE_UPLOAD_PATH):
        try:
            os.makedirs(GCODE_UPLOAD_PATH)
            print(f"Created directory for uploads: {GCODE_UPLOAD_PATH}")
        except OSError as e:
            print(f"Error creating upload directory {GCODE_UPLOAD_PATH}: {e}")
            GCODE_UPLOAD_PATH = os.getcwd() # Fallback to current dir


# --- Configuration ---
WIFI_SSID = "the realest"
WIFI_PASSWORD = "assflAt1-"

# Motor 1 Pins
MOTOR1_DIR_PIN = 19
MOTOR1_STEP_PIN = 18
MOTOR1_STEPS_PER_REV = 100 # Example value, seems low for steppers, maybe full steps?

# Motor 2 Pins
MOTOR2_DIR_PIN = 23
MOTOR2_STEP_PIN = 22
MOTOR2_STEPS_PER_REV = 100 # Example value

# LED Pin (Common on ESP boards)
LED_PIN = 2

# Robot Arm Configuration
ARM_STEPS_PER_REV = 200  # Actual motor steps (e.g., 1.8 deg/step)
ARM_MICROSTEPS = 16      # Microstepping setting on driver
ARM_GEAR_RATIO_BASE = 2.6
ARM_GEAR_RATIO_ARM1 = 2.6
ARM_GEAR_RATIO_ARM2 = 2.6
ARM_LINK_LENGTH = 75 # Assuming arm link length is 75 units (mm?)

# --- Stepper Motor Class ---
class StepperMotor:
    def __init__(self, dir_pin_num, step_pin_num, steps_per_rev):
        self.dir_pin = Pin(dir_pin_num, Pin.OUT)
        self.step_pin = Pin(step_pin_num, Pin.OUT)
        self.steps_per_rev = steps_per_rev
        self.current_step = 0 # Track position relative to start
        self.step_pin.value(0) # Ensure step pin is low initially
        self.dir_pin.value(0)  # Default direction

    def _pulse(self, delay_us):
        """Generates a single step pulse."""
        self.step_pin.value(1)
        sleep_us(delay_us // 2) # Use half delay for high
        self.step_pin.value(0)
        sleep_us(delay_us // 2) # Use half delay for low

    def step(self, num_steps, direction, delay_us=2000):
        """Steps the motor a specific number of times."""
        self.dir_pin.value(direction)
        # print(f"Stepping {'CW' if direction else 'CCW'} by {num_steps} steps...") # Can be verbose
        for _ in range(num_steps):
            self._pulse(delay_us)

        # Update internal position tracking
        if direction:
            self.current_step += num_steps
        else:
            self.current_step -= num_steps

    def move_to_step(self, target_step, delay_us=2000):
        """Moves the motor to an absolute step position."""
        steps_needed = target_step - self.current_step
        if steps_needed == 0:
            print(f"Already at target step {target_step}.")
            return

        direction = 1 if steps_needed > 0 else 0
        num_steps = abs(steps_needed)

        print(f"Moving from step {self.current_step} to {target_step} ({num_steps} steps {'CW' if direction else 'CCW'})")
        self.step(num_steps, direction, delay_us)
        # Note: self.current_step is updated within the step method now
        print(f"Reached step: {self.current_step}") # Should now match target_step

# --- Robot Arm Control Class ---
class RobotArm:
    def __init__(self, motor1, motor2): # Add base motor if needed
        self.motor1 = motor1 # Corresponds to arm1
        self.motor2 = motor2 # Corresponds to arm2
        # self.base_motor = base_motor # If you have a base motor

        # Robot Arm Configuration (moved from global scope)
        self.steps_per_rev = ARM_STEPS_PER_REV
        self.microsteps = ARM_MICROSTEPS
        self.gear_ratio_base = ARM_GEAR_RATIO_BASE
        self.gear_ratio_arm1 = ARM_GEAR_RATIO_ARM1
        self.gear_ratio_arm2 = ARM_GEAR_RATIO_ARM2
        self.link_length = ARM_LINK_LENGTH

        self.current_pos = [0.0, 0.0, 0.0] # Track current X, Y, Z tool position
        # It's often better to track current *angles* or *steps* for accuracy
        self.current_angles = {'b': 0.0, 'a1': 0.0, 'a2': 0.0} # Base, Arm1, Arm2 angles
        self.current_motor_steps = {'base': 0, 'arm1': self.motor1.current_step, 'arm2': self.motor2.current_step}


    def _angle_to_steps(self, angle_deg, gear_ratio):
        """Convert angle (degrees) to motor steps considering microstepping and gear ratio."""
        # Angle seen by the *motor* shaft
        motor_angle_deg = angle_deg * gear_ratio
        # Steps for that motor angle
        steps = (motor_angle_deg / 360.0) * self.steps_per_rev * self.microsteps
        return int(round(steps)) # Round to nearest whole step

    def _calculate_ik(self, x, y, z):
        """Calculate Inverse Kinematics for the target X, Y, Z. Returns angles (b, a1, a2) in degrees."""
        try:
            # Base angle (rotation around Z)
            b = math.atan2(y, x) * (180 / math.pi)

            # Calculate planar distance (l) and height (z)
            l = math.sqrt(x**2 + y**2)
            h = math.sqrt(l**2 + z**2) # Distance from base joint to end effector

            # Check reachability
            if h > self.link_length * 2:
                 raise ValueError(f"Position ({x},{y},{z}) is unreachable (too far). Max reach approx {self.link_length*2}")
            if h < 0: # Should not happen with sqrt, but good practice
                 raise ValueError("Position calculation resulted in negative distance.")


            # Angle of the line from base to end effector relative to the XY plane
            phi = math.atan2(z, l) * (180 / math.pi)

            # Angle calculation using law of cosines (for angles within the arm linkage)
            # Ensure the argument for acos is within [-1, 1] due to potential floating point inaccuracies
            cos_arg = h / (2 * self.link_length)
            cos_arg = max(-1.0, min(1.0, cos_arg)) # Clamp value
            theta = math.acos(cos_arg) * (180 / math.pi)

            # Calculate joint angles relative to horizontal/base frame
            # Adjust these based on your arm's zero position definition
            a1 = phi + theta  # Angle of the first link relative to the XY plane
            a2 = phi - theta  # Angle of the second link relative to the XY plane

            return b, a1, a2
        except ValueError as e:
            print(f"IK Error: {e}")
            return None # Indicate failure

    def move_to_position_ik(self, target_x, target_y, target_z, gripper_angle=0, step_delay_us=1000):
        """Moves the arm to a target X, Y, Z coordinate using Inverse Kinematics."""
        print(f"Attempting to move to position: X={target_x}, Y={target_y}, Z={target_z}")
        print(f"Current logical position: {self.current_pos}")

        angles = self._calculate_ik(target_x, target_y, target_z)
        if angles is None:
            print("IK calculation failed. Cannot move.")
            return

        b_deg, a1_deg, a2_deg = angles
        print(f"Calculated target angles (degrees): Base={b_deg:.2f}, Arm1={a1_deg:.2f}, Arm2={a2_deg:.2f}")

        # --- Convert angles to target motor steps ---
        # IMPORTANT: Define the zero angle reference for your physical setup.
        # The "+90", "-90", "+101" offsets in the original code suggest the zero angle
        # definitions might be different from standard conventions. Adjust these offsets
        # based on how your arm is built and how sensors (if any) define zero.
        # Let's assume for now:
        # Base: 0 degrees is along the X axis.
        # Arm1: 0 degrees is horizontal.
        # Arm2: 0 degrees is horizontal relative to Arm1's frame (or maybe horizontal in world frame? Needs clarification).

        # Assuming a1=0 is horizontal, a2=0 is horizontal:
        # target_base_steps = self._angle_to_steps(b_deg, self.gear_ratio_base) # If base motor exists
        target_arm1_steps = self._angle_to_steps(a1_deg, self.gear_ratio_arm1)
        target_arm2_steps = self._angle_to_steps(a2_deg, self.gear_ratio_arm2) # Check if a2 needs offset relative to a1

        # Using the original offsets for demonstration, assuming they are correct for the hardware:
        # target_base_steps = self._angle_to_steps(b_deg + 90, self.gear_ratio_base)
        target_arm1_steps = self._angle_to_steps(a1_deg - 90, self.gear_ratio_arm1) # Motor 1 controls Arm 1
        target_arm2_steps = self._angle_to_steps(a2_deg + 101, self.gear_ratio_arm2) # Motor 2 controls Arm 2

        print("Target motor steps calculated:")
        # print(f"  Base motor target: {target_base_steps} steps")
        print(f"  Arm1 motor target: {target_arm1_steps} steps (Motor 1)")
        print(f"  Arm2 motor target: {target_arm2_steps} steps (Motor 2)")
        # print(f"  Gripper target: {gripper_angle}°") # Gripper control not implemented here

        # --- Calculate steps needed from current motor positions ---
        steps_to_move1 = target_arm1_steps - self.motor1.current_step
        steps_to_move2 = target_arm2_steps - self.motor2.current_step

        # --- Simultaneous Movement (Linear Interpolation - Bresenham's Line Algo adapted) ---
        print(f"Moving Motor 1 by {steps_to_move1} steps, Motor 2 by {steps_to_move2} steps.")

        dir1 = 1 if steps_to_move1 > 0 else 0
        dir2 = 1 if steps_to_move2 > 0 else 0

        self.motor1.dir_pin.value(dir1)
        self.motor2.dir_pin.value(dir2)

        steps1 = abs(steps_to_move1)
        steps2 = abs(steps_to_move2)
        max_steps = max(steps1, steps2)

        if max_steps == 0:
            print("Motors already at target steps.")
            # Update logical position even if no steps moved physically
            self.current_pos = [target_x, target_y, target_z]
            # Update current angles and motor steps based on calculation
            self.current_angles = {'b': b_deg, 'a1': a1_deg, 'a2': a2_deg}
            self.current_motor_steps['arm1'] = self.motor1.current_step # Should match target
            self.current_motor_steps['arm2'] = self.motor2.current_step # Should match target
            return

        # Bresenham-like interpolation for smooth simultaneous movement
        delta1 = 2 * steps1
        delta2 = 2 * steps2
        error1 = delta1 - max_steps
        error2 = delta2 - max_steps

        for i in range(max_steps):
            step1_pulse = False
            step2_pulse = False

            if error1 >= 0:
                step1_pulse = True
                error1 -= 2 * max_steps

            if error2 >= 0:
                step2_pulse = True
                error2 -= 2 * max_steps

            if step1_pulse or step2_pulse:
                if step1_pulse: self.motor1.step_pin.value(1)
                if step2_pulse: self.motor2.step_pin.value(1)
                sleep_us(step_delay_us // 2) # Pulse high time

                if step1_pulse: self.motor1.step_pin.value(0)
                if step2_pulse: self.motor2.step_pin.value(0)
                # Update internal step counts immediately *after* the pulse
                if step1_pulse: self.motor1.current_step += (1 if dir1 else -1)
                if step2_pulse: self.motor2.current_step += (1 if dir2 else -1)

            error1 += delta1
            error2 += delta2

            # Low time for the step pulse (or delay between steps if no pulse occurred)
            sleep_us(step_delay_us // 2) # Ensure minimum low time


        print("Movement finished.")
        # Update logical position *after* movement completes
        self.current_pos = [target_x, target_y, target_z]
        # Update current angles and motor steps based on final physical state
        self.current_angles = {'b': b_deg, 'a1': a1_deg, 'a2': a2_deg} # Store calculated angles
        self.current_motor_steps['arm1'] = self.motor1.current_step # Should match target
        self.current_motor_steps['arm2'] = self.motor2.current_step # Should match target
        print(f"Reached Position: X={self.current_pos[0]}, Y={self.current_pos[1]}, Z={self.current_pos[2]}")
        print(f"Final Motor Steps: M1={self.motor1.current_step}, M2={self.motor2.current_step}")


# --- Web Server Class ---
class SimpleWebServer:
    def __init__(self, host='', port=9000, robot_arm=None, motor1=None, motor2=None):
        self.host = host
        self.port = port
        self.robot_arm = robot_arm # Pass the arm instance
        self.motor1 = motor1 # Pass motor instances if direct control is needed
        self.motor2 = motor2
        self.server_socket = None

        # Component States
        self.led_pin = Pin(LED_PIN, Pin.OUT) if IS_MICROPYTHON else DummyPin(LED_PIN, Pin.OUT) # Use dummy if needed
        self.led_state = 0
        self.selected_motor = 1 # Default to motor 1

    def _setup_wifi(self):
        if IS_MICROPYTHON:
            sta = network.WLAN(network.STA_IF)
            sta.active(True)
            print(f"Connecting to Wi-Fi SSID: {WIFI_SSID}...")
            sta.connect(WIFI_SSID, WIFI_PASSWORD)
            max_wait = 15
            while not sta.isconnected() and max_wait > 0:
                print(".")
                time.sleep(1)
                max_wait -= 1

            if sta.isconnected():
                print("Connected to Wi-Fi.")
                ip_info = sta.ifconfig()
                print("IP Address:", ip_info[0])
                self.host = ip_info[0] # Update host IP for listening
                return True
            else:
                print("Wi-Fi connection failed.")
                return False
        else:
            print("Skipping Wi-Fi setup for CPython.")
            # Try to get local IP for user info
            try:
                hostname = socket.gethostname()
                local_ip = socket.gethostbyname(hostname)
                self.host = local_ip # Update host IP
                print(f"Running on CPython. Assumed IP: {local_ip}")
                print(f"Access the server at http://{local_ip}:{self.port}")
            except socket.gaierror:
                 print("Could not determine local IP address. Using '0.0.0.0'.")
                 print(f"Try accessing via http://127.0.0.1:{self.port} or your machine's network IP.")
                 self.host = '0.0.0.0' # Listen on all interfaces
            return True # Assume network is available


    def _generate_html(self):
        # Get current motor positions from the instances
        pos1 = self.motor1.current_step if self.motor1 else "N/A"
        pos2 = self.motor2.current_step if self.motor2 else "N/A"
        arm_pos_str = f"X={self.robot_arm.current_pos[0]:.1f}, Y={self.robot_arm.current_pos[1]:.1f}, Z={self.robot_arm.current_pos[2]:.1f}" if self.robot_arm else "N/A"

        led_btn_state = "ON" if self.led_state else "OFF"
        motor1_selected = "checked" if self.selected_motor == 1 else ""
        motor2_selected = "checked" if self.selected_motor == 2 else ""

        # Basic styling
        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Control Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ font-family: Arial, sans-serif; padding: 15px; }}
        .container {{ max-width: 600px; margin: auto; background: #f4f4f4; padding: 20px; border-radius: 5px; }}
        h1, h2 {{ text-align: center; }}
        .button, input[type=submit] {{ padding: 10px 20px; font-size: 16px; margin: 5px; cursor: pointer; border: none; border-radius: 4px; }}
        .button-on {{ background-color: #4CAF50; color: white; }}
        .button-off {{ background-color: #f44336; color: white; }}
        .button-action {{ background-color: #008CBA; color: white; }}
        .section {{ margin-bottom: 20px; padding: 15px; background: #fff; border: 1px solid #ddd; border-radius: 4px;}}
        label {{ display: inline-block; margin-right: 10px; }}
        input[type=number], input[type=text] {{ padding: 8px; margin: 5px; width: 80px; }}
        input[type=file] {{ margin-top: 10px; }}
        .status {{ font-weight: bold; }}
        form {{ margin-top: 10px; }}
    </style>
</head>
<body>
<div class="container">
    <h1>ESP Control Panel</h1>

    <div class="section">
        <h2>Status</h2>
        <p>LED Status: <span class="status">{led_btn_state}</span></p>
        <p>Motor 1 Position: <span class="status">{pos1} steps</span></p>
        <p>Motor 2 Position: <span class="status">{pos2} steps</span></p>
        <p>Robot Arm Position: <span class="status">{arm_pos_str}</span></p>
         <p>Selected Motor for Jogging: <span class="status">Motor {self.selected_motor}</span></p>
    </div>

    <div class="section">
        <h2>Controls</h2>
        <form action="/toggle_led" method="get">
            <button class="button {'button-on' if not self.led_state else 'button-off'}" type="submit">
                Toggle LED ({'Turn ON' if not self.led_state else 'Turn OFF'})
            </button>
        </form>
        <hr>
        <form action="/select_motor" method="get">
            Select motor for manual jog:
            <input type="radio" id="motor1" name="motor" value="1" {motor1_selected}>
            <label for="motor1">Motor 1</label>
            <input type="radio" id="motor2" name="motor" value="2" {motor2_selected}>
            <label for="motor2">Motor 2</label>
            <input type="submit" value="Select">
        </form>
        <form action="/jog" method="get">
            <input type="hidden" name="direction" value="1">
            <button class="button button-action" type="submit">Jog Clockwise (Selected Motor)</button>
        </form>
        <form action="/jog" method="get">
            <input type="hidden" name="direction" value="0">
            <button class="button button-action" type="submit">Jog Anti-Clockwise (Selected Motor)</button>
        </form>
         <hr>
        <form action="/move_to_step" method="get">
             Move Selected Motor ({self.selected_motor}) to Step:
             <input type="number" name="position" required>
             <input type="submit" value="Move Step">
         </form>

    </div>

    <div class="section">
        <h2>Robot Arm IK Control</h2>
        <form action="/move_to_pos" method="get">
            Move to Position (X, Y, Z):<br>
            <label for="x_pos">X:</label><input type="number" step="any" name="x" id="x_pos" required>
            <label for="y_pos">Y:</label><input type="number" step="any" name="y" id="y_pos" required>
            <label for="z_pos">Z:</label><input type="number" step="any" name="z" id="z_pos" required>
            <input type="submit" value="Move Arm">
        </form>
    </div>

    <div class="section">
        <h2>File Upload (G-code)</h2>
         <form action="/upload" method="post" enctype="multipart/form-data">
             <input type="file" name="gcodefile" accept=".gcode,.nc">
             <input type="submit" value="Upload G-code">
         </form>
         <p><i>Note: G-code processing is not implemented in this example. File will be saved.</i></p>
         <p><i>Save location: {GCODE_UPLOAD_PATH}</i></p>
    </div>

</div>
</body>
</html>"""
        return html

    def _parse_query_params(self, request_path):
        """Helper to parse query parameters from a GET request path."""
        params = {}
        query_string = request_path.split('?')
        if len(query_string) > 1:
            pairs = query_string[1].split('&')
            for pair in pairs:
                parts = pair.split('=')
                if len(parts) == 2:
                    # Simple URL decoding for %20 etc. might be needed for robustness
                    params[parts[0]] = parts[1]
        return params
        
    def _handle_request(self, client_conn, addr):
        """Handles a single client connection."""
        try:
            # Increase buffer size for potential POST data
            request_data = b""
            # Read headers first (MicroPython socket might behave differently than CPython)
            # A simple approach: read until timeout or enough data, then parse.
            # Robust parsing requires checking Content-Length for POST.
            client_conn.settimeout(2.0) # Set timeout for reading request
            try:
                while True:
                    chunk = client_conn.recv(1024)
                    if not chunk:
                        break
                    request_data += chunk
                    # Simple check if headers might be complete (look for double CRLF)
                    # This is NOT robust for POST but okay for simple GETs
                    if b"\r\n\r\n" in request_data:
                         # If it seems like a GET or headers are done, break early
                         # A better check would be needed for large POSTs
                         if b"GET /" in request_data.split(b'\r\n')[0]:
                             break 
                         # Add check for Content-Length if POST needed more data
            except socket.timeout:
                 pass # Data read finished or timed out

            if not request_data:
                print(f"No data received from {addr}. Closing connection.")
                client_conn.close()
                return

            request_str = request_data.decode('utf-8', errors='ignore') # Decode safely
            # print(f"\n--- Request from {addr} ---")
            # print(request_str.split('\r\n\r\n')[0]) # Print only headers
            # print("--- End Request ---")
            
            # --- Basic Routing ---
            first_line = request_str.split('\r\n', 1)[0]
            parts = first_line.split(' ')
            if len(parts) < 2:
                # Malformed request
                client_conn.close()
                return

            method = parts[0]
            path = parts[1]

            response_content = ""
            content_type = "text/html"
            status_code = "200 OK"

            if method == "GET":
                if path == '/':
                    response_content = self._generate_html()
                elif path == '/toggle_led':
                    self.led_state = not self.led_state
                    self.led_pin.value(self.led_state)
                    print(f"Toggled LED to {'ON' if self.led_state else 'OFF'}")
                    # Redirect back to main page
                    status_code = "302 Found"
                    headers = f"HTTP/1.1 {status_code}\r\nLocation: /\r\nConnection: close\r\n\r\n"
                    client_conn.sendall(headers.encode())
                    client_conn.close()
                    return # Skip standard response sending

                elif path.startswith('/select_motor'):
                     params = self._parse_query_params(path)
                     motor_val = params.get('motor')
                     try:
                         motor_num = int(motor_val)
                         if motor_num in [1, 2]:
                             self.selected_motor = motor_num
                             print(f"Selected motor {self.selected_motor}")
                         else:
                            print(f"Invalid motor selection: {motor_val}")
                     except (ValueError, TypeError):
                         print(f"Invalid motor value: {motor_val}")
                     # Redirect back
                     status_code = "302 Found"
                     headers = f"HTTP/1.1 {status_code}\r\nLocation: /\r\nConnection: close\r\n\r\n"
                     client_conn.sendall(headers.encode())
                     client_conn.close()
                     return

                elif path.startswith('/jog'):
                    params = self._parse_query_params(path)
                    direction = int(params.get('direction', '1')) # Default to CW
                    motor_to_jog = self.motor1 if self.selected_motor == 1 else self.motor2
                    if motor_to_jog:
                        print(f"Jogging motor {self.selected_motor} {'CW' if direction else 'CCW'}")
                        # Jog a fixed number of steps, e.g., 10
                        motor_to_jog.step(num_steps=10, direction=direction, delay_us=1500)
                    else:
                        print(f"Cannot jog motor {self.selected_motor}, not initialized.")
                    # Redirect back
                    status_code = "302 Found"
                    headers = f"HTTP/1.1 {status_code}\r\nLocation: /\r\nConnection: close\r\n\r\n"
                    client_conn.sendall(headers.encode())
                    client_conn.close()
                    return

                elif path.startswith('/move_to_step'):
                    params = self._parse_query_params(path)
                    position = params.get('position')
                    try:
                        target_step = int(position)
                        motor_to_move = self.motor1 if self.selected_motor == 1 else self.motor2
                        if motor_to_move:
                            print(f"Moving motor {self.selected_motor} to step {target_step}")
                            motor_to_move.move_to_step(target_step, delay_us=1500)
                        else:
                             print(f"Cannot move motor {self.selected_motor}, not initialized.")
                    except (ValueError, TypeError):
                        print(f"Invalid target step: {position}")
                    # Redirect back
                    status_code = "302 Found"
                    headers = f"HTTP/1.1 {status_code}\r\nLocation: /\r\nConnection: close\r\n\r\n"
                    client_conn.sendall(headers.encode())
                    client_conn.close()
                    return

                elif path.startswith('/move_to_pos'):
                     params = self._parse_query_params(path)
                     try:
                         x = float(params.get('x', '0'))
                         y = float(params.get('y', '0'))
                         z = float(params.get('z', '0'))
                         if self.robot_arm:
                             print(f"Received move command: X={x}, Y={y}, Z={z}")
                             self.robot_arm.move_to_position_ik(x, y, z)
                         else:
                             print("Robot arm not initialized, cannot move.")
                     except (ValueError, TypeError) as e:
                        print(f"Invalid coordinate in request: {params} - Error: {e}")
                     # Redirect back
                     status_code = "302 Found"
                     headers = f"HTTP/1.1 {status_code}\r\nLocation: /\r\nConnection: close\r\n\r\n"
                     client_conn.sendall(headers.encode())
                     client_conn.close()
                     return
                     
                elif path == '/current_pos': # Simple API endpoint example
                    content_type = "text/plain"
                    if self.robot_arm:
                        response_content = f"X={self.robot_arm.current_pos[0]:.2f}, Y={self.robot_arm.current_pos[1]:.2f}, Z={self.robot_arm.current_pos[2]:.2f}"
                    else:
                        response_content = "Robot arm position not available."
                
                else:
                    status_code = "404 Not Found"
                    response_content = "<h1>404 Not Found</h1>"

            elif method == "POST":
                 if path == '/upload':
                     print("Attempting to handle file upload...")
                     # --- VERY Basic and Fragile POST multipart/form-data parsing ---
                     # --- This needs a robust library in a real application! ---
                     try:
                         # Find boundary (assumes it's correctly formatted)
                         header_part, body_part = request_str.split('\r\n\r\n', 1)
                         content_type_header = next((h for h in header_part.split('\r\n') if h.lower().startswith('content-type:')), None)
                         
                         if not content_type_header or 'boundary=' not in content_type_header:
                             raise ValueError("Missing or invalid boundary in Content-Type header.")
                             
                         boundary = "--" + content_type_header.split('boundary=')[1]
                         
                         # Find the part containing the file
                         parts = body_part.split(boundary)
                         file_part_content = None
                         filename = "uploaded_file.gcode" # Default filename

                         for part in parts:
                             if 'filename="' in part:
                                 # Extract filename
                                 try:
                                     filename = part.split('filename="')[1].split('"')[0]
                                     if not filename: filename = "uploaded_file.gcode" # Use default if empty
                                 except IndexError:
                                     pass # Keep default filename
                                     
                                 # Find the actual content after the part headers
                                 try:
                                     file_content_start = part.index('\r\n\r\n') + 4
                                     file_part_content = part[file_content_start:]
                                     break # Found the file part
                                 except ValueError:
                                      print("Warning: Found filename but couldn't find content delimiter in part.")
                                      continue

                         if file_part_content:
                             # Ensure the upload directory exists
                             if IS_MICROPYTHON and GCODE_UPLOAD_PATH == "/gcode":
                                 # On MicroPython, we might need to ensure /gcode exists if using it
                                 try:
                                     os.stat(GCODE_UPLOAD_PATH) # Check if exists
                                 except OSError:
                                     try: 
                                         os.mkdir(GCODE_UPLOAD_PATH) # Try to create it
                                         print(f"Created directory {GCODE_UPLOAD_PATH}")
                                     except OSError as e:
                                         print(f"Error creating MicroPython directory {GCODE_UPLOAD_PATH}: {e}. Saving to root.")
                                         GCODE_UPLOAD_PATH = "/" # Fallback to root

                             elif not IS_MICROPYTHON and not os.path.exists(GCODE_UPLOAD_PATH):
                                 try:
                                     os.makedirs(GCODE_UPLOAD_PATH)
                                     print(f"Created directory {GCODE_UPLOAD_PATH}")
                                 except OSError as e:
                                     print(f"Error creating CPython directory {GCODE_UPLOAD_PATH}: {e}. Saving to current dir.")
                                     GCODE_UPLOAD_PATH = os.getcwd()


                             # Sanitize filename slightly (very basic)
                             safe_filename = "".join(c for c in filename if c.isalnum() or c in ('.', '_', '-')).strip()
                             if not safe_filename: safe_filename = "uploaded_file.gcode" # Prevent empty names
                             
                             save_path = os.path.join(GCODE_UPLOAD_PATH, safe_filename) if not IS_MICROPYTHON else GCODE_UPLOAD_PATH + "/" + safe_filename

                             print(f"Saving uploaded file '{filename}' as '{save_path}'")
                             # Write the content (decoded earlier) - needs to handle binary write
                             # We need the raw bytes from request_data for this.
                             # Re-finding the content start in the raw data is more robust.
                             raw_header_part, raw_body_part = request_data.split(b'\r\n\r\n', 1)
                             raw_boundary = boundary.encode('utf-8')
                             raw_parts = raw_body_part.split(raw_boundary)
                             raw_file_content = None
                             for r_part in raw_parts:
                                 if b'filename="' in r_part:
                                     try:
                                         content_start_index = r_part.index(b'\r\n\r\n') + 4
                                         # Need to strip trailing CRLF if present before boundary
                                         raw_file_content = r_part[content_start_index:].rstrip(b'\r\n')
                                         break
                                     except ValueError:
                                         continue
                                         
                             if raw_file_content:        
                                 try:
                                     with open(save_path, "wb") as f:
                                         f.write(raw_file_content)
                                     print("File upload successful.")
                                     response_content = f"<h1>Upload Successful</h1><p>Saved as {safe_filename}</p><p><a href='/'>Back</a></p>"
                                 except Exception as e:
                                     print(f"Error writing file: {e}")
                                     status_code = "500 Internal Server Error"
                                     response_content = f"<h1>Upload Failed</h1><p>Error writing file: {e}</p><p><a href='/'>Back</a></p>"
                             else:
                                 print("Could not extract raw file content from POST.")
                                 status_code = "400 Bad Request"
                                 response_content = "<h1>Upload Failed</h1><p>Could not parse file content.</p><p><a href='/'>Back</a></p>"
                         else:
                            print("File content not found in POST request.")
                            status_code = "400 Bad Request"
                            response_content = "<h1>Upload Failed</h1><p>No file data found in request.</p><p><a href='/'>Back</a></p>"

                     except Exception as e:
                         print(f"Error processing POST request: {e}")
                         status_code = "500 Internal Server Error"
                         response_content = f"<h1>Server Error</h1><p>Error processing upload: {e}</p>"
                 else:
                      status_code = "405 Method Not Allowed"
                      response_content = "<h1>405 Method Not Allowed</h1>"

            # --- Send Response ---
            headers = f"HTTP/1.1 {status_code}\r\nContent-Type: {content_type}\r\nContent-Length: {len(response_content)}\r\nConnection: close\r\n\r\n"
            client_conn.sendall(headers.encode())
            if response_content: # Only send body if there is one
                client_conn.sendall(response_content.encode())

        except OSError as e:
            print(f"Socket Error handling request from {addr}: {e}")
        except Exception as e:
             print(f"General Error handling request from {addr}: {e}")
             # Try to send an error response if possible
             try:
                 error_response = b"HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<h1>Internal Server Error</h1>"
                 client_conn.sendall(error_response)
             except Exception as send_e:
                 print(f"Could not send error response to client: {send_e}")
        finally:
            # Ensure connection is closed
             try:
                client_conn.close()
             except Exception:
                 pass # Ignore errors during close


    def start(self):
        """Starts the web server."""
        if not self._setup_wifi():
             # On MicroPython, stop if Wi-Fi fails (unless intended to run without network)
             if IS_MICROPYTHON:
                 print("Halting due to Wi-Fi connection failure.")
                 return
             # On CPython, it might still be accessible locally

        try:
            # Use self.host which might have been updated by _setup_wifi
            addr = socket.getaddrinfo(self.host, self.port)[0][-1]
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Allow address reuse
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(addr)
            self.server_socket.listen(5) # Listen for up to 5 connections
            print(f"Web server started. Listening on {addr[0]}:{addr[1]}")

            while True:
                try:
                    conn, addr = self.server_socket.accept()
                    # print(f"\nConnection accepted from: {addr[0]}:{addr[1]}")
                    # Handle client request in the main loop (simple blocking server)
                    # For MicroPython, threading might be limited or unavailable.
                    # For CPython, you could use threading/asyncio for concurrent handling.
                    self._handle_request(conn, addr)

                except OSError as e:
                     print(f"Socket Accept/Handling Error: {e}")
                     # Add a small delay to prevent spamming errors in a tight loop if accept fails repeatedly
                     time.sleep(0.5) 
                except KeyboardInterrupt:
                     print("Server stopped by user.")
                     break
                except Exception as e:
                     print(f"Unexpected error in server loop: {e}")
                     time.sleep(1) # Prevent rapid looping on unexpected errors


        except Exception as e:
            print(f"Failed to start server: {e}")
        finally:
            if self.server_socket:
                self.server_socket.close()
                print("Server socket closed.")


# --- Main Execution ---
if __name__ == "__main__":
    print(f"Script starting in {'MicroPython' if IS_MICROPYTHON else 'CPython'} mode.")

    # Initialize Motors
    motor_1 = StepperMotor(MOTOR1_DIR_PIN, MOTOR1_STEP_PIN, MOTOR1_STEPS_PER_REV)
    motor_2 = StepperMotor(MOTOR2_DIR_PIN, MOTOR2_STEP_PIN, MOTOR2_STEPS_PER_REV)
    # Initialize Base Motor if you have one
    # base_motor = StepperMotor(BASE_DIR_PIN, BASE_STEP_PIN, BASE_STEPS_PER_REV)

    # Initialize Robot Arm
    # Pass the motor instances to the arm
    robot_arm = RobotArm(motor1=motor_1, motor2=motor_2) # Add base_motor if used

    # Initialize and Start Web Server
    # Pass the arm and/or motors to the server if it needs to control them
    web_server = SimpleWebServer(robot_arm=robot_arm, motor1=motor_1, motor2=motor_2)
    
    try:
      web_server.start()
    except KeyboardInterrupt:
      print("\nExiting program.")
    finally:
        # Optional: Cleanup GPIO if needed (especially on Raspberry Pi CPython)
        if not IS_MICROPYTHON:
             print("CPython environment: No hardware cleanup needed for DummyPin.")
        else:
             # MicroPython might not require explicit cleanup, depends on board
             pass