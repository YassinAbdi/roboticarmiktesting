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

# Define constants
DIR = 19
STEP = 18
STEPS_PER_REV = 100

# Initialize pins
dir_pin = Pin(DIR, Pin.OUT)
step_pin = Pin(STEP, Pin.OUT)

def step_motor(delay_us, direction):
    dir_pin.value(direction)  # Set direction
    print("Spinning Clockwise..." if direction else "Spinning Anti-Clockwise...")

    for _ in range(STEPS_PER_REV):
        step_pin.value(1)
        time.sleep_us(delay_us)
        step_pin.value(0)
        time.sleep_us(delay_us)



# Track current position
current_position_1 = 0  # Start at step 0

def move_to_position1(target_position, step_delay_us=2000):
    """Move the stepper motor to a specific position based on user input."""
    global current_position_1

    # Calculate steps required
    steps_needed = target_position - current_position_1
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    dir_pin.value(direction)

    print(f"Moving to position {target_position} ({'CW' if direction else 'CCW'})")

    for _ in range(abs(steps_needed)):
        step_pin.value(1)
        time.sleep_us(step_delay_us)
        step_pin.value(0)
        time.sleep_us(step_delay_us)

    # Update current position
    current_position_1 = target_position
    print(f"Reached position: {current_position_1}")


current_position_2 = 0  # Start at step 0

# Define constants
DIR2 = 23
STEP2 = 22
STEPS_PER_REV2 = 100

# Initialize pins
dir_pin2 = Pin(DIR2, Pin.OUT)
step_pin2 = Pin(STEP2, Pin.OUT)

def step_motor2(delay_us, direction):
    dir_pin2.value(direction)  # Set direction
    print("Spinning Clockwise..." if direction else "Spinning Anti-Clockwise...")

    for _ in range(STEPS_PER_REV2):
        step_pin2.value(1)
        time.sleep_us(delay_us)
        step_pin2.value(0)
        time.sleep_us(delay_us)

def move_to_position2(target_position, step_delay_us=2000):
    """Move the stepper motor to a specific position based on user input."""
    global current_position_2

    # Calculate steps required
    steps_needed = target_position - current_position_2
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    dir_pin2.value(direction)

    print(f"Moving to position {target_position} ({'CW' if direction else 'CCW'})")

    for _ in range(abs(steps_needed)):
        step_pin2.value(1)
        time.sleep_us(step_delay_us)
        step_pin2.value(0)
        time.sleep_us(step_delay_us)

    # Update current position
    current_position_2 = target_position
    print(f"Reached position: {current_position_2}")

# Motor and gear setup
steps_per_rev = 200  # 1.8° stepper motor
microsteps = 16       # Microstepping factor
gear_ratio_base = 2.6
gear_ratio_arm1 = 2.6
gear_ratio_arm2 = 2.6

# Track the current position of the arm
current_pos = [0, 0, 0]  # x, y, z

def angle_to_steps(angle, gear_ratio):
    """
    Convert angle (degrees) to stepper motor steps based on gear ratio and microstepping.
    """
    total_steps_per_deg = ((steps_per_rev * microsteps)/360)
    steps = angle * total_steps_per_deg
    return int(steps)

def move_to_angle(b, a1, a2, g):
    """
    Convert angles to stepper motor steps based on gear ratio and microstepping.
    """
    base_steps = angle_to_steps(b + 90, gear_ratio_base)        # Base: 0° means pointing forward

    # Arm1: 90° is home (step=0), subtract 90
    arm1_steps = angle_to_steps(a1 - 90, gear_ratio_arm1)

    # Arm2: adjust to your real zero (let's assume 0° = -101 offset for now)
    arm2_steps = angle_to_steps(a2 + 101, gear_ratio_arm2)  # You can change this too

    print("Stepper motor commands:")
    print(f"  Base motor: {base_steps} steps")
    print(f"  Arm1 motor: {arm1_steps} steps")
    print(f"  Arm2 motor: {arm2_steps} steps")
    print(f"  Gripper: {g}° (manual or servo)")
    # Here you would send the steps to the motor driver
    move_to_position1(arm1_steps*-1)
    move_to_position2(arm2_steps)
    
    

def move_to_pos(x, y, z, g):
    """
    Compute joint angles to reach target position (x, y, z) and convert to motor steps.
    Uses delta from current_pos and updates it afterward.
    """
    dx = x - current_pos[0]
    dy = y - current_pos[1]
    dz = z - current_pos[2]

    print(f"Moving delta: dx={dx}, dy={dy}, dz={dz}")

    # Use target pos to compute angles
    b = math.atan2(y, x) * (180 / math.pi)
    l = math.sqrt(x**2 + y**2)
    h = math.sqrt(l**2 + z**2)
    phi = math.atan2(z, l) * (180 / math.pi)
    theta = math.acos(min(max(h / 2 / 75, -1), 1)) * (180 / math.pi)

    a1 = phi + theta
    a2 = phi - theta

    print(f"Calculated angles (degrees):")
    print(f"  Base: {b:.2f}, Arm1: {a1:.2f}, Arm2: {a2:.2f}")

    move_to_angle(b, a1, a2, g)

    # Update current position after moving
    current_pos[0] = x
    current_pos[1] = y
    current_pos[2] = z

while not sta.isconnected():
    pass

print("Connected to Wi-Fi:", sta.ifconfig())

# Define GPIO pin (LED or other output device)
LED_PIN = 2  # Change this based on your board (GPIO2 = Onboard LED for ESP8266)
led = machine.Pin(LED_PIN, machine.Pin.OUT)
led_state = 0  # Track LED state
clockwise_state = 0
counterclockwise_state = 0
selected_motor = 1
x_pos = 0
y_pos = 0
z_pos = 0
# Function to return HTML page with button
def web_page():
    btn_state = "ON" if led_state else "OFF"
    btn2_state = "ON" if clockwise_state else "OFF"
    btn3_state = "ON" if counterclockwise_state else "OFF"
    html = f"""<!DOCTYPE html>
    <html>
    <head>
    <title>MicroPython Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ text-align: center; font-family: Arial, sans-serif; }}
        .button {{ padding: 15px 30px; font-size: 20px; margin: 10px; cursor: pointer; }}
        .on {{ background-color: green; color: white; }}
        .off {{ background-color: red; color: white; }}
    </style>
    </head>
    <body>
    <h1>ESP Web Server</h1>
    <p>LED is <strong>{btn_state}</strong></p>
    <a href="/toggle"><button class="button {'on' if led_state else 'off'}">Toggle LED</button></a>
    <p>turn clock wise is <strong>{btn2_state}</strong></p>
    <p>Select motor:</p>
    <a href="/motor1"><button class="button {'on' if selected_motor == 1 else 'off'}">Motor 1</button></a>
    <a href="/motor2"><button class="button {'on' if selected_motor == 2 else 'off'}">Motor 2</button></a>
    <a href="/clockwise"><button class="button {'on' if clockwise_state else 'off'}">Toggle clockwise</button></a>
    <p>turn counter clock wise is <strong>{btn3_state}</strong></p>
    <a href="/antiwise"><button class="button {'on' if counterclockwise_state else 'off'}">Toggle counter-clockwise</button></a>
    <p>Move to specific position (0-100):</p>
    <form action="/move_to" method="get">
        <input type="number" name="position" min="0" max="3200" required>
        <input type="submit" value="Move">
    </form>
    <p>Current position: {current_position_1}</p>
    <p>Upload G-code file:</p>
    <form method="POST" enctype="multipart/form-data" action="/upload">
        <input type="file" name="file">
        <input type="submit" value="Upload">
    </form>
    
    <p>Move to position (x, y, z):</p>
    <form action="/smove_to_pos" method="get">
        <input type="number" name="x" placeholder="X" required>
        <input type="number" name="y" placeholder="Y" required>
        <input type="number" name="z" placeholder="Z" required>
        <input type="submit" value="Move">
    </form>

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
    
    if "GET /toggle" in request:
        print("Toggling LED")
        led_state = not led_state  # Toggle state
        led.value(led_state)  # Update GPIO

    if "GET /clockwise" in request:
        print("Toggling Stepper")
        clockwise_state = not clockwise_state  # Toggle state
        if selected_motor == 1:
            step_motor(2500, 1)
        if selected_motor == 2:
            step_motor2(2500, 1)  # Clockwise rotation
        
    if "GET /motor1" in request:
        print("Selected Motor 1")
        selected_motor = 1
    if "GET /motor2" in request:    
        print("Selected Motor 2")
        selected_motor = 2
    if "GET /antiwise" in request:
        print("Toggling Stepper Anti")
        counterclockwise_state = not counterclockwise_state  # Toggle state
        if selected_motor == 1:
            step_motor(2500, 0)
        if selected_motor == 2: 
            step_motor2(2500, 0)  # Anti-clockwise rotation

    # add a rest call for moving to a specific position
    if "GET /move_to" in request:
        print("Moving to specific position")
        # Extract position from request
        position = int(request.split('=')[1].split(' ')[0])
        
        print(f"Moving to position {position}")
        if(selected_motor == 1):
            move_to_position1(position)
        if(selected_motor == 2):
            move_to_position2(position)
    
    # get x, y, z from the request
    if "GET /smove_to_pos" in request:
        print("Moving to specific x, y, z position")
        # Extract x, y, z from request
        params = request.split(' ')[1].split('?')[1].split('&')
        x = int(params[0].split('=')[1])
        y = int(params[1].split('=')[1])
        z = int(params[2].split('=')[1])
        print(f"Moving to position x={x}, y={y}, z={z}")
        move_to_pos(x, y, z, 0)  # Assuming g is not used here
        # #reset current position
        # current_position_1 = 0
        # current_position_2 = 0
        

    # return current pos 
    if "GET /current_pos" in request:
        print("Returning current position")
        response = f"Current position: {current_pos}"
        conn.send("HTTP/1.1 200 OK\n")
        conn.send("Content-Type: text/plain\n")
        conn.send("Connection: close\n\n")
        conn.sendall(response.encode())
        conn.close()
        continue


    if "POST /upload" in request:
        print("Receiving file...")

        # Read until we hit the end of headers
        while b"\r\n\r\n" not in request.encode():
            request += conn.recv(1024).decode()

        # Now read the remaining body
        boundary = request.split("Content-Type: multipart/form-data; boundary=")[1].split("\r\n")[0]
        boundary = boundary.strip()

        # Find start of file data
        file_data_start = request.find("\r\n\r\n") + 4
        file_data = request[file_data_start:].encode()

        # Open a file to write
        with open("/gcode/benchy.gcode", "wb") as f:
            f.write(file_data)
            while True:
                chunk = conn.recv(1024)
                if not chunk or boundary.encode() in chunk:
                    break
                f.write(chunk)

        print("Upload complete.")

        # Respond to client
        conn.send("HTTP/1.1 200 OK\n")
        conn.send("Content-Type: text/html\n")
        conn.send("Connection: close\n\n")
        conn.sendall(b"<html><body><h1>Upload complete!</h1></body></html>")
        conn.close()
        continue

    response = web_page()
    conn.send("HTTP/1.1 200 OK\n")
    conn.send("Content-Type: text/html\n")
    conn.send("Connection: close\n\n")
    conn.sendall(response)
    conn.close()
