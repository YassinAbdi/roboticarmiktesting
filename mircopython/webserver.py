import network
import socket
import machine
from machine import Pin
import time

# Wi-Fi credentials
SSID = "the realest"
PASSWORD = "assflAt1-"

# Setup Wi-Fi connection
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect(SSID, PASSWORD)

# Define constants
DIR = 23
STEP = 22
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
current_position = 0  # Start at step 0

def move_to_position(target_position, step_delay_us=2000):
    """Move the stepper motor to a specific position based on user input."""
    global current_position

    # Calculate steps required
    steps_needed = target_position - current_position
    direction = 1 if steps_needed > 0 else 0  # 1 for CW, 0 for CCW
    dir_pin.value(direction)

    print(f"Moving to position {target_position} ({'CW' if direction else 'CCW'})")

    for _ in range(abs(steps_needed)):
        step_pin.value(1)
        time.sleep_us(step_delay_us)
        step_pin.value(0)
        time.sleep_us(step_delay_us)

    # Update current position
    current_position = target_position
    print(f"Reached position: {current_position}")


while not sta.isconnected():
    pass

print("Connected to Wi-Fi:", sta.ifconfig())

# Define GPIO pin (LED or other output device)
LED_PIN = 2  # Change this based on your board (GPIO2 = Onboard LED for ESP8266)
led = machine.Pin(LED_PIN, machine.Pin.OUT)
led_state = 0  # Track LED state
clockwise_state = 0
counterclockwise_state = 0
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
    <a href="/clockwise"><button class="button {'on' if clockwise_state else 'off'}">Toggle clockwise</button></a>
    <p>turn counter clock wise is <strong>{btn3_state}</strong></p>
    <a href="/antiwise"><button class="button {'on' if counterclockwise_state else 'off'}">Toggle counter-clockwise</button></a>
    <p>Move to specific position (0-100):</p>
    <form action="/move_to" method="get">
        <input type="number" name="position" min="0" max="100" required>
        <input type="submit" value="Move">
    </form>
    <p>Current position: {current_position}</p>
    <p>Upload G-code file:</p>
    <form method="POST" enctype="multipart/form-data" action="/upload">
        <input type="file" name="file">
        <input type="submit" value="Upload">
    </form>
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
    
    if "GET /toggle" in request:
        print("Toggling LED")
        led_state = not led_state  # Toggle state
        led.value(led_state)  # Update GPIO

    if "GET /clockwise" in request:
        print("Toggling Stepper")
        clockwise_state = not clockwise_state  # Toggle state
        step_motor(2500, 1)  # Clockwise rotation
    
    if "GET /antiwise" in request:
        print("Toggling Stepper Anti")
        counterclockwise_state = not counterclockwise_state  # Toggle state
        step_motor(2500, 0)  # Anti-clockwise rotation

    # add a rest call for moving to a specific position
    if "GET /move_to" in request:
        print("Moving to specific position")
        # Extract position from request
        position = int(request.split('=')[1].split(' ')[0])
        print(f"Moving to position {position}")
        move_to_position(position)

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
