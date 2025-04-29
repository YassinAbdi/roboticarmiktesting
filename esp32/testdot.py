from microdot import Microdot
import time
from ArmConfig import *
import arm as arm

if MIRCOPYTHON:
    from machine import Pin
     


app = Microdot()
arm_instance = arm.Arm()


# Load HTML from a file
def load_html(filename):
    try:
        with open(filename, 'r') as f:
            return f.read()
    except Exception as e:
        print("Failed to load HTML:", str(e))
        return "<h1>Failed to load page</h1>"

html = load_html('index.html')

# Global states
led_state = False
clockwise_state = False
counterclockwise_state = False
selected_motor = 1
current_position_1 = 0
current_position_2 = 0

# Initialize GPIO pins
LED_PIN = 2  # GPIO pin for the LED

# GPIO setup
if MIRCOPYTHON:
    led = Pin(LED_PIN, Pin.OUT)
    led.value(led_state)  # Initialize LED state

@app.route('/')
async def index(request):
    print("Serving index.html")
    return html, 200, {'Content-Type': 'text/html'}

@app.route('/state')
async def state(request):
    print("Getting state")
    return {
        'led_state': led_state,
        'clockwise_state': clockwise_state,
        'counterclockwise_state': counterclockwise_state,
        'selected_motor': selected_motor,
        'current_position_1': current_position_1,
        'current_position_2': current_position_2
    }

@app.route('/toggle')
async def toggle_led(request):
    global led_state
    led_state = not led_state
    led.value(led_state)  # Update GPIO
    # Also actually toggle the LED GPIO if needed here
    return {'status': 'ok', 'led_state': led_state}

@app.route('/clockwise')
async def toggle_clockwise(request):
    global clockwise_state
    clockwise_state = not clockwise_state

    # Actually move the selected motor a few steps clockwise
    direction = 1  # Clockwise
    print(f"Rotating motor {selected_motor} clockwise")
    arm_instance.step_motor(delay_us=2000, direction=direction, motor=selected_motor)

    return {'status': 'ok', 'clockwise_state': clockwise_state}

@app.route('/antiwise')
async def toggle_counterclockwise(request):
    global counterclockwise_state
    counterclockwise_state = not counterclockwise_state

    # Actually move the selected motor a few steps counterclockwise
    direction = 0  # Counterclockwise
    print(f"Rotating motor {selected_motor} counterclockwise")
    arm_instance.step_motor(delay_us=2000, direction=direction, motor=selected_motor)

    return {'status': 'ok', 'counterclockwise_state': counterclockwise_state}

@app.route('/motor1')
async def motor1(request):
    global selected_motor
    selected_motor = 1
    print("Selected motor 1")
    return {'status': 'ok'}

@app.route('/motor2')
async def motor2(request):
    global selected_motor
    selected_motor = 2
    print("Selected motor 2")
    return {'status': 'ok'}



@app.route('/get_positionM1')
async def get_positionM1(request):
    global current_position_1
    # Get the current position of motor 1
    current_position_1 = arm_instance.M1_current_position
    print(f"Motor 1 current position: {current_position_1}")
    return {'current_position': current_position_1}
@app.route('/get_positionM2')
async def get_positionM2(request):
    global current_position_2
    # Get the current position of motor 2
    current_position_2 = arm_instance.M2_current_position
    print(f"Motor 2 current position: {current_position_2}")
    return {'current_position': current_position_2}

@app.route('/move')
async def move(request):
    try:
        # Get query parameters from the URL
        x = request.args.get('x', type=float)
        y = request.args.get('y', type=float)
        z = request.args.get('z', type=float)
        grip = request.args.get('grip', default=0, type=int)
        print(f"Received move command with x: {x}, y: {y}, z: {z}, grip: {grip}")
        if None in (x, y, z):
            return {'error': 'x, y, and z are required as query parameters'}, 400

        print(f"Moving to x: {x}, y: {y}, z: {z}, grip: {grip}")
        result = arm_instance.move_to_pos(x, y, z, grip)
        return {'status': 'success', 'result': result}

    except Exception as e:
        print("Move error:", str(e))
        return {'error': str(e)}, 500

app.run(debug=True)