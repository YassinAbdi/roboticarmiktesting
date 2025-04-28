from microdot import Microdot
import time
import math
from ArmConfig import *
import arm as arm


if MIRCOPYTHON:
    from microdot import Microdot
    import network
    import socket
    import machine
    from machine import Pin


# connect to the network
if MIRCOPYTHON:
    # Setup Wi-Fi connection
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    sta.connect(SSID, PASSWORD)
    while not sta.isconnected():
        print("Connecting to Wi-Fi...")
        time.sleep(1)
    print("Connected to Wi-Fi")
    print("IP Address:", sta.ifconfig()[0])

app = Microdot()
arm_instance = arm.Arm()
print(arm_instance.move_to_pos(0, 0, 0,0))

html = '''<!DOCTYPE html>
<html>
    <head>
        <title>Microdot Example Page</title>
        <meta charset="UTF-8">
    </head>
    <body>
        <div>
            <h1>Microdot Example Page</h1>
            <p>Hello from Microdot!</p>
            <p><a href="/shutdown">Click to shutdown the server</a></p>

        </div>
    </body>
</html>
'''


@app.route('/')
async def hello(request):
    return html, 200, {'Content-Type': 'text/html'}


@app.route('/shutdown')
async def shutdown(request):
    print("shutdown")
    request.app.shutdown()
    return 'The server is shutting down...'


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