from microdot import Microdot
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
while not sta.isconnected():
    print("Connecting to Wi-Fi...")
    time.sleep(1)
print("Connected to Wi-Fi")
print("IP Address:", sta.ifconfig()[0])
# app = Microdot()

# html = '''<!DOCTYPE html>
# <html>
#     <head>
#         <title>Microdot Example Page</title>
#         <meta charset="UTF-8">
#     </head>
#     <body>
#         <div>
#             <h1>Microdot Example Page</h1>
#             <p>Hello from Microdot!</p>
#             <p><a href="/shutdown">Click to shutdown the server</a></p>
#         </div>
#     </body>
# </html>
# '''


# @app.route('/')
# async def hello(request):
#     return html, 200, {'Content-Type': 'text/html'}


# @app.route('/shutdown')
# async def shutdown(request):
#     request.app.shutdown()
#     return 'The server is shutting down...'


# app.run(debug=True)