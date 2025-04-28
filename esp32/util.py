import os

# Get filesystem stats: (total_blocks, block_size, used_blocks, ...)
stats = os.statvfs('/')

total_bytes = stats[0] * stats[1]
free_bytes = stats[3] * stats[1]

print("Total flash: {} bytes".format(total_bytes))
print("Free flash: {} bytes".format(free_bytes))


import os



def createdir(name):
    if name not in os.listdir():
        os.mkdir(name)

createdir("microdot")


with open("/gcode/benchy.gcode") as f:
    for line in f:
        print(line)


import network
import time 
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
mpremote connect /dev/ttyUSB0 fs rmdir esp32