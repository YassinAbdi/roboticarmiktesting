import requests

# ESP32 IP with port
ip_address = "10.0.0.116:5000"
url = f'http://{ip_address}/move'

# Coordinates and grip value
params = {
    'x': 10,   # Replace with desired X coordinate
    'y': 20,   # Replace with desired Y coordinate
    'z': 30,   # Replace with desired Z coordinate
    
}

try:
    response = requests.get(url, params=params)
    if response.status_code == 200:
        print("Success:", response.json())
    else:
        print("Failed:", response.status_code, response.text)
except Exception as e:
    print("Error:", e)