from flask import Flask, jsonify
from EEZYbotArm import EEZYbotARM_Mk2
import numpy as np
import math

# init EEZYbotARM_Mk2
eezybot = EEZYbotARM_Mk2(initial_q1=0, initial_q2=70, initial_q3=-100)
# Assign cartesian position where we want the robot arm end effector to move to
# (x,y,z in mm from centre of robot base)
x = 240  # mm
y = 85  # mm
z = 200  # mm


# Compute inverse kinematics
a1, a2, a3 = eezybot.inverseKinematics(x, y, z)

# Print the result
#print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))
print(eezybot.plot())

app = Flask(__name__)


# Route for serving files (like SimpleHTTPRequestHandler does)
@app.route('/')
def serve_files():
    """Serve the index.html file."""
    return app.send_static_file('./index.html')


def handle_nan(arr):
    return [None if math.isnan(x) else x for x in arr]

@app.route('/send_data', methods=['GET'])
def send_data():
    # Create numpy arrays with NaN values
    data = [
        np.array([0, 0, 0]),
        np.array([ 0., -0., 92.]),
        np.array([ 0.,  0., 92.]),
        np.array([4.61727193e+01, 7.76784303e-15, 2.18858504e+02]),
        np.array([1.73478454e+02, 3.26726604e-15, 1.45358504e+02]),
        np.array([2.60478454e+02, 3.26726604e-15, 1.45358504e+02]),
        np.array([-5.19615242e+01,  1.83697020e-15,  1.22000000e+02]),
        np.array([float('nan'), 9.89251112e-15, 2.53556967e+02])  # Example of NaN
    ]
    
    # Convert numpy arrays to lists and handle NaN values
    data_as_lists = [handle_nan(arr.tolist()) for arr in data]
    
    # Send data as JSON
    return jsonify(data_as_lists)

if __name__ == '__main__':
    app.run(port=8004, debug=True)  # Set the port to 8004 and enable debug mode