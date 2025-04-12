from flask import Flask, jsonify
from EEZYbotArm import EEZYbotARM_Mk2
import numpy as np
import math

# init EEZYbotARM_Mk2
eezybot = EEZYbotARM_Mk2(initial_q1=0, initial_q2=70, initial_q3=-100)
# # Assign cartesian position where we want the robot arm end effector to move to
# # (x,y,z in mm from centre of robot base)
# x = 240  # mm
# y = 85  # mm
# z = 200  # mm


# # Compute inverse kinematics
# a1, a2, a3 = eezybot.inverseKinematics(x, y, z)

# # Print the result
# #print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))
# print(eezybot.plot())

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
    data = eezybot.plot()
    # Convert numpy arrays to lists and handle NaN values
    data_as_lists = [handle_nan(arr.tolist()) for arr in data]
    
    # Send data as JSON
    return jsonify(data_as_lists)

# route for calling the inverse kinematics
@app.route('/inverse_kinematics/<x>/<y>/<z>', methods=['GET'])
def inverse_kinematics(x, y, z):
    """Calculate inverse kinematics for given x, y, z."""
    x = float(x)
    y = float(y)
    z = float(z)

    # Compute inverse kinematics
    a1, a2, a3 = eezybot.inverseKinematics(x, y, z)

    # Return the joint angles as JSON
    return jsonify({'q1': a1, 'q2': a2, 'q3': a3})

# route for calling the inverse kinematics and setting the angles
@app.route('/set_angles/<q1>/<q2>/<q3>', methods=['GET'])
def set_angles(q1, q2, q3):
    """Set the joint angles."""
    q1 = float(q1)
    q2 = float(q2)
    q3 = float(q3)

    # Set the joint angles
    eezybot.setJointAngles(q1, q2, q3)

    # Return the new angles as JSON
    return jsonify({'q1': q1, 'q2': q2, 'q3': q3})


if __name__ == '__main__':
    app.run(port=8004, debug=True)  # Set the port to 8004 and enable debug mode