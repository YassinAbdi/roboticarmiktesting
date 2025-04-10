from flask import Flask
import os
from EEZYbotArm import EEZYbotARM_Mk2



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
print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))
eezybot.plot()

app = Flask(__name__)


# Route for serving files (like SimpleHTTPRequestHandler does)
@app.route('/')
def serve_files():
    """Serve the index.html file."""
    return app.send_static_file('./index.html')



if __name__ == '__main__':
    app.run(port=8004, debug=True)  # Set the port to 8004 and enable debug mode