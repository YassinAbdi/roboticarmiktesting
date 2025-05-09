# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
import matplotlib.pyplot as plt
import requests
import time
# Initialise robot arm with initial joint angles
myRobotArm = EEZYbotARM_Mk2(initial_q1=0, initial_q2=90, initial_q3=-90)
starting_x = 234.0
starting_y = 0.0
starting_z = 227.0
print(myRobotArm.forwardKinematics(q1=0, q2=90, q3=-90))  # Forward kinematics to get the end effector position

url = "http://10.0.0.114"

# Assign cartesian position where we want the robot arm end effector to move to
# (x,y,z in mm from centre of robot base)
x = 240  # mm
y = 85  # mm
z = 100  # mm

# Compute inverse kinematics
a1, a2, a3 = myRobotArm.inverseKinematics(x, y, z)

# Print the result
print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))

# Motor and gear setup for arm (assuming Motors 1 and 2 control arm joints)
steps_per_rev = 200  # 1.8° stepper motor base resolution
microsteps = 16       # Microstepping factor
gear_ratio_base = 2.6
gear_ratio_arm1 = 2.6
gear_ratio_arm2 = 2.6


def angle_to_steps(angle, gear_ratio, motor_steps_per_rev=steps_per_rev, microstepping_factor=microsteps):
    """
    Convert angle (degrees) to stepper motor steps based on gear ratio and microstepping.
    """
    total_steps_per_rev_geared = motor_steps_per_rev * microstepping_factor * gear_ratio
    total_steps_per_deg = total_steps_per_rev_geared / 360
    steps = angle * total_steps_per_deg
    return int(steps)

def convert_to_stepper_angle(a1, a2, a3):
    # Convert angles to stepper motor angles
    stepper_a1 = a1 - myRobotArm.q1  
    stepper_a2 = a2 - myRobotArm.q2 
    stepper_a3 = a3 - myRobotArm.q3 
    print('The stepper angles are a1 = {}, a2= {}, a3 = {}'.format(stepper_a1, stepper_a2, stepper_a3))
    # Determine direction for each motor
    dir1 = 1 if stepper_a1 > 0 else 0 # 1 for CW, 0 for CCW
    dir2 = 1 if stepper_a2 > 0 else 0
    dir3 = 1 if stepper_a3 > 0 else 0
    print('The stepper directions are dir1 = {}, dir2= {}, dir3 = {}'.format(dir1, dir2, dir3))
    base_steps = angle_to_steps(stepper_a1, gear_ratio_base)
    arm1_steps = angle_to_steps(stepper_a2, gear_ratio_arm1)
    arm2_steps = angle_to_steps(stepper_a3, gear_ratio_arm2)

    print('The stepper steps are base_steps = {}, arm1_steps= {}, arm2_steps = {}'.format(base_steps, arm1_steps, arm2_steps))
    try:
        params = {
            "base_steps": base_steps,
            "base_dir": dir1,
            "arm1_steps": arm1_steps,
            "arm1_dir": dir2,
            "arm2_steps": arm2_steps,
            "arm2_dir": dir3
        }
        print("########################################################")
        print(f"{url}/move_by_steps")
        response = requests.get(f"{url}/bmove_by_steps", params=params)
        print("Status Code:", response.status_code)
    except Exception as e:
        print("Error sending steps:", e)
        


def move_to_position(x, y, z):
    # Compute inverse kinematics
    a1, a2, a3 = myRobotArm.inverseKinematics(x, y, z)
    print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))
    convert_to_stepper_angle(a1, a2, a3)
    myRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
    print('The new joint angles are q1 = {}, q2= {}, q3 = {}'.format(myRobotArm.q1, myRobotArm.q2, myRobotArm.q3))
    myRobotArm.plot()
    

def move_to_start():
    # Move to starting position
    print('Moving to starting position')
    move_to_position(starting_x, starting_y, starting_z)

def check_joint_limits(x, y, z):
    # Check if the position is within joint limits
    q1, q2, q3 = myRobotArm.inverseKinematics(x, y, z)
    if myRobotArm.checkErrorJointLimits(q1=q1, q2=q2, q3=q3):
        print("Position is within joint limits.")
        return True
    else:
        print("Position exceeds joint limits.")
        return False

def visualise_joint_angles(x, y, z):
    # Visualise the new joint angles
    a1, a2, a3 = myRobotArm.inverseKinematics(x, y, z)
    print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))
    myRobotArm.q1 = a1
    myRobotArm.q2 = a2
    myRobotArm.q3 = a3
    print('The new joint angles are q1 = {}, q2= {}, q3 = {}'.format(myRobotArm.q1, myRobotArm.q2, myRobotArm.q3))
    myRobotArm.plot()
# move_to_position(x, y, z)
# time.sleep(10)
# move_to_start()
# # Visualise the new joint angles

current_z = 150
def move_to_gcode(gcode):
    global current_z
    # inverse kinematics
    # Extract the G-code command and arguments
    command = gcode.get("command")
    args = gcode.get("args")
    if command in list_of_gcode:
        if command == "G0" or command == "G1":
            if "X" in args and "Y" in args:
                # Extract X, Y, Z coordinates from the arguments
                x = float(args.get("X", 0)) + 140
                y = float(args.get("Y", 0)) - 50 
                if "Z" in args:
                    z = float(args.get("Z", 0)) + 150
                    current_z = z
                    print(f"Moving to position: X={x}, Y={y}, Z={z}")
                else:
                    z = current_z
                    print(f"Moving to position without z: X={x}, Y={y}, Z={z}")
                # move_to_position(x, y, z)
                #visualise_joint_angles(x, y, z)
                check_joint_limits(x, y, z)
                

list_of_gcode = [   
"G0",
"G1",
"G2",
"G3",
"G4",
"G28",
"G90",
"G91",
"G92",
"M0",
"M18",
"M84",
"M104",
"M105",
"M106",
"M109",
"M112",
"M114",
"M140",
"M190",
"M220",
"M221"
]

def parse_gcode_line(line):
    """
    Parses a single line of G-code and returns the command and its arguments.
    """
    line = line.strip()  # Remove leading/trailing whitespace
    if not line or line.startswith(";"):  # Ignore empty lines and comments
        return None

    parts = line.split()
    command = parts[0]  # The G-code command (e.g., G0, M104)
    args = {}

    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=")
            args[key] = value
        else:
            key = part[0]
            value = part[1:]
            args[key] = value

    return {"command": command, "args": args}

with open("./mircopython/3DBenchy_PLA_1h6m.gcode") as f:
        count = 0
        for line in f:
            gcode = parse_gcode_line(line)
            if gcode and count > 401:
                print("Parsed G-code:", gcode)
                move_to_gcode(gcode)
                #time.sleep(0.1)
            print("CURRENT LINE:", line, "COUNT:", count)
            count += 1
        # lines = f.readlines()
        # specific_line = lines[29]  # Change the index to the line you want
        # gcode = parse_gcode_line(specific_line)
        # print("Parsed G-code:", gcode)
        # move_to_gcode(gcode)