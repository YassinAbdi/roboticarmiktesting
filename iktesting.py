# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
import requests
# Initialise robot arm with initial joint angles
myRobotArm = EEZYbotARM_Mk2(initial_q1=0, initial_q2=90, initial_q3=-90)
myRobotArm.plot()  # plot it
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
        
convert_to_stepper_angle(a1, a2, a3)
# Visualise the new joint angles
myRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
print('The new joint angles are q1 = {}, q2= {}, q3 = {}'.format(myRobotArm.q1, myRobotArm.q2, myRobotArm.q3))
myRobotArm.plot()
# try:
#     params = {
#         "x": x,
#         "y": y,
#         "z": z
#     }
#     response = requests.get(f"{url}/smove_to_pos", params=params)
#     print("Status Code:", response.status_code)
#     #print("Response Text:", response.text)
# except Exception as e:
#     print("Error sending position:", e)
# #http://10.0.0.114/tmove_to_angle_cmd?b=19.5&a1=80.2&a2=-90.1&g=0