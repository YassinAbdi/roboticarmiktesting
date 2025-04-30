import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import requests
import ast
from time import sleep
# Motor and gear setup
steps_per_rev = 200  # 1.8° stepper motor
microsteps = 16       # Microstepping factor
gear_ratio_base = 2.6
gear_ratio_arm1 = 2.6
gear_ratio_arm2 = 2.6

# url
url = "http://10.0.0.116"

# Track the current position of the arm
current_pos = [0, 0, 0]  # x, y, z

def get_current_pos():
    global current_pos, url
    # get current position
    response = requests.get(f"{url}/current_pos")

    print("Status Code:", response.status_code)
    print("Response Text:", response.text)
    text = response.text.strip()
    if text.startswith("Current position: "):
        position_str = text.replace("Current position: ", "")
        current_pos = ast.literal_eval(position_str)  # safely evaluates string as Python literal
        print(f"Current position: {current_pos}")
    else:
        print("Unexpected format:", text)
        current_pos = None
    return current_pos

def angle_to_steps(angle, gear_ratio):
    """
    Convert angle (degrees) to stepper motor steps based on gear ratio and microstepping.
    """
    total_steps_per_deg = ((steps_per_rev * microsteps)/360)
    steps = angle * total_steps_per_deg
    return int(steps)

def move_to_angle(b, a1, a2, g):
    """
    Convert angles to stepper motor steps based on gear ratio and microstepping.
    """
    base_steps = angle_to_steps(b + 90, gear_ratio_base)        # Base: 0° means pointing forward

    # Arm1: 90° is home (step=0), subtract 90
    arm1_steps = angle_to_steps(a1 - 90, gear_ratio_arm1)

    # Arm2: adjust to your real zero (let's assume 0° = -101 offset for now)
    arm2_steps = angle_to_steps(a2 + 101, gear_ratio_arm2)  # You can change this too

    print("Stepper motor commands:")
    print(f"  Base motor: {base_steps} steps")
    print(f"  Arm1 motor: {arm1_steps} steps")
    print(f"  Arm2 motor: {arm2_steps} steps")
    print(f"  Gripper: {g}° (manual or servo)")

def calculate_angles(x, y, z):
    """
    Calculate angles for base, arm1, and arm2 given target position (x, y, z).
    """
    b = math.atan2(y, x) * (180 / math.pi)
    l = math.sqrt(x**2 + y**2)
    h = math.sqrt(l**2 + z**2)
    phi = math.atan2(z, l) * (180 / math.pi)
    theta = math.acos(min(max(h / 2 / 75, -1), 1)) * (180 / math.pi)

    a1 = phi + theta
    a2 = phi - theta

    return b, a1, a2
def move_to_pos(x, y, z, g):
    """
    Compute joint angles to reach target position (x, y, z) and convert to motor steps.
    Uses delta from current_pos and updates it afterward.
    """
    dx = x - current_pos[0]
    dy = y - current_pos[1]
    dz = z - current_pos[2]

    print(f"Moving delta: dx={dx}, dy={dy}, dz={dz}")

    # Calculate angles
    b, a1, a2 = calculate_angles(dx, dy, dz)

    print(f"Calculated angles (degrees):")
    print(f"  Base: {b:.2f}, Arm1: {a1:.2f}, Arm2: {a2:.2f}")

    move_to_angle(b, a1, a2, g)
    visualize_arm(b, a1, a2)
    # Update current position after moving
    current_pos[0] = x
    current_pos[1] = y
    current_pos[2] = z

def visualize_arm(b, a1, a2):
    """
    3D Visualization of arm pose given joint angles.
    """
    b_rad = math.radians(b)
    a1_rad = math.radians(a1)
    a2_rad = math.radians(a2)

    arm1_length = 135
    arm2_length = 140

    base_x, base_y, base_z = 0, 0, 0

    # Arm 1 joint position
    arm1_end_x = arm1_length * math.cos(a1_rad) * math.cos(b_rad)
    arm1_end_y = arm1_length * math.cos(a1_rad) * math.sin(b_rad)
    arm1_end_z = arm1_length * math.sin(a1_rad)

    # Arm 2 joint position
    arm2_end_x = arm1_end_x + arm2_length * math.cos(a2_rad) * math.cos(b_rad)
    arm2_end_y = arm1_end_y + arm2_length * math.cos(a2_rad) * math.sin(b_rad)
    arm2_end_z = arm1_end_z + arm2_length * math.sin(a2_rad)

    # Plotting
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Base point
    ax.scatter(base_x, base_y, base_z, color='black', label='Base')

    # Arm segments
    ax.plot([base_x, arm1_end_x], [base_y, arm1_end_y], [base_z, arm1_end_z], color='blue', label='Arm 1')
    ax.plot([arm1_end_x, arm2_end_x], [arm1_end_y, arm2_end_y], [arm1_end_z, arm2_end_z], color='green', label='Arm 2')

    # Gripper point
    ax.scatter(arm2_end_x, arm2_end_y, arm2_end_z, color='red', label='Gripper')

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('3D Robot Arm Visualization')
    ax.legend()
    ax.set_box_aspect([1, 1, 1])
    plt.ion()  # Turn on interactive mode
    plt.tight_layout()
    plt.show()
    plt.draw()
    plt.pause(.001)  # Short pause so the plot window updates


def move_robot_to_pos(x, y, z, g):
    """
    Move the robot to a specified position (x, y, z) with gripper angle g.
    """

    # Use target pos to compute angles
    b, a1, a2 = calculate_angles(x, y, z)

    print(f"Calculated angles (degrees):")
    print(f"  Base: {b:.2f}, Arm1: {a1:.2f}, Arm2: {a2:.2f}")
    move_to_angle(b, a1, a2, g)
    visualize_arm(b, a1, a2)
    try:
        params = {
            "x": x,
            "y": y,
            "z": z
        }
        response = requests.get(f"{url}/smove_to_pos", params=params)
        print("Status Code:", response.status_code)
        print("Response Text:", response.text)
    except Exception as e:
        print("Error sending position:", e)


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


# Example usage
if __name__ == "__main__":
    lastz = 0
    #move_to_pos(current_pos[0], current_pos[1], current_pos[2], 90)
    #move_to_pos(200, 0, 0, 90)
    #move_robot_to_pos(200, 0, 0, 90)
    with open("./benchy.gcode") as f:
        lines = f.readlines()
        specific_line = lines[29]  # Change the index to the line you want
        gcode = parse_gcode_line(specific_line)
        print("Parsed G-code:", gcode)
        if("X" in gcode["args"]):
            x = float(gcode["args"]["X"])
        if("Y" in gcode["args"]):
            y = float(gcode["args"]["Y"])

        if("Z" in gcode["args"]):
            z = float(gcode["args"]["Z"])
            lastz = z
        print("X:", x, "Y:", y, "Z:", lastz)
        move_robot_to_pos(x, y, lastz, 90)
        sleep(5)
        move_robot_to_pos(150, 0, 30, 90)
    # move_robot_to_pos(150, 0, 30, 90)
    # sleep(30)
    # move_robot_to_pos(0, 0, 50, 90)
    while True:
        plt.pause(0.1)  # Keep the plot open