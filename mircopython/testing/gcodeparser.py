"""
Ryan Hoover, Faculty in Interdisciplinary Sculpture, Maryland Institute College of Art, Digital Fabrication Studio, 2015
Code Description
;   comment
G0  Rapid Movement
G1  Coordinated Movement X Y Z E
G2  CW ARC
G3  CCW ARC
G4  Dwell S<seconds> or P<milliseconds>
G28 Home all Axis
G90 Use Absolute Coordinates
G91 Use Relative Coordinates
G92 Set current position to coordinates given
M0  Unconditional stop
M18  Disable all stepper motors; same as M84
M84  Disable steppers until next move or set an inactivity timeout
M104 Set extruder target temp
M105 Read current temp
M106 Fan on
M109 Set extruder target temp and wait for it to be reached
M112 Emergency stop
M114 Output current position to serial port
M140 Set bed target temp
M190 Set bed target temp and wait for it to be reached
M220 set speed factor override percentage
M221 set extrude factor override percentage
"""

from EEZYbotArm import EEZYbotARM_Mk2

# init EEZYbotARM_Mk2
eezybot = EEZYbotARM_Mk2(initial_q1=0, initial_q2=70, initial_q3=-100)

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


# Define your offsets here
X_OFFSET = -50  # Example: shift G-code X by -70 mm
Y_OFFSET = 45 # Example: shift G-code Y by -90 mm
Z_OFFSET = 50    # Example: no change to Z

# grab a specific line
with open("./benchy.gcode") as f:
    lines = f.readlines()
    specific_line = lines[25]  # Change the index to the line you want
    gcode = parse_gcode_line(specific_line)
    print(gcode)
    print(eezybot.plot())

    # convert to float and apply offset
    gcode['args']['X'] = float(gcode['args']['Y']) + X_OFFSET
    gcode['args']['Y'] = float(gcode['args']['X']) + Y_OFFSET
    gcode['args']['Z'] = float(gcode['args']['Z']) + Z_OFFSET

    print(f"Adjusted coords: X={gcode['args']['X']}, Y={gcode['args']['Y']}, Z={gcode['args']['Z']}")

    data = eezybot.inverseKinematics(gcode['args']['X'], gcode['args']['Y'], gcode['args']['Z'])
    print(data)
    pdate = eezybot.updateJointAngles(data[0], data[1], data[2])
    print(eezybot.plot())