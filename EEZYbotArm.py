from math import *
import numpy as np

class EEZYbotARM:
    """
    --Description--
    This is a parent class for the EExybot Robotic arm

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - __init__ --> Initialise a robotic arm with current joint position
        - updateJointAngles --> Update the stored joint angles for the robot arm
        - checkErrorJointLimits --> Check if any of the supplied joint angles are outside of physical limits
        - FK_EEZYbotARM --> Perform forward kinematics to find the end effector position, given joint angles
        - IK_EEZYbotARM --> Perform inverse kinematics to find the joint angles, given end effector position
        - plot_EEZYbotARM --> Plot the robotic arm in 3D using matplotlib Axes 3D

    """

    # Class attribute
    # e.g. species = 'mammal'

    # Initializer / Instance attributes
    def __init__(self, initial_q1, initial_q2, initial_q3):
        self.q1 = initial_q1
        self.q2 = initial_q2
        self.q3 = initial_q3

    # Instance methods
    def updateJointAngles(self, q1, q2, q3):
        """
        --Description--
        Update the three EEzybot Arm joint angles (by providing values in degrees)
        Will return an error message if the joint angles are outside of given limits

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees)
        @q2 -> the value of the angle q2 (in degrees) 
        @q3 -> the value of the angle q3 (in degrees)

        --Returns--
        Function doesn't return a value

        """
        # Check given values are inside valid limits
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Assign new angles
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def checkErrorJointLimits(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate whether the EzzyBot Arm is outside of joint limits
        If outside of joint limts return error code for appropriate joint
        Otherwise return zero value

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """
        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Find angle limits
        q3_min, q3_max = self.q3CalcLimits(q2=q2)

        # Check given angles are within limits, else raise exception
        if q1 < self.q1_min or q1 > self.q1_max:
            raise Exception('Value for q1 is outside joint limits. Joint limits in degrees are ({},{})'.format(
                self.q1_min, self.q1_max))

        if q2 < self.q2_min or q2 > self.q2_max:
            raise Exception('Value for q2 is outside joint limits. Joint limits in degrees are ({},{})'.format(
                self.q2_min, self.q2_max))

        if q3 < q3_min or q3 > q3_max:
            raise Exception(
                'Value for q3 is outside joint limits. Joint limits in degrees are ({},{})'.format(q3_min, q3_max))

    def forwardKinematics(self, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3, find the forward kinematics for the end effector position of the EEZYbotARM

        If q1, q2, q3 are not provided then the current arm angles will be used (self.q1,q2,q3)

        The forward kinematics uses DH proximal convention and the method is disclosed seperate to this code

        The reference angle definitions are as follows:

        - EzzyBot base (q1) : 0 degree position is facing directly forwards
        - Main arm position (q2)
        - Horarm position(q3)

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        x_EE, y_EE, z_EE -> position of the end effector in the world frame (mm)

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Convert to radians
        q1 = q1 * pi/180
        q2 = q2 * pi/180
        q3 = q3 * pi/180

        # Joint length definitions (for ease of reading code)
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Find the position of the end effector using the forward kinematics equations
        x_EE = round((cos(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*cos(q1)), 3)
        y_EE = round((sin(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*sin(q1)), 3)
        z_EE = round((L1 + sin(q2)*L2 + sin(q2+q3)*L3), 3)

        # Return the end effector position in (mm)
        return x_EE, y_EE, z_EE

    def inverseKinematics(self, x_EE, y_EE, z_EE):
        """
        --Description--
        Function: to find the Inverse Kinematics for the EEZYbotARM Mk2 

        Description: given x y z positions of the desired EE position inside the
        manipulator workspace, find the corresponding joint angles

        --Parameters--
        x_EE, y_EE, z_EE -> the cartesian position of the end effector in the world frame

        --Returns--
        q1, q2, q3 -> Corresponding joint angles in degrees

        """
        # DH parameters (Proximal Convention)
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Find the value for the fist angle
        q1 = atan2(y_EE, x_EE)

        # Find the values for the position of joint #4 for x, y, z
        x_4 = x_EE - (L4 * cos(q1))
        y_4 = y_EE - (L4 * sin(q1))
        z_4 = z_EE

        # Find the length of the third side of the (virtual) triangle made by L2,
        # L3 in the vertical plane of the robot arm

        # --- Specify the z poisition of joint #1
        z_1 = L1

        # --- First we find the z distance between joint #1 and joint #4 (in the world frame)
        z_1_4 = z_4 - z_1

        # --- Find the true distance (in x, y plane) to joint #4
        xy_4 = sqrt((x_4**2)+(y_4**2))

        # --- Then we find the length of the virtual side made by the right angle triangle
        v_side = sqrt((z_1_4**2) + (xy_4**2))

        # Find the value for the angle at joint #3
        q3 = - (pi - acos((L2**2 + L3**2 - v_side**2)/(2 * L2 * L3)))

        # Find the value for the angle at joint #2 %DEBUG HERE
        # --- Find the value of the angle from the x-y plane to the virtual side
        q2_a = atan2(z_1_4, xy_4)

        q2_b = acos((v_side**2 + L2**2 - L3**2)/(2 * v_side * L2))

        q2 = q2_a + q2_b  # NOTE there's some more work to do here to make this correctly summation or subtraction dependant on the position of the arm!

        # # Print the input world frame position of the end effector
        # print('Input Position of End Effector: \n')
        # print('x_EE: {}'.format(x_EE))
        # print('y_EE: {}'.format(y_EE))
        # print('z_EE: {} \n'.format(z_EE))

        # # Print the output joint angles
        # print('Ouput joint angles: \n')
        # print('q1: {:+.2f}'.format(q1 * 180/pi))
        # print('q2: {:+.2f}'.format(q2 * 180/pi))
        # print('q3: {:+.2f} \n'.format(q3 * 180/pi))

        # round values
        q1 = round(q1 * 180/pi, 2)
        q2 = round(q2 * 180/pi, 2)
        q3 = round(q3 * 180/pi, 2)

        return q1, q2, q3
    
    def plot(self, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3, calculate joint positions for plotting EEZybot arm using matplotlib.
        Function computes the configuration of the manipulator using DH proximal convention.

        This function *calculates* the points but does *not* perform the plotting itself.
        It returns the key joint positions needed for external plotting.

        The function will raise an exception if any of the angles are outside defined limits

        --Optional **kwargs parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        joint_positions -> A dictionary containing key joint coordinates:
                           {'base': [x,y,z], 'j2': [x,y,z], 'j3_elbow': [x,y,z], 'j4_wrist': [x,y,z], 'ee': [x,y,z],
                            'a': [x,y,z], 'b': [x,y,z]}
                           (Coordinates are in mm)
        """

        # Use **kwargs if provided, otherwise use current values
        q1_deg = kwargs.get('q1', self.q1)
        q2_deg = kwargs.get('q2', self.q2)
        q3_deg = kwargs.get('q3', self.q3)

        # Check limits first
        self.checkErrorJointLimits(q1=q1_deg, q2=q2_deg, q3=q3_deg)

        # Convert angles to radians
        q1 = q1_deg * pi/180
        q2 = q2_deg * pi/180
        q3 = q3_deg * pi/180 # Note: q3 definition (relative to L2 extension)

        # DH parameters (Proximal Convention), Link lengths
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4
        # Lengths for the links which attach to the hoarm parallelogram linkage
        L2A = self.L2A # Distance from J2 axis along L2 to point A
        LAB = self.LAB # Length of link AB
        LB3 = self.LB3 # Distance from J4 axis back along L3 to point B (Should match L2A for parallelogram)

        # --- Calculate Joint Positions Directly ---

        # Joint 1 (Base) - World Origin
        Base_Pos = np.array([0.0, 0.0, 0.0])

        # Joint 2 (Shoulder rotation axis - on top of base)
        J2_Pos = np.array([0.0, 0.0, L1])

        # Joint 3 (Elbow)
        J3_x = cos(q1) * (L2 * cos(q2))
        J3_y = sin(q1) * (L2 * cos(q2))
        J3_z = L1 + L2 * sin(q2)
        J3_Pos = np.array([J3_x, J3_y, J3_z])

        # Joint 4 (Wrist)
        J4_x = cos(q1) * (L2 * cos(q2) + L3 * cos(q2 + q3))
        J4_y = sin(q1) * (L2 * cos(q2) + L3 * cos(q2 + q3))
        J4_z = L1 + L2 * sin(q2) + L3 * sin(q2 + q3)
        J4_Pos = np.array([J4_x, J4_y, J4_z])

        # End Effector (EE) - Offset by L4 from Wrist along the final link's direction
        # Direction vector from Elbow (J3) to Wrist (J4)
        J4_minus_J3 = J4_Pos - J3_Pos
        norm_J4_minus_J3 = np.linalg.norm(J4_minus_J3)
        if norm_J4_minus_J3 > 1e-6:
             wrist_direction = J4_minus_J3 / norm_J4_minus_J3
        else: # Handle case where elbow and wrist coincide (arm fully extended or folded)
             # Use direction based on q2+q3 angle in the arm's plane
             wrist_direction_x_plane = cos(q1) * cos(q2+q3)
             wrist_direction_y_plane = sin(q1) * cos(q2+q3)
             wrist_direction_z_plane = sin(q2+q3)
             # Normalize this direction vector
             norm_dir = np.sqrt(wrist_direction_x_plane**2 + wrist_direction_y_plane**2 + wrist_direction_z_plane**2)
             if norm_dir > 1e-6:
                wrist_direction = np.array([wrist_direction_x_plane, wrist_direction_y_plane, wrist_direction_z_plane]) / norm_dir
             else: # Default to pointing along L2 if all else fails (e.g. q2+q3 = 0 or pi)
                 # This case might need more careful handling depending on desired behavior at singularity
                 dir_L2_x = cos(q1) * cos(q2)
                 dir_L2_y = sin(q1) * cos(q2)
                 dir_L2_z = sin(q2)
                 norm_L2 = np.sqrt(dir_L2_x**2 + dir_L2_y**2 + dir_L2_z**2)
                 if norm_L2 > 1e-6:
                     wrist_direction = np.array([dir_L2_x, dir_L2_y, dir_L2_z]) / norm_L2
                 else: # Failsafe: point straight up along Z if L2 is somehow zero length or q2=pi/2
                     wrist_direction = np.array([0.0, 0.0, 1.0])


        EE_Pos = J4_Pos + L4 * wrist_direction
        EE_Pos = np.round(EE_Pos, 3) # Apply rounding here after calculation

        # --- Calculate Parallelogram Linkage Points A and B ---
        # Point A: On Link 2, distance L2A from Joint 2 axis
        A_x = cos(q1) * (L2A * cos(q2))
        A_y = sin(q1) * (L2A * cos(q2))
        A_z = L1 + L2A * sin(q2)
        A_Pos = np.array([A_x, A_y, A_z])

        # Point B: On Link 3, distance LB3 from Joint 4 axis (measured back towards J3)
        # Vector from J4 towards J3
        J3_minus_J4 = J3_Pos - J4_Pos
        norm_J3_minus_J4 = np.linalg.norm(J3_minus_J4)
        if norm_J3_minus_J4 > 1e-6:
            vec_J4_to_J3 = J3_minus_J4 / norm_J3_minus_J4
        else: # Handle J3=J4 case
            vec_J4_to_J3 = -wrist_direction # Opposite direction of wrist

        # For a parallelogram, LB3 should equal L2A.
        B_Pos = J4_Pos + LB3 * vec_J4_to_J3

        # --- Create the dictionary to return ---
        joint_positions = {
            'base': Base_Pos.round(3).tolist(),
            'j2': J2_Pos.round(3).tolist(),
            'j3_elbow': J3_Pos.round(3).tolist(),
            'j4_wrist': J4_Pos.round(3).tolist(),
            'ee': EE_Pos.round(3).tolist(), # EE already rounded
            'a': A_Pos.round(3).tolist(),
            'b': B_Pos.round(3).tolist()
        }

        # Optional: Keep the debug print if you like, or remove it
        # print(f"plot method --> End effector calculated at: x:{EE_Pos[0]:.1f}, y:{EE_Pos[1]:.1f}, z:{EE_Pos[2]:.1f}")

        return joint_positions # Make sure this line returns the dictionary





# ------------------------------------------#
# Child class [inherits from EEZYbotARM() class]
# ------------------------------------------#

class EEZYbotARM_Mk2(EEZYbotARM):
    """
    --Description--
    This is a child class for the EEzybot Robotic arm MK2 (inherits from EEZYbotARM() class) 

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - q3CalcLimits --> Calculate the physical limits for joint 3 (because these depend on the angle of joint 2)
        - map_kinematicsToServoAngles --> Map angles defined in kinematics to physical servo angles on the robot
    """

    # Class attribute
    # DH parameters (Proximal Convention)
    L1 = 92  # mm
    L2 = 135
    L3 = 147
    L4 = 87

    # --- Lengths of links which attach to the hoarm
    L2A = 60
    LAB = 140
    LB3 = 60

    # Joint limits
    q1_min = -30  # degrees
    q1_max = 30
    q2_min = 39
    q2_max = 120
    # q3_min, q3_max are given by q3CalcLimits() function

    def q3CalcLimits(self, **kwargs):
        """
        Calculate q3 angle limits for the EzzyBot Arm given a value for the angle q2 in degrees
        These limits have been determined experimentally for the EEzybotMk2

        If no q2 value is given then the current value of q2 is used 

        --Optional kwargs Parameters--
        @q2 -> the value of the angle q2 (in degrees)

        --Returns--
        q3_min, q3_max -> the min and max limits for the angle q3 (in degrees)

        """
        # Use **kwarg if provided, otherwise use current q2 value
        q2 = kwargs.get('q2', self.q2)

        # calculate q3 min limits in degrees
        q3_min = (-0.6755 * q2) - 70.768
        q3_max = (-0.7165 * q2) - 13.144

        return q3_min, q3_max

    def map_kinematicsToServoAngles(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate the required physical servo position with respect to the reference position.
        If three angles are not provided as **kwargs then the current values for the arm are used

        The reference positions for the three servos are as follows:

        EzzyBot base (q1) : 90 degree servo position is facing directly forwards
        Main arm (q2): 90 degree servo position is with main arm perpendicular (at 90 degrees to) base
        Horarm (q3): 90 degree servo poisition is with horarm servo link at 45 degrees to base

        The function will be updated to raise an error message when any of the returned angles are outside of the servo limits.

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Check none of the angles are outside of joint limits! So that servos cannot get damaged
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Calculate for q1
        servoAngle_q1 = ((-2.0497)*q1) + 91.726  # from experimentation !
        servoAngle_q1 = round(servoAngle_q1, 2)

        # Calculate for q2
        servoAngle_q2 = 180 - q2  # approximate adjusted q2 value
        servoAngle_q2 = round(servoAngle_q2, 2)

        # Calculate for q3
        q3_a = 180 - (- q3)  # approximate adjusted q3 value
        servoAngle_q3 = q2 - 45 + q3_a
        servoAngle_q3 = round(servoAngle_q3, 2)

        return servoAngle_q1, servoAngle_q2, servoAngle_q3