# File: main_control.py

# Import the specific class you want to use from the module file
from EEZYbotArm import EEZYbotARM_Mk2
import time # Example for delays

# --- Matplotlib Imports ---
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np # Needed for set_axes_equal helper

# --- Helper function for setting equal axes (optional but recommended for 3D) ---
def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# --- Visualization Function ---
def plot_arm(ax, joint_coords, title="EEZYbotARM Mk2"):
    """
    Plots the EEZYbotARM configuration on the given 3D axes.

    Args:
        ax: Matplotlib 3D axes object.
        joint_coords: Dictionary of joint coordinates from my_arm.plot().
                      Expected keys: 'base', 'j2', 'j3_elbow', 'j4_wrist', 'ee', 'a', 'b'
        title: Optional title for the plot.
    """
    # Clear previous plot elements
    ax.cla()

    # Get individual joint positions
    base = np.array(joint_coords['base'])
    j2 = np.array(joint_coords['j2'])
    j3 = np.array(joint_coords['j3_elbow'])
    j4 = np.array(joint_coords['j4_wrist'])
    ee = np.array(joint_coords['ee'])
    a = np.array(joint_coords['a'])
    b = np.array(joint_coords['b'])

    # --- Plot Links ---
    # Main arm links
    ax.plot([base[0], j2[0]], [base[1], j2[1]], [base[2], j2[2]], 'k-', linewidth=3, label='Base') # Base vertical
    ax.plot([j2[0], j3[0]], [j2[1], j3[1]], [j2[2], j3[2]], 'b-', linewidth=5, label='L2 (Shoulder-Elbow)') # Link 2
    ax.plot([j3[0], j4[0]], [j3[1], j3[1]], [j3[2], j4[2]], 'r-', linewidth=5, label='L3 (Elbow-Wrist)')  # Link 3
    ax.plot([j4[0], ee[0]], [j4[1], ee[1]], [j4[2], ee[2]], 'g-', linewidth=3, label='L4 (Wrist-EE)')    # Link 4 (EE offset)

    # Parallelogram linkage links
    ax.plot([j2[0], a[0]], [j2[1], a[1]], [j2[2], a[2]], 'c--', linewidth=2, label='Link J2-A') # J2 to A
    ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], 'm--', linewidth=2, label='Link A-B')    # A to B
    ax.plot([b[0], j4[0]], [b[1], b[1]], [b[2], j4[2]], 'y--', linewidth=2, label='Link B-J4')    # B to J4

    # --- Plot Joints ---
    # Combine all joint coordinates for scatter plot
    all_joints_x = [base[0], j2[0], j3[0], j4[0], ee[0], a[0], b[0]]
    all_joints_y = [base[1], j2[1], j3[1], j4[1], ee[1], a[1], b[1]]
    all_joints_z = [base[2], j2[2], j3[2], j4[2], ee[2], a[2], b[2]]
    ax.scatter(all_joints_x, all_joints_y, all_joints_z, c='k', marker='o', s=50, label='Joints/Points')
    ax.scatter(ee[0], ee[1], ee[2], c='lime', marker='X', s=100, label='End Effector') # Highlight EE


    # --- Plot Settings ---
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(title)

    # Set reasonable limits based on arm dimensions (adjust if needed)
    max_reach_xy = my_arm.L2 + my_arm.L3 + my_arm.L4 + 20 # Add buffer
    max_reach_z = my_arm.L1 + my_arm.L2 + my_arm.L3 + 20 # Add buffer
    ax.set_xlim([-max_reach_xy, max_reach_xy])
    ax.set_ylim([-max_reach_xy, max_reach_xy])
    ax.set_zlim([0, max_reach_z])

    # Apply equal aspect ratio using the helper function
    set_axes_equal(ax)

    ax.legend()
    ax.grid(True)


# --- Main Execution ---

# --- Initial Setup ---
initial_q1 = 0.0
initial_q2 = 90.0
initial_q3 = -90.0

print(f"Initializing EEZYbotARM_Mk2 with angles: q1={initial_q1}, q2={initial_q2}, q3={initial_q3}")

try:
    # Instantiate the arm object
    my_arm = EEZYbotARM_Mk2(initial_q1, initial_q2, initial_q3)
    print("\nArm initialized successfully.")

    # --- Set up Matplotlib Figure ---
    fig = plt.figure(figsize=(10, 8)) # Adjust figure size if needed
    ax = fig.add_subplot(111, projection='3d')

    # --- Plot Initial Position ---
    print("\nCalculating and plotting initial position...")
    initial_coords = my_arm.plot() # Get coordinates for initial pose
    plot_arm(ax, initial_coords, title=f"Initial Pose: q=({my_arm.q1:.1f}, {my_arm.q2:.1f}, {my_arm.q3:.1f})")
    plt.show(block=False) # Show plot without blocking execution yet
    plt.pause(1) # Pause briefly to see the initial plot

    # --- Perform IK and Move ---
    target_x = 200.0
    target_y = 50.0
    target_z = 150.0
    print(f"\nCalculating Inverse Kinematics for target: x={target_x}, y={target_y}, z={target_z}...")
    try:
        ik_q1, ik_q2, ik_q3 = my_arm.inverseKinematics(target_x, target_y, target_z)
        print(f"--> Required Joint Angles: q1={ik_q1:.2f} deg, q2={ik_q2:.2f} deg, q3={ik_q3:.2f} deg")

        print("\nUpdating arm angles and re-plotting...")
        my_arm.updateJointAngles(ik_q1, ik_q2, ik_q3)
        print(f"Arm angles updated to: q1={my_arm.q1}, q2={my_arm.q2}, q3={my_arm.q3}")

        # --- Plot New Position ---
        new_coords = my_arm.plot() # Get coordinates for the new pose
        plot_arm(ax, new_coords, title=f"Moved Pose: q=({my_arm.q1:.1f}, {my_arm.q2:.1f}, {my_arm.q3:.1f})")
        plt.draw() # Update the plot
        print("\nDisplaying final plot. Close the plot window to exit.")
        plt.show() # Now block until the plot window is closed


    except ValueError as e:
        print(f"IK Calculation or Angle Update failed: {e}")
        print("\nDisplaying initial plot only. Close the plot window to exit.")
        plt.show() # Show the initial plot if IK failed
    except Exception as e:
         print(f"An unexpected error occurred during IK/Update: {e}")
         plt.show()


    # --- Test invalid moves (optional, plot won't update for these) ---
    print("\nTesting movement to an invalid angle (q2 too low)...")
    try:
        my_arm.updateJointAngles(q1=0, q2=30, q3=-90) # q2=30 is below limit of 39
    except ValueError as e:
        print(f"Caught expected error: {e}")

    print("\nTesting IK for an unreachable point (too far)...")
    try:
        my_arm.inverseKinematics(x_EE=500, y_EE=0, z_EE=100) # Likely too far
    except ValueError as e:
         print(f"Caught expected error: {e}")


except ValueError as e:
    print(f"Initialization Error: {e}")
except Exception as e:
    print(f"An unexpected error occurred during initialization: {e}")

print("\nScript finished.")