from ArmConfig import *
import time
import math
if MIRCOPYTHON:
    from machine import Pin
    
else:
    class Pin:
        OUT = 1
        IN = 0

        def __init__(self, pin_number, mode):
            self.pin_number = pin_number
            self.mode = mode
            self._val = 0  # Internal state

        def value(self, value=None):
            if value is not None:
                self._val = value
            return self._val
            
    class time:
        @staticmethod
        def sleep_us(microseconds):
            pass

class Arm():
    def __init__(self):
        self.M1_dir = 19
        self.M1_step = 18
        self.M2_dir = 23
        self.M2_step = 22
        self.steps_per_rev = 200

        self.M1_dir_pin = Pin(self.M1_dir, Pin.OUT)
        self.M1_step_pin = Pin(self.M1_step, Pin.OUT)
        self.M2_dir_pin = Pin(self.M2_dir, Pin.OUT)
        self.M2_step_pin = Pin(self.M2_step, Pin.OUT)

        
        self.M1_current_position = 0
        self.M2_current_position = 0

        self.Mircostepping = 16
        self.gear_ratio_base = 2.6
        self.gear_ratio_arm1 = 2.6
        self.gear_ratio_arm2 = 2.6
        
        self.ArmCurrentPos = [0, 0, 0]  # Initialize current position of the arm


    
    def step_motor(self,delay_us, direction, motor):
        self.M1_dir_pin.value(direction)  # Set direction
        dir_pin = None
        step_pin = None
        if motor == 1:
            dir_pin = self.M1_dir_pin
            step_pin = self.M1_step_pin
        elif motor == 2:
            dir_pin = self.M2_dir_pin
            step_pin = self.M2_step_pin
        else:
            print("Invalid motor number. Use 1 or 2.")
            return
        
        dir_pin.value(direction)  # Set direction
        
        print("Spinning Clockwise..." if direction else "Spinning Anti-Clockwise...")

        for _ in range(self.steps_per_rev):
            step_pin.value(1)
            time.sleep_us(delay_us)
            step_pin.value(0)
            time.sleep_us(delay_us)

    def move_motor(self, target_position,motor, step_delay_us=2000):

        dir_pin = None
        step_pin = None
        current_pos = None
        if motor == 1:
            current_pos = self.M1_current_position
            dir_pin = Pin(self.M1_dir_pin, Pin.OUT)
            step_pin = Pin(self.M1_step_pin, Pin.OUT)
        elif motor == 2:
            current_pos = self.M2_current_position
            dir_pin = Pin(self.M2_dir_pin, Pin.OUT)
            step_pin = Pin(self.M2_step_pin, Pin.OUT)
        else:
            print("Invalid motor number. Use 1 or 2.")
            return
        # Calculate steps required
        steps_needed = target_position - current_pos
        direction = 1 if steps_needed > 0 else 0
        # Set direction
        dir_pin.value(direction)
        print(f"Moving to position {target_position} ({'CW' if direction else 'CCW'})")
        # Move motor
        for _ in range(abs(steps_needed)):
            step_pin.value(1)
            time.sleep_us(step_delay_us)
            step_pin.value(0)
            time.sleep_us(step_delay_us)
        # Update current position
        if motor == 1:
            self.M1_current_position = target_position
        elif motor == 2:
            self.M2_current_position = target_position
        print(f"Reached position: {target_position}")

    def angle_to_steps(self,angle, gear_ratio):
        """
        Convert angle (degrees) to stepper motor steps based on gear ratio and microstepping.
        """
        total_steps_per_deg = ((self.steps_per_rev * self.Mircostepping)/360)
        steps = angle * total_steps_per_deg
        return int(steps)

    def move_to_angle(self,b, a1, a2, g):
        """
        Convert angles to stepper motor steps based on gear ratio and microstepping.
        """
        base_steps = self.angle_to_steps(b + 90, self.gear_ratio_base)        # Base: 0° means pointing forward

        # Arm1: 90° is home (step=0), subtract 90
        arm1_steps = self.angle_to_steps(a1 - 90, self.gear_ratio_arm1)

        # Arm2: adjust to your real zero (let's assume 0° = -101 offset for now)
        arm2_steps = self.angle_to_steps(a2 + 101, self.gear_ratio_arm2)  # You can change this too

        print("Stepper motor commands:")
        print(f"  Base motor: {base_steps} steps")
        print(f"  Arm1 motor: {arm1_steps} steps")
        print(f"  Arm2 motor: {arm2_steps} steps")
        print(f"  Gripper: {g}° (manual or servo)")
        # Here you would send the steps to the motor driver
        self.move_motor(arm1_steps*-1, 1)  # Move arm1 in the opposite direction
        self.move_motor(arm2_steps, 2)  # Move arm2
    
    
    def move_to_pos(self,x, y, z, g):
        """
        Compute joint angles to reach target position (x, y, z) and convert to motor steps.
        Uses delta from current_pos and updates it afterward.
        """
        dx = x - self.ArmCurrentPos[0]
        dy = y - self.ArmCurrentPos[1]
        dz = z - self.ArmCurrentPos[2]

        print(f"Moving delta: dx={dx}, dy={dy}, dz={dz}")

        # Use target pos to compute angles
        b = math.atan2(y, x) * (180 / math.pi)
        l = math.sqrt(x**2 + y**2)
        h = math.sqrt(l**2 + z**2)
        phi = math.atan2(z, l) * (180 / math.pi)
        theta = math.acos(min(max(h / 2 / 75, -1), 1)) * (180 / math.pi)

        a1 = phi + theta
        a2 = phi - theta

        print(f"Calculated angles (degrees):")
        print(f"  Base: {b:.2f}, Arm1: {a1:.2f}, Arm2: {a2:.2f}")

        self.move_to_angle(b, a1, a2, g)

        # Update current position after moving
        self.ArmCurrentPos[0] = x
        self.ArmCurrentPos[1] = y
        self.ArmCurrentPos[2] = z
