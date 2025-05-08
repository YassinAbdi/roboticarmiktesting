import time
from machine import Pin
# Define pins for the stepper motor

# Define constants for Motor 1 (Base)
DIR1 = 21
STEP1 = 18
STEPS_PER_REV1 = 100 # Example value, adjust as needed

# Initialize pins for Motor 1
dir_pin1 = Pin(DIR1, Pin.OUT)
step_pin1 = Pin(STEP1, Pin.OUT)

# Generic step motor function (can be used for any motor by passing pins)
def step_motor_generic(step_pin, dir_pin, delay_us, direction, steps_to_take):
    dir_pin.value(direction)  # Set direction
    # print(f"Spinning {'Clockwise' if direction else 'Anti-Clockwise'} for {steps_to_take} steps...")
    print(f"Spinning {'Clockwise' if direction else 'Anti-Clockwise'} for {steps_to_take} steps...")
    for _ in range(steps_to_take):
        step_pin.value(1)
        time.sleep_us(delay_us)
        step_pin.value(0)
        time.sleep_us(delay_us)


step_motor_generic(step_pin1, dir_pin1, 2000, 0, 600)