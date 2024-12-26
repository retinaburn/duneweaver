#Source: electrocredible.com, Language: MicroPython
import utime
from machine import Pin

# Define the pins for the stepper motor
#stepper_pins = [Pin(16, Pin.OUT), Pin(17, Pin.OUT), Pin(14, Pin.OUT), Pin(15, Pin.OUT)]
ROTstepper_pins = [Pin(4, Pin.OUT), Pin(3, Pin.OUT), Pin(2, Pin.OUT), Pin(1, Pin.OUT)]
MOVstepper_pins = [Pin(9, Pin.OUT), Pin(8, Pin.OUT), Pin(7, Pin.OUT), Pin(6, Pin.OUT)]

# Define the sequence of steps for the motor to take
step_sequence = [
    [1, 0, 0, 1],
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
]

def step(direction, steps, delay):
    # Use the global step_index variable so that it can be modified by this function
    global step_index
    # Loop through the specified number of steps in the specified direction
    for i in range(steps):                                                                                                                                               
        # Add the specified direction to the current step index to get the new step index
        step_index = (step_index + direction) % len(step_sequence)
        # Loop through each pin in the motor
        for pin_index in range(len(ROTstepper_pins)):
            # Get the value for this pin from the step sequence using the current step index
            pin_value = step_sequence[step_index][pin_index] 
            # Set the pin to this value
            ROTstepper_pins[pin_index].value(pin_value)
            MOVstepper_pins[pin_index].value(pin_value)
        # Delay for the specified amount of time before taking the next step
        utime.sleep(delay)
# Set the initial step index to 0
step_index = 0
# Take the specified number of steps in the anti-clockwise direction with a delay of 0.01 seconds between steps
MOTOR_FACTOR=2048 / 40 # - 0.6
print(f"Motor Factor: {MOTOR_FACTOR}")
for i in range(40):
    ##step(1, 1.0 * MOTOR_FACTOR, 0.020)
    step(1, 1.0 * MOTOR_FACTOR, 0.002)
# Take the specified number of steps in the clockwise direction with a delay of 0.01 seconds between steps
#step(-1, 1000, 0.001)
