import machine
import time

# Define the pins connected to the motor coils (adjust to your wiring)
coil_pins = [
    machine.Pin(4, machine.Pin.OUT),  # Coil 1
    machine.Pin(3, machine.Pin.OUT),  # Coil 2
    machine.Pin(2, machine.Pin.OUT), # Coil 3
    machine.Pin(1, machine.Pin.OUT)  # Coil 4
]

# Step sequence
step_sequence = [
    [1, 0, 0, 1],
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1]
]

def drive_stepper(direction, steps, delay):
    """Drives the stepper motor a given number of steps."""
    if direction == -1:
        step_sequence.reverse()
    for _ in range(steps):
        print("Step: ", _)
        for step in step_sequence:
            for i, pin in enumerate(coil_pins):
                pin.value(step[i])
            time.sleep_ms(delay) # Adjust delay for speed

steps = 512 * 6.25
# Example usage:
#while True:
drive_stepper(1, steps, 2)  # One full rotation (512 steps), delay of 2ms
time.sleep_ms(500)       # Pause for 0.5 seconds
drive_stepper(-1, steps, 2) # One full rotation in reverse
time.sleep_ms(500)

for pin in coil_pins:
    pin.value(0) #Turn all coils off on exit