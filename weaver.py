import machine
import time
import math

# Define pins (adjust for your specific board)
ROT_PIN1 = machine.Pin(4, machine.Pin.OUT)
ROT_PIN2 = machine.Pin(3, machine.Pin.OUT)
ROT_PIN3 = machine.Pin(2, machine.Pin.OUT)
ROT_PIN4 = machine.Pin(1, machine.Pin.OUT)

INOUT_PIN1 = machine.Pin(9, machine.Pin.OUT)
INOUT_PIN2 = machine.Pin(8, machine.Pin.OUT)
INOUT_PIN3 = machine.Pin(7, machine.Pin.OUT)
INOUT_PIN4 = machine.Pin(6, machine.Pin.OUT)

rot_total_steps = 12800
inOut_total_steps = 4642
gearRatio = 100.0 / 16.0

BUFFER_SIZE = 10
buffer = [[0.0, 0.0] for _ in range(BUFFER_SIZE)]
bufferCount = 0
batchComplete = False

# Stepper driver control (example using basic stepping)
def set_stepper(pins, step_sequence):
    #print("Pins: " + str(pins) + " Step Sequence: " + str(step_sequence))
    for i in range(4):
        pins[i].value(step_sequence[i])

def step(pins, steps, direction): #direction 1 is forward, -1 is backward
    step_sequence = [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1]]
    if direction == -1:
        step_sequence.reverse()
    for _ in range(steps):
        for seq in step_sequence:
            set_stepper(pins, seq)
            time.sleep_us(2000) # Adjust delay for speed

rot_pins = [ROT_PIN1, ROT_PIN3, ROT_PIN2, ROT_PIN4]
inout_pins = [INOUT_PIN1, INOUT_PIN3, INOUT_PIN2, INOUT_PIN4]

# Variables˜
currentTheta = 0.0
currentRho = 0.0
isFirstCoordinates = True
maxSpeed = 550
maxAcceleration = 5000 #Not implemented
interpolationResolution = 1

def modulus(x, y):
    return x % y

def homing():
    print("HOMING")
    step(inout_pins,int(inOut_total_steps * 1.1), -1)
    currentRho = 0.0
    currentTheta = 0.0
    print("HOMED")

def movePolar(theta, rho):
    print("Moving to theta: " + str(theta) + " rho: " + str(rho))
    
    global currentTheta, currentRho
    if rho < 0.0:
        rho = 0.0
    elif rho > 1.0:
        rho = 1.0

    rotSteps = int(theta * (rot_total_steps / (2.0 * math.pi)))
    revolutions = theta / (2.0 * math.pi)
    offsetSteps = int(revolutions * (rot_total_steps / gearRatio))
    inOutSteps = int(rho * inOut_total_steps)

    inOutSteps -= offsetSteps

    print("Rotating " + str(rotSteps) + " steps")
    step(rot_pins, abs(rotSteps), 1 if rotSteps >= 0 else -1)
    print("Moving " + str(inOutSteps) + " steps")
    step(inout_pins,abs(inOutSteps), 1 if inOutSteps >= 0 else -1)

    currentTheta = theta
    currentRho = rho

def interpolatePath(startTheta, startRho, endTheta, endRho, stepSize):
    distance = math.sqrt((endTheta - startTheta)**2 + (endRho - startRho)**2)
    numSteps = max(1, int(distance / stepSize))
    print("Interpolating path distance: " + str(distance) + " numSteps: " + str(numSteps))

    for step_num in range(numSteps + 1):
        t = float(step_num) / numSteps
        interpolatedTheta = startTheta + t * (endTheta - startTheta)
        interpolatedRho = startRho + t * (endRho - startRho)
        movePolar(interpolatedTheta, interpolatedRho)

# Main loop (using basic input for demonstration)
while True:
    print("Hi")
    homing()
    try:
        #input_str = input("Enter theta,rho pairs (e.g., 0.1,0.5;0.2,0.7): ")
        input_str = "0.1,0.5" 
        print("input_str: " + input_str)
        pairs = input_str.split(';')
        for pair in pairs:
            if pair:
                print(pair)
                theta_str, rho_str = pair.split(',')
                theta = float(theta_str)
                rho = float(rho_str)
                interpolatePath(currentTheta, currentRho, theta, rho, interpolationResolution)
    except (ValueError, IndexError):
        print("Invalid input")
    break
