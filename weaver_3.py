import machine
import time
import math

# Define pins (adjust to your wiring)
ROT_PIN1 = machine.Pin(4, machine.Pin.OUT)
ROT_PIN2 = machine.Pin(3, machine.Pin.OUT)
ROT_PIN3 = machine.Pin(2, machine.Pin.OUT)
ROT_PIN4 = machine.Pin(1, machine.Pin.OUT)

INOUT_PIN1 = machine.Pin(9, machine.Pin.OUT)
INOUT_PIN2 = machine.Pin(8, machine.Pin.OUT)
INOUT_PIN3 = machine.Pin(7, machine.Pin.OUT)
INOUT_PIN4 = machine.Pin(6, machine.Pin.OUT)

# Stepper parameters
rot_total_steps = 512 * 6.25 #12800
inOut_total_steps = 4642
gearRatio = 100.0 / 16.0

# Buffer for theta-rho pairs
BUFFER_SIZE = 10
buffer = [[0.0, 0.0] for _ in range(BUFFER_SIZE)]
bufferCount = 0
batchComplete = False

# Current position
currentTheta = 0.0
currentRho = 0.0
isFirstCoordinates = True
maxSpeed = 550
maxAcceleration = 5000 # Not directly used in this simple implementation
subSteps = 1/16 # 1

# Stepper driver functions (emulating AccelStepper)
def set_stepper_speed(pins, speed):
    # This is a VERY basic speed control. A proper implementation would use timers/PWM.
    delay = abs(1000.0 / speed) if speed != 0 else 0
    return delay

def move_stepper(name, pins, steps):
    print("Moving",name,"stepper",steps, "steps")
    for _ in range(abs(steps)):
        if steps > 0:
            # Full step sequence (adjust if needed for half-stepping, etc.)
            for seq in [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1]]:
                for i, pin in enumerate(pins):
                    pin.value(seq[i])
                #time.sleep_us(int(set_stepper_speed(pins, maxSpeed))) # Crude speed control
                time.sleep(0.002)
        else: #Reverse direction
            for seq in [[0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1]]:
                for i, pin in enumerate(pins):
                    pin.value(seq[i])
                #time.sleep_us(int(set_stepper_speed(pins, maxSpeed))) # Crude speed control
                time.sleep(0.002)
    print("Stepper moved")

# Create "stepper" objects (just lists of pins for now)
rotStepper = [ROT_PIN1, ROT_PIN2, ROT_PIN3, ROT_PIN4]
inOutStepper = [INOUT_PIN1, INOUT_PIN2, INOUT_PIN3, INOUT_PIN4]

currentInOut = 0

def homing():
    print("HOMING")
    # Move inOutStepper inward for homing (crude homing, relies on hitting a limit)
    #while True:
    move_stepper("inout", inOutStepper, -100) #Move inwards, adjust steps if needed
    #    if machine.Pin(5, machine.Pin.IN, machine.Pin.PULL_UP).value() == 0: #Example limit switch on pin 5
    #        break
        
    for pin in inOutStepper:
        pin.value(0)
    for pin in rotStepper:
        pin.value(0)
    currentRho = 0.0
    currentTheta = 0.0
    print("HOMED")

def movePolar(theta, rho):
    rho = max(0.0, min(1.0, rho))  # Clamp rho to [0, 1]

    rotSteps = round(theta * (rot_total_steps / (2.0 * math.pi)))
    revolutions = theta / (2.0 * math.pi)
    offsetSteps = round(revolutions * (rot_total_steps / gearRatio))
    inOutSteps = round(rho * inOut_total_steps)
    inOutSteps -= offsetSteps

    move_stepper("rot", rotStepper, rotSteps)
    move_stepper("inout", inOutStepper, inOutSteps)

    currentTheta = theta
    currentRho = rho

def interpolatePath(startTheta, startRho, endTheta, endRho, subSteps):
    print("Theta from:",startTheta, "to",endTheta,"Rho from:",startRho, "to",endRho)
    distance = math.sqrt((endTheta - startTheta)**2 + (endRho - startRho)**2)
    numSteps = max(1, int(distance / subSteps))

    print("Steps: ", numSteps)
    t = 1 / numSteps
    interpolatedTheta = startTheta + t * (endTheta - startTheta)
    interpolatedRho = startRho + t * (endRho - startRho)
    for step in range(numSteps):
        print(" Step: ", step)
        #t = step / numSteps
        #interpolatedTheta = startTheta + t * (endTheta - startTheta)
        #interpolatedRho = startRho + t * (endRho - startRho)
        movePolar(interpolatedTheta, interpolatedRho)



test_input1 = """SET_SPEED 1
0.1,0.5;
360.0,0.0;
-360.0,-0.5;
"""

test_input2 = """1.25673,0.03250;
0.66059,0.03508;
0.62272,0.03571;
"""

test_input3 = """SET_SPEED 1
HOME
0,0;
1.5708,1;
3.14159,2;
4.71239,1;
6.28319,1;"""

test_input4 = """SET_SPEED 1
0,0;
6.28319,0;"""


input = test_input4

# Main loop (simplified serial handling)
while True:
    try:
        for line in input.split("\n"):
            if line == "HOME":
                homing()
            elif line == "RESET_THETA":
                isFirstCoordinates = True
                currentTheta = 0
                currentRho = 0
                print("THETA_RESET")
            elif line.startswith("SET_SPEED"):
                try:
                    speed = float(line.split(" ")[1])
                    if speed > 0:
                        maxSpeed = speed
                        print("SPEED_SET")
                    else:
                        print("INVALID_SPEED")
                except (IndexError, ValueError):
                    print("INVALID_COMMAND")
            elif line.endswith(";"):
                pairs = line.split(";")[:-1]
                bufferCount = len(pairs)
                for i, pair in enumerate(pairs):
                    theta, rho = map(float, pair.split(","))
                    buffer[bufferCount][0] = theta
                    buffer[bufferCount][1] = rho
                    bufferCount += 1
                batchComplete = True

        if batchComplete and bufferCount > 0:
            if isFirstCoordinates:
                #homing()
                isFirstCoordinates = False
            startTheta = currentTheta
            startRho = currentRho
            print ("Buffer Count: ", bufferCount)
            for i in range(bufferCount):
                print("Interpolating buffer item ", i)
                interpolatePath(startTheta, startRho, buffer[i][0], buffer[i][1], subSteps)
                startTheta = buffer[i][0]
                startRho = buffer[i][1]
            batchComplete = False
            bufferCount = 0
            print("R")
    except EOFError:
        break
    except ValueError:
        print("IGNORED")
    break