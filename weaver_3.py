import machine
import time
import math
import array

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
rot_total_steps = 512 * 6.25 # 100 (driven gear)/ 16 (teeth on gear) #12800
inOut_total_steps = 4642 #4600 
# A: 3200 steps = 100 (driven gear)/ 16 (drive gear) * 512 (steps pre revolution)
# B: 0.397 mm/tooth = 13.5 (length of linear) / 34 (teeth on linear) 
# C: 0.000123 mm/step  B / A
# D: 2048 # ?? Supposed to be steps per revolution which is 512
# E: 13.5mm / 2048 steps = 0.0066 mm/step | 13.5mm / 512 steps = 0.0264 mm/step
# F: C / E | 0.000123 mm/step / 0.0264 mm/step = 0.0046
compensation_ratio = 0.01888 # # 100 / 16 * 512 ... so why does 0.01888 work better ?
gearRatio = 100.0 / 16.0

# Buffer for theta-rho pairs
BUFFER_SIZE = 32198  # Example size
# Efficient approach using array.array (recommended)
large_array = array.array('f', [0.0] * (BUFFER_SIZE * 2)) # 'f' is for single-precision float (4 bytes)

# Reshape the 1D array into a 2D structure (without extra memory overhead)
def reshape_array(arr, rows, cols):
    if len(arr) != rows * cols:
        raise ValueError("Array size does not match rows x cols")
    for i in range(rows):
        yield arr[i*cols:(i+1)*cols]

two_d_array = list(reshape_array(large_array, BUFFER_SIZE, 2))


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
    global rotSeqIndex, inOutSeqIndex
    print("Moving", name, "stepper", steps, "steps")

    seqIndex = rotSeqIndex if name == "rot" else inOutSeqIndex
    seqAmt = 1 if steps > 0 else -1
    sequence = [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1]]

    for _ in range(abs(steps)): #Correct loop
        for i, pin in enumerate(pins):
            pin.value(sequence[seqIndex % 4][i])
        time.sleep(0.002)  # Adjust delay as needed
        seqIndex += seqAmt
    if name == "rot":
        rotSeqIndex = seqIndex
    else:
        inOutSeqIndex = seqIndex
    print("Stepper moved")

# Create "stepper" objects (just lists of pins for now)
rotStepper = [ROT_PIN1, ROT_PIN2, ROT_PIN3, ROT_PIN4]
inOutStepper = [INOUT_PIN1, INOUT_PIN2, INOUT_PIN3, INOUT_PIN4]

currentInOut = 0

def homing():
    print("HOMING")
    # Move inOutStepper inward for homing (crude homing, relies on hitting a limit)
    #while True:
    move_stepper("inout", inOutStepper, -inOut_total_steps) #Move inwards, adjust steps if needed
    #    if machine.Pin(5, machine.Pin.IN, machine.Pin.PULL_UP).value() == 0: #Example limit switch on pin 5
    #        break
        
    for pin in inOutStepper:
        pin.value(0)
    for pin in rotStepper:
        pin.value(0)
    currentRho = 0.0
    currentTheta = 0.0
    print("HOMED")

current_rot_step = 0  # Initialize current rotational step
current_inout_step = 0  # Initialize current in-out step

def movePolar(theta, rho):
    global current_rot_step, current_inout_step #Make global so they can be changed

    rho = max(0.0, min(1.0, rho))

    target_rot_step = round(theta * (rot_total_steps / (2.0 * math.pi)))
    target_inout_step = round(rho * inOut_total_steps)
    offsetDifference = (target_rot_step - current_rot_step)
    print("Offset difference: ", offsetDifference)
    offsetSteps = round(offsetDifference * compensation_ratio)
    print("Offset Steps: ", offsetSteps)
    offsetSteps = 5 * offsetSteps
    print("Hacked Offset Steps: ", offsetSteps)
    #offsetSteps = round(target_rot_step * compensation_ratio)

    rot_step_diff = target_rot_step - current_rot_step
    inout_step_diff = (target_inout_step - current_inout_step) - offsetSteps

    print(f"Target Rot Step: {target_rot_step}, Current Rot Step: {current_rot_step}, Difference: {rot_step_diff}")
    print(f"Target InOut Step: {target_inout_step}, Current InOut Step: {current_inout_step}, Offset: {offsetSteps}, Difference: {inout_step_diff}")

    move_synchronized(rotStepper, rot_step_diff, inOutStepper, inout_step_diff)

    current_rot_step = target_rot_step  # Update current step positions
    current_inout_step = target_inout_step

    currentTheta = theta
    currentRho = rho


rotSeqIndex = 0
inOutSeqIndex = 0

def move_synchronized(rot_pins, rot_steps, inout_pins, inout_steps):
    """Moves both steppers simultaneously using integer math."""

    rot_steps = int(rot_steps)
    inout_steps = int(inout_steps)
    
    rot_direction = 1 if rot_steps > 0 else -1
    inout_direction = 1 if inout_steps > 0 else -1

    rot_steps_remaining = abs(rot_steps)
    inout_steps_remaining = abs(inout_steps)

    rot_sequence = [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1]]
    inout_sequence = [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1]]

    if (rot_direction == -1):
        rot_sequence.reverse()
    if (inout_direction == -1):
        inout_sequence.reverse()

    while rot_steps_remaining > 0 or inout_steps_remaining > 0:
        if rot_steps_remaining > 0:
            for seq in rot_sequence:
                for j, pin in enumerate(rot_pins):
                    pin.value(seq[j])
                #print("ROT Pin: ", "[ ", seq[0],", ", seq[1],", ", seq[2],", ", seq[3],"]")    
                time.sleep(0.002)    
            rot_steps_remaining -= 1
        if inout_steps_remaining > 0:
            for seq in inout_sequence:
                for j, pin in enumerate(inout_pins):
                    pin.value(seq[j])
                #print("INOUT Pin: ", "[ ", seq[0],", ", seq[1],", ", seq[2],", ", seq[3],"]")
                time.sleep(0.002)
            inout_steps_remaining -= 1
    


def interpolatePath(startTheta, startRho, endTheta, endRho, subSteps):
    print("Theta from:", startTheta, "to", endTheta, "Rho from:", startRho, "to", endRho)

    distance = math.sqrt((endTheta - startTheta)**2 + (endRho - startRho)**2)
    print("Distance:", distance)

    numSteps = 1 if distance <= 1 else int(distance / subSteps) # More readable
    print("Steps:", numSteps)

    print("Start Theta:", startTheta, "End Theta:", endTheta)

    for step in range(numSteps + 1): # Removed unnecessary abs()
        t = step / numSteps
        interpolatedTheta = startTheta + t * (endTheta - startTheta)
        interpolatedRho = startRho + t * (endRho - startRho)
        print(" Step:", step, "t (", step, "/", numSteps, "):", t, "Interpolated Theta:", interpolatedTheta, "Interpolated Rho:", interpolatedRho)
        movePolar(interpolatedTheta, interpolatedRho)


def parseFile(filename):
    global two_d_array, bufferCount
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()  # Remove leading/trailing whitespace
                #print("Line: ", line)
                if line and not line.startswith('#'):  # Ignore empty lines and comments
                    elements = line.split(" ")  # Or process the line further here if needed
                    two_d_array[bufferCount][0] = float(elements[0])
                    two_d_array[bufferCount][1] = float(elements[1])
                    bufferCount += 1
            return
    except OSError: #Catch file not found or other OS related errors
        print(f"Error: File '{filename}' not found or could not be opened.")
        return None
    except Exception as e: #Catch other exceptions. Useful for debugging
        print(f"An error occurred: {e}")
        return None

def readInput(input):
    global two_d_array, bufferCount
    for line in input.split("\n"):
        if line.endswith(";"):
            pairs = line.split(";")[:-1]
            for i, pair in enumerate(pairs):
                print("Processing pair ", i, ":", pair)
                theta, rho = map(float, pair.split(","))
                two_d_array[bufferCount][0] = theta
                two_d_array[bufferCount][1] = rho
                bufferCount += 1

test_input2 = """1.25673,0.03250;
0.66059,0.03508;
0.62272,0.03571;
"""

test_input3 = """SET_SPEED 1
HOME
0,0;
1.5708,1;
3.14159,0;
4.71239,1;
6.28319,0;"""

test_input4 = """SET_SPEED 1
0,0;
6.28319,0;"""

test_input5 = """SET_SPEED 1
0,0;
1.5708,0;
0,0;
1.5708,0;
0,0;"""

test_input4 = """SET_SPEED 1
0,0;
6.28319,0;
12.5664,0;
18.8495,0;
"""

test_input5 = """
1.5708,0;
3.14159,0;
4.71239,0;
6.28319,0;"""

input = test_input4

filename = "patterns/03 pnuttrellis (E) (N N).thr"

print("Parsing")
#parseFile(filename)
readInput(input)
print("Parsed")
print("Executing")
batchComplete = True

while True:
    try:
        if batchComplete and bufferCount > 0:
            if isFirstCoordinates:
                maxSpeed = 1
                homing()
                isFirstCoordinates = False
            startTheta = currentTheta
            startRho = currentRho
            print ("Buffer Count: ", bufferCount)
            for index in range(bufferCount):
                row = two_d_array[index]
                print("Interpolating buffer item ", row)
                interpolatePath(startTheta, startRho, row[0], row[1], subSteps)
                startTheta = row[0]
                startRho = row[1]
                #time.sleep(10)
                print("Row complete")
            batchComplete = False
            bufferCount = 0
            print("R")
    except EOFError:
        break
    except ValueError:
        print("IGNORED")
    break