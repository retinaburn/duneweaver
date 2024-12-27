import machine
import time
import math

# Define pins (adjust for your board)
ROT_PIN1 = machine.Pin(4, machine.Pin.OUT)
ROT_PIN2 = machine.Pin(3, machine.Pin.OUT)
ROT_PIN3 = machine.Pin(2, machine.Pin.OUT)
ROT_PIN4 = machine.Pin(1, machine.Pin.OUT)

INOUT_PIN1 = machine.Pin(9, machine.Pin.OUT)
INOUT_PIN2 = machine.Pin(8, machine.Pin.OUT)
INOUT_PIN3 = machine.Pin(7, machine.Pin.OUT)
INOUT_PIN4 = machine.Pin(6, machine.Pin.OUT)

# Stepper parameters
rot_total_steps = 12800 #/1000
inOut_total_steps = 4642
gearRatio = 100.0 #/ 16.0

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
maxAcceleration = 5000
subSteps = 1

# Stepper driver (using a simple full-step driver emulation)
class Stepper:
    def __init__(self, name, pin1, pin2, pin3, pin4):
        self.name = name
        self.pins = [pin1, pin2, pin3, pin4]
        self.current_step = 0
        self.current_position = 0
        self.speed = 0
        self.max_speed = 0
        self.acceleration = 0
        self.enabled = False

    def enable_outputs(self):
        self.enabled = True

    def disable_outputs(self):
        self.enabled = False
        for pin in self.pins:
            pin.value(0)

    def set_speed(self, speed):
        print(self.name, " speed:", speed)
        self.speed = speed

    def set_max_speed(self, max_speed):
        print(self.name, " max speed:", max_speed)
        self.max_speed = max_speed

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration

    def set_current_position(self, position):
        self.current_position = position

    def current_position(self):
        return self.current_position

    def run_speed(self):
        #print("Self: ", self.name," speed:", self.speed, " current_position:", self.current_position)
        if not self.enabled:
            return
        if self.speed > 0:
            self.current_step = (self.current_step + 1) % 4
            self.current_position += 1
        elif self.speed < 0:
            self.current_step = (self.current_step - 1) % 4
            self.current_position -= 1
        else:
            return
        self.update_pins()
        #print("Sleeping for: ", int(1000000 / abs(self.speed)))
        #time.sleep_us(int(1000000 / abs(self.speed)))  # Crude delay for speed control
        #time.sleep(0.002)
        time.sleep(0.002)

    def run_speed_to_position(self):

        # false negative, true positive
        speed_direction = True
        last_speed_direction = False
        speed_printed = False
        #print("Current Position: ", self.current_position, "Target Position: ", self.target_position)
        while self.current_position != self.target_position:
            if speed_direction != last_speed_direction:
                speed_printed = False
            if not speed_printed:
                    speed_printed = True
                    #last_speed_direction = speed_direction
                    #print(self.name, " Current Position: ", self.current_position, "Target Position: ", self.target_position, "Speed: ", self.speed)
            if (self.current_position + self.speed) > self.target_position:
                #print("Reached target position")
                self.current_position = self.target_position
                continue
            if self.current_position < self.target_position:     
                self.speed = self.max_speed
            else:
                self.speed = -self.max_speed
            self.run_speed()

    def move_to(self, target_position):
        self.target_position = target_position
        #print("From position: ", self.current_position, "To position: ", self.target_position)
        self.run_speed_to_position()

    def update_pins(self):
        sequence = [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1]]
        for i in range(4):
            self.pins[i].value(sequence[self.current_step][i])


# Create stepper objects
rotStepper = Stepper("ROT", ROT_PIN1, ROT_PIN2, ROT_PIN3, ROT_PIN4)
inOutStepper = Stepper("INOUT", INOUT_PIN1, INOUT_PIN2, INOUT_PIN3, INOUT_PIN4)

# UART setup
uart = machine.UART(0, 115200)

def homing():
    print("HOMING")
    inOutStepper.enable_outputs()
    inOutStepper.set_speed(-maxSpeed)
    currentInOut = inOutStepper.current_position
    while True:
        inOutStepper.run_speed()
        if inOutStepper.current_position <= currentInOut - int(inOut_total_steps * 1.1):
            break
    inOutStepper.set_current_position(0)
    rotStepper.set_current_position(0)
    currentTheta = 0.0
    currentRho = 0.0
    inOutStepper.disable_outputs()
    print("HOMED")

def movePolar(theta, rho):
    print("Moving to theta: " + str(theta) + " rho: " + str(rho))
    rho = max(0.0, min(1.0, rho))

    rotSteps = round(theta * (rot_total_steps / (2.0 * math.pi)))
    revolutions = theta / (2.0 * math.pi)
    offsetSteps = round(revolutions * (rot_total_steps / gearRatio))

    inOutSteps = round(rho * inOut_total_steps)
    inOutSteps -= offsetSteps

    move_to(rotSteps, inOutSteps)

    currentTheta = theta
    currentRho = rho
    print("Moved")

def move_to(rotSteps, inOutSteps):
    print("Moving with rotSteps: " + str(rotSteps) + " inOutSteps: " + str(inOutSteps))
    #print("Rotating " + str(rotSteps) + " steps")
    rotStepper.move_to(rotSteps)
    #print("Moving " + str(inOutSteps) + " steps")
    inOutStepper.move_to(inOutSteps)

def interpolatePath(startTheta, startRho, endTheta, endRho, subSteps):
    print("Interpolating path", startTheta, startRho, endTheta, endRho, subSteps)
    distance = math.sqrt((endTheta - startTheta)**2 + (endRho - startRho)**2)
    numSteps = max(1, int(distance / subSteps))

    print("Interpolating path distance: " + str(distance) + " numSteps: " + str(numSteps))
    for step in range(numSteps + 1):
        t = step / numSteps
        print("Step: ", step, "T: ", t, "NumSteps: ", numSteps)
        interpolatedTheta = startTheta + t * (endTheta - startTheta)
        interpolatedRho = startRho + t * (endRho - startRho)
        movePolar(interpolatedTheta, interpolatedRho)

def process_serial_input(input):
    global bufferCount, batchComplete, currentTheta, currentRho, isFirstCoordinates
    #if uart.any():
        #input_str = uart.readline().decode('utf-8').strip()
    for input_str in input.split("\n"):
        print ("Input: ", input_str)
        if input_str.startswith("SET_SPEED"):
            try:
                speed = float(input_str.split()[1])
                if speed > 0:
                    rotStepper.set_max_speed(speed)
                    inOutStepper.set_max_speed(speed)
                    print("SPEED_SET")
                else:
                    print("INVALID_SPEED")
            except (ValueError, IndexError):
                print("INVALID_COMMAND")
        elif input_str == "HOME":
            homing()
        elif input_str == "RESET_THETA":
            isFirstCoordinates = True
            currentTheta = 0
            currentRho = 0
            print("THETA_RESET")
        elif input_str.endswith(";"):
            pairs = input_str.split(';')[:-1]
            #bufferCount = 0
            for pair_str in pairs:
                try:
                    theta, rho = map(float, pair_str.split(','))
                    buffer[bufferCount][0] = theta
                    buffer[bufferCount][1] = rho
                    bufferCount += 1
                    if bufferCount >= BUFFER_SIZE:
                        break
                except ValueError:
                    print("INVALID_PAIR")

            batchComplete = True

        else:
            print("IGNORED")
        

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
3.14159,0;
4.71239,1;
6.28319,0;"""

test_input4 = """SET_SPEED 1
0,0;
6.28319,0;"""

while True:
    process_serial_input(test_input4)
    print("Buffer Count: ", bufferCount, "Batch Complete: ", batchComplete)
    if batchComplete and bufferCount > 0:
        rotStepper.enable_outputs()
        inOutStepper.enable_outputs()
        startTheta = currentTheta
        startRho = currentRho

        if isFirstCoordinates:
            #homing()
            isFirstCoordinates = False

        for i in range(bufferCount):
            interpolatePath(startTheta, startRho, buffer[i][0], buffer[i][1], subSteps)
            startTheta = buffer[i][0]
            startRho = buffer[i][1]
        print("Complete")
        rotStepper.disable_outputs()
        inOutStepper.disable_outputs()
        batchComplete = False
        bufferCount = 0
        print("R")
    break