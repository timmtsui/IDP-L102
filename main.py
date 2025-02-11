### Imports

from machine import Pin, PWM
from time import sleep
import heapq
from qrcode import scan


### Classes

class Motor:
    def __init__(self, pin1, pin2):
        self.m1Dir = Pin(pin1, Pin.OUT) 
        self.pwm1 = PWM(Pin(pin2)) 
        self.pwm1.freq(1000)
        self.pwm1.duty_u16(0)
        
    def off(self):
        self.pwm1.duty_u16(0)
        
    def forward(self, speed):
        self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
        self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1
        
    def reverse(self, speed):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535*speed/100))

    def speed(self, speed):
        if speed > 0:
            self.m1Dir.value(0) # Forward = 0
        else:
            self.m1Dir.value(1) # Reverse = 1
        
        self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1
        
    def turn(self, angle, confirmation=False): # motor_left.turn is the same as motor_right.turn
        motor_left.off()
        motor_right.off()
        if confirmation:
            angle = angle * 0.75
        if angle<0:
            # Left turn
            motor_left.speed(0)
            motor_right.speed(100)
            sleep(angle*-0.018)
            motor_left.off()
            motor_right.off()

            if confirmation:
                while S3.value() == 0:
                    motor_left.turn(-3)
                    print("Turning left incrementally")
            
        if angle>0:
            # Right turn
            motor_left.speed(100)
            motor_right.speed(0)
            sleep(angle*0.018)
            motor_left.off()
            motor_right.off()

            if confirmation:
                while S2.value() == 0:
                    motor_left.turn(3)
                    print("Turning right incrementally")


            
        
### Functions

def LineFollow(): 
    # Will need to implement ghosting prevention
    if S2.value() == 0 and S3.value() == 0:
        motor_left.speed(50)
        motor_right.speed(50)
        print("Move forward")
    elif S3.value() == 1:
        motor_left.speed(100)
        motor_right.speed(-100*0.75)
        print("Turn right")
        sleep(0.05)
    else:
        motor_left.speed(-100)
        motor_right.speed(100*0.75)
        print("Turn left")
        sleep(0.05)

def blindstraight(speed, time):
    motor_left.speed(speed)
    motor_right.speed(speed*0.75)
    senseall = [0, 0, 0 ,0]

    for i in range(20):
        sense = [S1.value(), S2.value(), S3.value(), S4.value()]
        for j in range(4):
            if sense[j] == 1:
                senseall[j] += 1
        sleep(time/20)

    motor_left.off()
    motor_right.off()

    for i in range(4):
        if senseall[i] >= 2: # more than 10% of the time
            senseall[i] = 1
        else:
            senseall[i] = 0
    
    return senseall

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_expected_sensors(approach_dir, junction_config):
    #  compass = ["N","E","S","W"]
    compass = [0,90,180,270]
    if sum(junction_config) == 3:  # T-junctions or crossroads
        nulljunc = None
        for i in range(0,4):
            if junction_config[i] == 0:
                nulljunc = compass[i]

        print("nulljunc",nulljunc)
        print("juncon",junction_config)
        print("approachdir",approach_dir)
        
        if nulljunc is not None:
            relative_orientation = (approach_dir - nulljunc) % 360
            print("relor",relative_orientation)
            if relative_orientation == 0:
                # Approach from front
                return [1, 1, 1, 1]
            elif relative_orientation == 90:
                # Right T
                return [0, 0, 1, 1]
            elif relative_orientation == 270:
                # Left T
                return [1, 1, 0, 0]
            

    if junction_config[0] != junction_config [2] and sum(junction_config) == 2:
        onejunc = None
        for i in range(0,4):
            print(type(approach_dir))
            if junction_config[i] == 1 and compass[i] != ( approach_dir + 180 ) % 360:
                onejunc = compass[i]
        if onejunc is not None:
            relative_orientation = (approach_dir - onejunc) % 360
            print("relor",relative_orientation)
            if relative_orientation == 90:
                # Right
                return [1, 1, 0, 0]
            elif relative_orientation == 270:
                # Left
                return [0, 0, 1, 1]

    return [0,0,0,0]  # Default straight-line behavior

def get_turn_direction(prev, current, next):
    if not next:
        return 'Straight'
    r, c = current
    nr, nc = next
    if nr > r:
        return 'Straight' if prev[0] < r else ('Right' if prev[1] < c else 'Left')
    if nr < r:
        return 'Straight' if prev[0] > r else ('Right' if prev[1] > c else 'Left')
    if nc > c:
        return 'Straight' if prev[1] < c else ('Right' if prev[0] > r else 'Left')
    if nc < c:
        return 'Straight' if prev[1] > c else ('Right' if prev[0] < r else 'Left')
    return 'Straight'

def astar(nav_grid, start, end):
    rows, cols = len(nav_grid), len(nav_grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            waypoints = []
            for i in range(len(path)):
                prev = path[i-1] if i > 0 else None
                curr = path[i]
                nxt = path[i+1] if i < len(path)-1 else None
                
                if prev:
                    direction = get_turn_direction(prev, curr, nxt)
                else:
                    direction = 'Straight'
                #["W","E","S","N"]
                approach_dir = 0 if prev and prev[0] > curr[0] else \
                               180 if prev and prev[0] < curr[0] else \
                               90 if prev and prev[1] < curr[1] else \
                               270 if prev and prev[1] > curr[1] else 0
                
                expected_sensors = get_expected_sensors(approach_dir, nav_grid[curr[0]][curr[1]])
                waypoints.append((curr, expected_sensors, direction))
            
            waypoints.pop(0)
            for i in waypoints:
                if i[1] == [0,0,0,0]:
                    waypoints.remove(i)
            
            return waypoints
    
        r, c = current
        neighbors = []
        directions = nav_grid[r][c]
        if directions[0] and r > 0:
            neighbors.append((r-1, c))
        if directions[1] and c < cols - 1:
            neighbors.append((r, c+1))
        if directions[2] and r < rows - 1:
            neighbors.append((r+1, c))
        if directions[3] and c > 0:
            neighbors.append((r, c-1))
        
        for neighbor in neighbors:
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return []



def dropoff():
    return

def get_button(button_status):
    count = 0
    if button.value() == 0:
        """while count < 20:
            count += button.value()"""
        return 0 # Returns toggled button status
    return 1


### Variables

addresses = {"A": (3,1), "B": (3,3), "C": (1,1), "D":(1,3)}

motor_left = Motor(7, 6)
motor_right = Motor(4, 5)
S1 = Pin(21, Pin.IN, Pin.PULL_DOWN)
S2 = Pin(20, Pin.IN, Pin.PULL_DOWN)
S3 = Pin(19, Pin.IN, Pin.PULL_DOWN)
S4 = Pin(18, Pin.IN, Pin.PULL_DOWN)
button = Pin(22, Pin.IN, Pin.PULL_DOWN)
# Set up PWM Pin for servo control
servo_pin = Pin(15)
servo = PWM(servo_pin)
# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
servo_horizontal = 4833
servo_carry = 3300
servo_highest = 2800
half_duty = int(max_duty/2)
#Set PWM frequency
frequency = 50

on = 0
current_pos = (5,2)
depot_1 = (5,4)
depot_2  = (5, 0)
start = (5, 2)
boxes_delivered = 0

# Nav_Grid is in [N, E, S, W]
Nav_Grid = [
    [[0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 1, 1], [0, 0, 1, 1]],
    [[1, 0, 1, 0], [0, 1, 0, 0], [1, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [0, 1, 0, 1], [1, 1, 0, 1], [0, 1, 1, 1], [1, 0, 1, 1]],
    [[1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [1, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 1, 1]],
    [[1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]]
]

def pickup():
    # LOWER FORKLIFT
    scanned = False
    servo.duty_u16(servo_horizontal)
    while scanned == False:
        scanned = scan()
    global destination
    destination = addresses[scanned]

    return

### Main loop
def navigate(start, end):
    waypoints = astar(Nav_Grid, start, end)
    
    
    while len(waypoints) > 0:
        # Line following when no junction detected
        sense = [S1.value(), S2.value(), S3.value(), S4.value()]
        while sense[0] == 0 and sense[3] == 0:
            sense = [S1.value(), S2.value(), S3.value(), S4.value()]
            print(sense)
            LineFollow()
            sleep(0.05)

        # Sees junction
        print(sense, waypoints[0][1])
        junctionsense = blindstraight(30, 0.5)
        if junctionsense == waypoints[0][1]: # Correct junction detected
            print("Correct junction detected")
        else:
            print("Incorrect junction detected")
        
        if True:
            if waypoints[0][2] == "Straight":
                blindstraight(50,1)
            elif waypoints[0][2] == "Right":
                motor_left.turn(90, True)
                sleep(1)
            elif waypoints[0][2] == "Left":
                motor_right.turn(-90, True)
                sleep(1)
            waypoints.pop(0)
            
            for i in range(10):
                LineFollow()
                sleep(0.1)

        sleep(0.1)

#handles movement and mechanism to pick up box
# also calls qr code scan and returns drop off coordinates
# should leave robot in N orientation away from junctions

    
while True:
    
    blindstraight(1, 1)
    navigate((5, 4), (0, 2))

    """
    while boxes_delivered < 8:
        destination = pickup()
        current_pos = depot_1
        navigate(current_pos, destination)
        dropoff()
        current_pos = destination
        if boxes_delivered <= 3:
            destination = depot_1
        else:
            destination  = depot_2
        navigate(current_pos, destination)
    """

    # Go back to start
    # navigate(current_pos, start)
    # break
