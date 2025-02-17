### Imports

from machine import Pin, PWM
from time import sleep
import heapq
from qrcode import scan
from tof_test import TOF


MOTOR_RIGHT_RATIO = 0.65

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

    def speed(self, vel):
        if vel > 0:
            self.m1Dir.value(0) # Forward = 0
        else:
            self.m1Dir.value(1) # Reverse = 1
        
        self.pwm1.duty_u16(int(65535*vel/100)) # speed range 0-100 motor 1
        
    def turn(self, angle, confirmation=False, both=0): # motor_left.turn is the same as motor_right.turn
        motor_left.off()
        motor_right.off()
        if both == -1:
            angle = angle/2
        if confirmation:
            angle = angle * 0.75
        if angle<0:
            # Left turn
            motor_left.speed(1*both)
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
            motor_right.speed(both*1)
            sleep(angle*0.018)
            motor_left.off()
            motor_right.off()

            if confirmation:
                while S2.value() == 0:
                    motor_left.turn(3)
                    print("Turning right incrementally")


            
        
### Functions

def LineFollow(ratio=1.0): 
    # Will need to implement ghosting prevention
    if S2.value() == 0 and S3.value() == 0:
        motor_left.speed(100*ratio)
        motor_right.speed(100*ratio)
        #print("Move forward")
    elif S3.value() == 1:
        motor_left.speed(50)
        motor_right.speed(-50*MOTOR_RIGHT_RATIO)
        #print("Turn right")
        sleep(0.05)
    else:
        motor_left.speed(-50)
        motor_right.speed(50*MOTOR_RIGHT_RATIO)
        #print("Turn left")
        sleep(0.05)

def blindstraight(speed, time):
    motor_left.speed(speed)
    motor_right.speed(speed*MOTOR_RIGHT_RATIO*1.3)
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
                waypoints.append([curr, expected_sensors, direction])
            
            waypoints.pop(0)
            for i in waypoints:
                if i[1] == [0,0,0,0] and i[0] not in special_addresses.values():
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
    blindstraight(80,0.5)
    for i in range(20):
        LineFollow()
    servo.duty_u16(servo_horizontal)
    blindstraight(-10, 1.5)
    servo.duty_u16(servo_carry)
    return

def ModLineFollow():
    if S3.value() == 1 and S1.value() == 1:
        motor_left.speed(50)
        motor_right.speed(-50*MOTOR_RIGHT_RATIO)
        #print("Turn right")
        sleep(0.05)
    elif S2.value() == 1 and S4.value() == 1:
        motor_left.speed(-50)
        motor_right.speed(50*MOTOR_RIGHT_RATIO)
    else:
        motor_left.speed(50)
        motor_right.speed(50)


### Variables

# Changed A from (3,1) to (3,3)
special_addresses = {"A": (3,3), "B": (3,3), "C": (1,1), "D":(1,3), "D1": (5, 4), "D2": (5, 0)}
dropoff_addresses = {"A": (3,3), "B": (3,3), "C": (1,1), "D":(1,3)}
pickup_addresses = {"D1": (5, 4), "D2": (5, 0)}
tight_addresses  ={"Bj": (2,3), "Cj": (1,2), "Dj": (0,3)}

motor_left = Motor(7, 6)
motor_right = Motor(4, 5)
S1 = Pin(20, Pin.IN, Pin.PULL_DOWN)
S2 = Pin(21, Pin.IN, Pin.PULL_DOWN)
S3 = Pin(22, Pin.IN, Pin.PULL_DOWN)
S4 = Pin(26, Pin.IN, Pin.PULL_DOWN)
button = Pin(19, Pin.IN, Pin.PULL_DOWN)
# Set up PWM Pin for servo control
servo_pin = Pin(13)
servo = PWM(servo_pin)
led_pin = Pin(12, Pin.OUT)
# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
servo_horizontal = 4733
servo_carry = 3300
servo_highest = 2800
half_duty = int(max_duty/2)
#Set PWM frequency
frequency = 50
servo.freq(frequency)

on = 0
current_pos = (5,2)
depot_1 = (5,4)
depot_2  = (5, 0)
start = (5, 2)
boxes_delivered = 0
destination = (0,0)

# Nav_Grid is in [N, E, S, W]
"""Nav_Grid = [
    [[0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 1, 1], [0, 0, 1, 1]],
    [[1, 0, 1, 0], [0, 1, 0, 0], [1, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [0, 1, 0, 1], [1, 1, 0, 1], [0, 1, 1, 1], [1, 0, 1, 1]],
    [[1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [1, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 1, 1]],
    [[1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]]
]"""
#Modified grid which stops the robot from turning right out of the box
Nav_Grid = [
    [[0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 1, 1], [0, 0, 1, 1]],
    [[1, 0, 1, 0], [0, 1, 0, 0], [1, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [0, 1, 0, 1], [1, 1, 0, 1], [0, 1, 1, 1], [1, 0, 1, 1]],
    [[1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [1, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]]
]

def pickupscan(depot_num=1):
    
    blindstraight(80,0.4)
    motor_left.off()
    motor_right.off()
    servo.duty_u16(servo_horizontal)
    dist = TOF()
    avg_dist = 1000
    print(dist)
    sleep(1)
    while avg_dist > 200:
        avg_dist = avg_dist*0.3 + dist*0.7
        dist = TOF()
        for i in range(20):
            LineFollow(0.5)
    for i in range(20):
        LineFollow(0.3)
    
    if depot_num == 1:
        if astar(Nav_Grid, current_pos, depot_1)[0][2] != "Straight":
            blindstraight(-40,0.01)
    
    # LOWER FORKLIFT
    scanned = None
    servo.duty_u16(servo_horizontal)
    sleep(1)
    count = 0
    while count < 300:
        LineFollow(0.5)
        scanned = scan()
        if scanned != None:
            addr = dropoff_addresses[scanned]
            print("Sucessfully scanned box: ", addr)
            return addr
        if count == 300:
            #blindstraight(-30,0.5)
            pass
        count +=1
        print("scanning attempt: ", count)
    print("Scanning failed")
    return (1, 1) #Tries to scan 250 times and if not sucessful then just delivers box to nearest depot

def pickup():
    motor_right.off()
    motor_left.off()
    sleep(0.5)
    dist = TOF()
    avg_dist = 100
    print(dist)
    while avg_dist > 20:
        avg_dist = avg_dist*0.3 + dist*0.7
        dist = TOF()
        print(dist)
        if S1.value() == 1 or S4.value() == 1:
            blindstraight(50,0.15)
        else:
            LineFollow(0.5)
    
    motor_left.off()
    motor_right.off()
    servo.duty_u16(servo_carry)

    sleep(1)
    return

def navigate(start, end, reverse_first=True,popfirst=False):
    waypoints = astar(Nav_Grid, start, end)
    last_waypoint = waypoints[-1]
    if popfirst:
        waypoints.pop(0)
    print(waypoints)
    if reverse_first:
        waypoints[0][1].reverse()
        if waypoints[0][2] == "Right":
            waypoints[0][2] = "Left"
        elif waypoints[0][2] == "Left":
            waypoints[0][2] = "Right"
    print(waypoints)
    
    
    while len(waypoints) > 0:
        # Line following when no junction detected
        if any([True for k,v in pickup_addresses.items() if v == waypoints[0][0]]):
            break
        sense = [S1.value(), S2.value(), S3.value(), S4.value()]
        if reverse_first:
            sense[0] = 1
        while sense[0] == 0 and sense[3] == 0:
            sense = [S1.value(), S2.value(), S3.value(), S4.value()]
            print(sense)
            LineFollow()
            sleep(0.05)

        # Sees junction
        
        both = 0
        if any([True for k,v in tight_addresses.items() if v == waypoints[0][0]]) and len(waypoints) > 2:
            print("Tight turn")
            both = -1
        else:
            print("Not tight turn")
            both = 0
        
        print(sense, waypoints[0][1])
        junctionsense = blindstraight(70, 0.2)
        if junctionsense == waypoints[0][1]: # Correct junction detected
            print("Correct junction detected")
        else:
            print("Incorrect junction detected")
        
        if True:
            if waypoints[0][2] == "Straight":
                while S4.value() != 0 or S1.value() != 0:
                     # Modified version of linefollow only using one side
                        #print("Move forward")
                    ModLineFollow()


            elif waypoints[0][2] == "Right":
                motor_left.turn(90, True,both)
                sleep(0.1)
            elif waypoints[0][2] == "Left":
                motor_right.turn(-90, True,both)
                sleep(0.1)
        
            reverse_first = False

            # If the destination is a delivery address, then it doesnt pop the last waypoint
            print(" nav waypoint", waypoints)
            if any([True for k,v in special_addresses.items() if v == waypoints[0][0]]):
                break
            else:
                waypoints.pop(0)
            
            for i in range(5):
                LineFollow()
                sleep(0.05)

        sleep(0.1)


    print(waypoints)
    try:
        if any([True for k,v in dropoff_addresses.items() if v == waypoints[0][0]]):
            print("Arrived at a destination, initialising dropoff...")  
            dropoff()
    except:
        pass
    
    motor_left.off()
    motor_right.off()
    print("Navigation from ",start, "to", end, "completed")
        
#handles movement and mechanism to pick up box
# also calls qr code scan and returns drop off coordinates

"""while True:    
    led_pin.value(1)
    sleep(10)"""

count = 0
while count < 20:
    if button.value() == 1:
        count += 1
        sleep(0.01)
    elif button.value() == 0:
        count = max(count-1, 0)

print("Button Pressed")

# Navigate out of the box

current_pos = start
servo.duty_u16(servo_highest)
blindstraight(80, 1)
led_pin.value(1)
navigate((5, 2), depot_1, False)
print("Initial nav finished")

while boxes_delivered < 4:
    # At depot
    destination = pickupscan()
    sleep(0.5)
    pickup()
    motor_left.off()
    motor_right.off()


    blindstraight(-5,1+boxes_delivered/3)
    motor_left.turn(-180,True,-1)
    Nav_Grid[4][4][3] = 0
    navigate(depot_1, destination, False,True)
    boxes_delivered += 1
    Nav_Grid[4][4][3] = 1


    current_pos = destination
    
    if boxes_delivered <= 3:
        destination = depot_1
    else:
        destination  = start

    navigate(current_pos, destination)

motor_left.turn(-20)
#Final movement into depot
blindstraight(80,2)

led_pin.value(0)

motor_left.off()
motor_right.off()


