### Imports

from machine import Pin, PWM
from time import sleep
import heapq



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
        
    def turn(self, angle): # motor_left.turn is the same as motor_right.turn
        motor_left.off()
        motor_right.off()
        if angle<0:
            motor_left.reverse(100)
            motor_right.forward(100)
            sleep(angle*-0.009)
            motor_left.off()
            motor_right.off()
            
        if angle>0:
            motor_left.forward(100)
            motor_right.reverse(100)
            sleep(angle*0.009)
            motor_left.off()
            motor_right.off()
            
        
### Functions

def LineFollow(): 
    # Will need to implement ghosting prevention
    if S2.value() == 0 and S3.value() == 0:
        motor_left.forward(50)
        motor_right.forward(50)
        print("Move forward")
    elif S3.value() == 1:
        motor_left.forward(50)
        motor_right.forward(0)
        print("Turn right")
    else:
        motor_left.forward(0)
        motor_right.forward(50)
        print("Turn left")

def move(speed, time):
    motor_left.forward(speed)
    motor_right.forward(speed)
    sleep(time)
    motor_left.off()
    motor_right.off()

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_expected_sensors(approach_dir, junction_config):
    if sum(junction_config) == 3:  # T-junctions or crossroads
        # compass = ["N","E","S","W"]
        compass = [0,90,180,270]
        nulljunc = None
        for i in range(0,3):
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
        pass
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
                               270 if prev and prev[1] > curr[1] else ''
                
                expected_sensors = get_expected_sensors(approach_dir, nav_grid[curr[0]][curr[1]])
                waypoints.append((curr, expected_sensors, direction))
            
            for i in range(len(waypoints)):
                if waypoints[i][1] == [0,0,0,0]:
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


### Variables

motor_left = Motor(7, 6)
motor_right = Motor(4, 5)
S1 = Pin(21, Pin.IN, Pin.PULL_DOWN)
S2 = Pin(20, Pin.IN, Pin.PULL_DOWN)
S3 = Pin(19, Pin.IN, Pin.PULL_DOWN)
S4 = Pin(18, Pin.IN, Pin.PULL_DOWN)
#button = Pin(22, Pin.IN, Pin.PULL_DOWN)
on = 0
current_pos = (5,2)
depot_1 = (5,4)
depot_2  = (5, 0)
start = (5, 2)
boxes_delivered = 0

# Nav_Grid is in [N, E, S, W]
Nav_Grid = [
    [[0,1,1,0] , [0,1,0,1] , [0,1,1,1] , [0,1,1,1] , [0,0,1,1]],
    [[1,0,1,0] , [0,1,0,0] , [1,0,1,1] , [1,0,0,0] , [1,0,1,0]],
    [[1,1,1,0] , [0,1,0,1] , [1,1,0,1] , [0,1,1,1] , [1,0,1,1]],
    [[1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [1, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 1, 1]],
    [[1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]]
]


### Main loop

def navigate(start, end):
    waypoints = astar(Nav_Grid, start, end)
    sense = [S1.value(), S2.value(), S3.value(), S4.value()]
    
    while len(waypoints) > 0:
        # Line following when no junction detected
        while sense[0] == 0 and sense[3] == 0:
            sense = [S1.value(), S2.value(), S3.value(), S4.value()]
            print(sense)
            LineFollow()
            sleep(0.05)

        # Sees junction
        print(sense, waypoints[0][1])
        if True:
            print("Correct junction detected")
            if waypoints[0][2] == "Straight":
                move(50,1)
            elif waypoints[0][2] == "Right":
                motor_left.turn(90)
                sleep(1)
            elif waypoints[0][2] == "Left":
                motor_right.turn(-90)
                sleep(1)
            waypoints.pop(0)
            
            for i in range(10):
                LineFollow()
                sleep(0.1)

        sleep(0.1)

#handles movement and mechanism to pick up box
# also calls qr code scan and returns drop off coordinates
# should leave robot in N orientation away from junctions
def pickup():
    return

def dropoff():
    return


while True:
    
    
    #When start button is pressed
    navigate((0,0), (2, 1))
    break

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

