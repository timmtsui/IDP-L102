import matplotlib.pyplot as plt
import numpy as np
import heapq

def visualize_nav_grid(nav_grid, waypoints):
    rows, cols = len(nav_grid), len(nav_grid[0])
    fig, ax = plt.subplots(figsize=(cols, rows))
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(-0.5, rows - 0.5)
    ax.set_xticks(np.arange(cols)-0.5, minor=True)
    ax.set_yticks(np.arange(rows)-0.5, minor=True)
    ax.grid(which='minor', color='black', linestyle='-', linewidth=1)
    ax.tick_params(bottom=False, left=False, labelbottom=False, labelleft=False)
    
    for r in range(rows):
        for c in range(cols):
            n, e, s, w = nav_grid[r][c]
            if n and r > 0:
                ax.plot([c, c], [r, r-1], 'b-', linewidth=2)
            if e and c < cols - 1:
                ax.plot([c, c+1], [r, r], 'b-', linewidth=2)
            if s and r < rows - 1:
                ax.plot([c, c], [r, r+1], 'b-', linewidth=2)
            if w and c > 0:
                ax.plot([c, c-1], [r, r], 'b-', linewidth=2)
    
    for i, (pos, sensors, direction) in enumerate(waypoints):
        r, c = pos
        ax.plot(c, r, 'ro', markersize=8)
        ax.annotate(f"{direction}\n{sensors}", (c, r), textcoords="offset points", xytext=(0,10), ha='center', fontsize=8, color='red')
    
    plt.gca().invert_yaxis()
    plt.show()

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_expected_sensors(approach_dir, junction_config):
    compass = [0,90,180,270]
    if sum(junction_config) == 3:  # T-junctions or crossroads
        # compass = ["N","E","S","W"]
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
        onejunc = None
        for i in range(0,3):
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
                               270 if prev and prev[1] > curr[1] else 0
                
                expected_sensors = get_expected_sensors(approach_dir, nav_grid[curr[0]][curr[1]])
                waypoints.append((curr, expected_sensors, direction))

            for i in waypoints:
                print(i)
                if i[1] == [0,0,0,0]:
                    #waypoints.remove(i)
                    pass

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

# Define the navigation grid
Nav_Grid = [
    [[0,1,1,0] , [0,1,0,1] , [0,1,1,1] , [0,1,1,1] , [0,0,1,1]],
    [[1,0,1,0] , [0,1,0,0] , [1,0,1,1] , [1,0,0,0] , [1,0,1,0]],
    [[1,1,1,0] , [0,1,0,1] , [1,1,0,1] , [0,1,1,1] , [1,0,1,1]],
    [[1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [1, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 1, 1]],
    [[1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]]
]

# Example usage
start = (0, 0)
end = (5, 4)
waypoints = astar(Nav_Grid, start, end)
print("Waypoints:", waypoints)

# Visualize the grid
visualize_nav_grid(Nav_Grid, waypoints)


