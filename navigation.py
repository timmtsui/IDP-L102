
import matplotlib.pyplot as plt
import numpy as np
import heapq

# Define the navigation grid
Nav_Grid = [
    [[0,1,1,0] , [0,1,0,1] , [0,1,1,1] , [0,1,1,1] , [0,0,1,1]],
    [[1,0,1,0] , [0,1,0,0] , [1,0,1,1] , [1,0,0,0] , [1,0,1,0]],
    [[1,1,1,0] , [0,1,0,1] , [1,1,0,1] , [0,1,1,1] , [1,0,1,1]],
    [[1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 1, 0]],
    [[1, 1, 1, 0], [1, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 1, 1]],
    [[1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]]
]


def visualize_nav_grid(nav_grid):
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
    
    plt.gca().invert_yaxis()
    plt.show()

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

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
            return path
        
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


# Example usage
start = (5, 4)
end = (1, 1)
path = astar(Nav_Grid, start, end)

# Visualize the grid
#visualize_nav_grid(Nav_Grid)

