# ----------
# compute_value returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# that cell will have a value of 99.
# ----------

def costs_and_optimum_policy(grid, goal, cost):
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True
    
    while change:
        change = False
        
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                # only change the goal state once
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True
                
                # if the current grid cell is not an obstacle
                elif grid[x][y] == 0:
                    
                    # check all four directions
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]
                        
                        # if the grid cell is valid, compute the hypothetical value of that grid cell
                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            v2 = value[x2][y2] + cost
                            
                            # only update if its less than the current value
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]
                                
                                
    
    for row in value:
        print(row)
        
    for row in policy:
        print(row)
    
    

if __name__ == "__main__":
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 1],
            [0, 1, 0, 1, 0, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0]]
    goal = [len(grid)-1, len(grid[0])-1]
    cost = 1 # the cost associated with moving from a cell to an adjacent one
    
    delta = [[-1, 0 ], # go up
             [ 0, -1], # go left
             [ 1, 0 ], # go down
             [ 0, 1 ]] # go right
    
    delta_name = ['^', '<', 'v', '>']
    
    costs_and_optimum_policy(grid, goal, cost)
