# --------------
# USER INSTRUCTIONS
#
# stochastic_value 
# returns two grids. The first grid, value, should 
# contain the computed value of each cell as shown 
# in the video. The second grid, policy, should 
# contain the optimum policy for each cell.
#

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>'] # Use these when creating your policy grid.


def stochastic_value(grid, goal, cost_step, collision_cost, success_prob):
    failure_prob = (1.0 - success_prob)/2.0 # Probability(stepping left) = prob(stepping right) = failure_prob
    value = [[collision_cost for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    
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
                        
                        x_fail_1 = x + delta[(a+1) % len(delta)][0]
                        y_fail_1 = y + delta[(a+1) % len(delta)][1]
                        
                        x_fail_2 = x + delta[(a-1) % len(delta)][0]
                        y_fail_2 = y + delta[(a-1) % len(delta)][1]
                            
                            
                        # if the grid cell is valid, compute the hypothetical value of that grid cell
                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            
                            # compute values for failed motions
                            if x_fail_1 >= 0 and x_fail_1 < len(grid) and y_fail_1 >= 0 and y_fail_1 < len(grid[0]) and grid[x_fail_1][y_fail_1] == 0:
                                val_fail_1 = failure_prob * value[x_fail_1][y_fail_1]
                            else:
                                val_fail_1 = failure_prob * collision_cost
                                
                            if x_fail_2 >= 0 and x_fail_2 < len(grid) and y_fail_2 >= 0 and y_fail_2 < len(grid[0]) and grid[x_fail_2][y_fail_2] == 0:
                                val_fail_2 = failure_prob * value[x_fail_2][y_fail_2]
                            else:
                                val_fail_2 = failure_prob * collision_cost
                                
                                
                            v2 = success_prob * value[x2][y2] + val_fail_1 + val_fail_2 + cost_step
                            
                            # only update if its less than the current value
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]
        
    return value, policy

# ---------------------------------------------
#  Use the code below to test your solution
# ---------------------------------------------

grid = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 1, 0]]
goal = [0, len(grid[0])-1] # Goal is in top right corner
cost_step = 1
collision_cost = 1000
success_prob = 0.5

value,policy = stochastic_value(grid,goal,cost_step,collision_cost,success_prob)
for row in value:
    print(row)
for row in policy:
    print(row)

# Expected outputs:
#
#[471.9397246855924, 274.85364957758316, 161.5599867065471, 0],
#[334.05159958720344, 230.9574434590965, 183.69314862430264, 176.69517762501977], 
#[398.3517867450282, 277.5898270101976, 246.09263437756917, 335.3944132514738], 
#[700.1758933725141, 1000, 1000, 668.697206625737]


#
# ['>', 'v', 'v', '*']
# ['>', '>', '^', '<']
# ['>', '^', '^', '<']
# ['^', ' ', ' ', '^']
