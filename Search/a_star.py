# ----------
# Successful search() calls return a list
# in the form of [optimal path length, row, col]
#
# If there is no valid path from the start point
# to the goal, search returns the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']


def a_star(grid, heuristic, init, goal, cost):

    # inititate a closed grid the same size as the world grid
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1
    
    # another grid to show when each node was explored
    explored = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    
    # another grid to display the movement it took to the next node
    move = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    
    x = init[0]
    y = init[1]
    g = 0
    f = g + heuristic[x][y]
    
    open = [[f, g, x, y]]

    step = 0
    found = False
    
    # A*
    while len(open) > 0:
        
        # choose the node with the lowest cost (including heuristic)
        open.sort()
        next_node = open.pop(0)
        g = next_node[1]
        x = next_node[2]
        y = next_node[3]
        
        explored[x][y] = step
        step += 1
        
        # if you found the goal you are done
        if x == goal[0] and y == goal[1]:
            print(explored)
            found = True
            break
        
        else:   # check all four directions
            for i in range(len(delta)):
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]
                
                # if it's a valid location, add it to the queue to be explored
                # and close it off in the 'closed' grid
                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                    if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                        g2 = g + cost
                        f2 = g2 + heuristic[x2][y2]
                        open.append([f2, g2, x2, y2])
                        closed[x2][y2] = 1
                        move[x2][y2] = i
        
    
    
    # build the optimal path grid using the symbols in delta_name
    path = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    x = goal[0]
    y = goal[1]
    path[x][y] = '*'
    
    while x != init[0] or y != init[1]:
        x2 = x - delta[move[x][y]][0]
        y2 = y - delta[move[x][y]][1]
        path[x2][y2] = delta_name[move[x][y]]
        x, y = x2, y2
        
    print(path)
     
    if found:
        return next_node
    else:
        return 'fail'
        
        


if __name__ == "__main__":
    grid = [[0, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, 1, 0],
            [0, 0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0, 0]]
    
    heuristic = [[9, 8, 7, 6, 5, 4],
                 [8, 7, 6, 5, 4, 3],
                 [7, 6, 5, 4, 3, 2],
                 [6, 5, 4, 3, 2, 1],
                 [5, 4, 3, 2, 1, 0]]
    
    init = [0, 0]
    goal = [len(grid)-1, len(grid[0])-1]
    cost = 1
    
    result = a_star(grid, heuristic, init, goal, cost)
    print(result)
