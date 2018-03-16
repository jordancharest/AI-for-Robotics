# ----------
# 
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


def search(grid,init,goal,cost):

    # inititate a closed grid the same size as the world grid
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    
    x = init[0]
    y = init[1]
    g = 0
    
    open = [[g, x, y]]

    # Breadth First Search* (choosing the element with the lowest cost rather than using a queue)
    while len(open) > 0:
        
        # choose the node with the lowest cost
        open.sort()
        next_node = open.pop(0)
        g = next_node[0]
        x = next_node[1]
        y = next_node[2]
        
        # if you found the goal you are done
        if x == goal[0] and y == goal[1]:
            return next_node
        
        else:   # check all four directions
            for i in range(len(delta)):
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]
                
                # if it's a valid location, add it to the queue to be explored
                # and close it off in the 'closed' grid
                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                    if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                        g2 = g + cost
                        open.append([g2, x2, y2])
                        closed[x2][y2] = 1
        
        
    return 'fail'
        
        


if __name__ == "__main__":
    grid = [[0, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 0]]
    init = [0, 0]
    goal = [len(grid)-1, len(grid[0])-1]
    cost = 1
    
    result = search(grid, init, goal, cost)
    print(result)
