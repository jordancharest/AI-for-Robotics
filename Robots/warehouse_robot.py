# -------------------
# Background Information
#
# In this problem, you will build a planner that helps a robot
# find the shortest way in a warehouse filled with boxes
# that he has to pick up and deliver to a drop zone.
# 
# For example:
#
# warehouse = [[ 1, 2, 3],
#              [ 0, 0, 0],
#              [ 0, 0, 0]]
# dropzone = [2,0] 
# todo = [2, 1]
# 
# The robot starts at the dropzone.
# The dropzone can be in any free corner of the warehouse map.
# todo is a list of boxes to be picked up and delivered to the dropzone.
#
# Robot can move diagonally, but the cost of a diagonal move is 1.5.
# The cost of moving one step horizontally or vertically is 1.
# So if the dropzone is at [2, 0], the cost to deliver box number 2
# would be 5.

# To pick up a box, the robot has to move into the same cell as the box.
# When the robot picks up a box, that cell becomes passable (marked 0)
# The robot can pick up only one box at a time and once picked up 
# it has to return the box to the dropzone by moving onto the dropzone cell.
# Once the robot has stepped on the dropzone, the box is taken away, 
# and it is free to continue with its todo list.
# Tasks must be executed in the order that they are given in the todo list.
# You may assume that in all warehouse maps, all boxes are
# reachable from beginning (the robot is not boxed in).

# --------------------
# Parameter Info
#
# warehouse - a grid of values, where 0 means that the cell is passable,
# and a number 1 <= n <= 99 means that box n is located at that cell.
# dropzone - determines the robot's start location and the place to return boxes 
# todo - list of tasks, containing box numbers that have to be picked up
#
# --------------------
# Testing
#
# You may use our test function below, solution_check(),
# to test your code for a variety of input parameters. 
import numpy as np

warehouse = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone = [2,0] 
todo = [2, 1]

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1], # go right
         [-1,-1], # diagonal up-left
         [ 1,-1], # diagonal down-left
         [ 1, 1], # diagonal down-right
         [-1, 1]] # diagonal up-right

# ------------------------------------------
# plan - Returns cost to take all boxes in the todo list to dropzone
#
# ----------------------------------------
def plan(warehouse, dropzone, todo):
    total_cost = 0
    goal = [0,0]

    for box_number in todo:
        
        # reset the closed grid
        closed = [[0 for col in range(len(warehouse[0]))] for row in range(len(warehouse))]
        closed[dropzone[0]][dropzone[1]] = 1    # robot starts in the dropzone
        
        # figure out the coordinates of the box we are looking for
        stop = False
        for row in range(len(warehouse)):
            for col in range(len(warehouse[row])):
                if box_number == warehouse[row][col]:
                    stop = True
                    goal[0] = row
                    goal[1] = col
                    break
                if stop:
                    break
 
              
        # reset the robot to the dropzone location
        x = dropzone[0]
        y = dropzone[1]
        cost = 0
        h = cost + np.sqrt((x - goal[0])**2 + (y - goal[1])**2)
    
        open = [[h, cost, x, y]]
        

        # A* with heuristic computed on-the-fly
        found = False
        while len(open) > 0:
            motion_cost = 1 # reset motion cost for non-diagonal movements
            
            # choose the node with the lowest heuristic (h)
            open.sort()
            next_node = open.pop(0)
            cost = next_node[1]
            x = next_node[2]
            y = next_node[3]
            
            # print('Exploring (',x,',',y,')  Cost:', cost)
            
            # if you found the goal, exit
            if x == goal[0] and y == goal[1]:
                found = True
                warehouse[x][y] = 0 # free this space so we can move through it when searching for the next box
                break
            
            else:   # check all eight directions
                for i in range(len(delta)):
                    if i == 4:  # diagonal moves cost 1.5
                        motion_cost = 1.5
                        
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    
                    # if it's a valid location, add it to the queue to be explored
                    # and close it off in the 'closed' grid
                    if x2 >= 0 and x2 < len(warehouse) and y2 >= 0 and y2 < len(warehouse[0]):
                        if closed[x2][y2] == 0 and (warehouse[x2][y2] == 0 or warehouse[x2][y2] == box_number):
                            cost2 = cost + motion_cost
                            
                            # compute new heuristic on the fly - use Euclidean Distance
                            h2 = cost2 + np.sqrt((x2 - goal[0])**2 + (y2 - goal[1])**2)
                            
                            open.append([h2, cost2, x2, y2])
                            closed[x2][y2] = 1
    
        # double the cost to include the trip back to the dropzone
        # no need to search, the robot is reset to the dropzone location at each iteration
        total_cost += (cost * 2)
        
        # print('Cost to reach box', box_number, 'was', (cost*2))
        
        
    if found:
        return total_cost
    else:
        return 'fail'
    
################# TESTING ##################
       
# ------------------------------------------
# solution check - Checks your plan function using
# data from list called test[]. Uncomment the call
# to solution_check to test your code.
#
def solution_check(test, epsilon = 0.00001):
    answer_list = []
    
    import time
    start = time.clock()
    correct_answers = 0
    for i in range(len(test[0])):
        user_cost = plan(test[0][i], test[1][i], test[2][i])
        true_cost = test[3][i]
        if abs(user_cost - true_cost) < epsilon:
            print ("Test case", i+1, "passed!\n")
            answer_list.append(1)
            correct_answers += 1
            #print "#############################################"
        else:
            print ("Test case ", i+1, "unsuccessful. Your answer ", user_cost, "was not within ", epsilon, "of ", true_cost, '\n') 
            answer_list.append(0)
    runtime =  time.clock() - start
    if runtime > 1:
        print ("Your code is too slow, try to optimize it! Running time was: ", runtime)
        return False
    if correct_answers == len(answer_list):
        print ("\nYou passed all test cases!")
        return True
    else:
        print ("\nYou passed", correct_answers, "of", len(answer_list), "test cases. Try to get them all!")
        return False
    
    
#Testing environment
# Test Case 1 
warehouse1 = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone1 = [2,0] 
todo1 = [2, 1]
true_cost1 = 9
# Test Case 2
warehouse2 = [[   1, 2, 3, 4],
              [   0, 0, 0, 0],
              [   5, 6, 7, 0],
              [ 'x', 0, 0, 8]] 
dropzone2 = [3,0] 
todo2 = [2, 5, 1]
true_cost2 = 21

# Test Case 3
warehouse3 = [[   1, 2,  3,  4, 5, 6,  7],
              [   0, 0,  0,  0, 0, 0,  0],
              [   8, 9, 10, 11, 0, 0,  0],
              [ 'x', 0,  0,  0, 0, 0, 12]] 
dropzone3 = [3,0] 
todo3 = [5, 10]
true_cost3 = 18

# Test Case 4
warehouse4 = [[ 1, 17, 5, 18,  9, 19,  13],
              [ 2,  0, 6,  0, 10,  0,  14],
              [ 3,  0, 7,  0, 11,  0,  15],
              [ 4,  0, 8,  0, 12,  0,  16],
              [ 0,  0, 0,  0,  0,  0, 'x']] 
dropzone4 = [4,6]
todo4 = [13, 11, 6, 17]
true_cost4 = 41

testing_suite = [[warehouse1, warehouse2, warehouse3, warehouse4],
                 [dropzone1, dropzone2, dropzone3, dropzone4],
                 [todo1, todo2, todo3, todo4],
                 [true_cost1, true_cost2, true_cost3, true_cost4]]


if __name__ == "__main__":
    solution_check(testing_suite)


