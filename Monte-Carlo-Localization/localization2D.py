# The function localize takes the following arguments:
#
# world:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should returns a 2D list (of the same dimensions as world)
# that gives the probabilities that the robot occupies
# each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up

def localize(world,measurements,motions,sensor_right,p_move):
    # initializes p to a uniform distribution over a grid of the same dimensions as world
    p_init = 1.0 / float(len(world)) / float(len(world[0]))
    p_distribution = [[p_init for row in range(len(world[0]))] for col in range(len(world))]
    
    for i in range(min(len(motions), len(measurements))):
        p_distribution = move(p_distribution, world, motions[i], p_move)
        p_distribution = sense(p_distribution, world, measurements[i], sensor_right)
        
            
    
    return p_distribution


# Sense the color of the block
def sense(p_distribution, world, measurement, sensor_right):
    temp_world = [[0.0 for row in range(len(world[0]))] for col in range(len(world))]
    sensor_wrong = 1.0 - sensor_right
    sum = 0.0
   
    # compute new probabilities
    for row in range(len(p_distribution)):
        for col in range(len(p_distribution[row])):
            hit = (measurement == world[row][col])
            temp_world[row][col] = p_distribution[row][col] * (hit*sensor_right + (1-hit)*sensor_wrong)
            sum += temp_world[row][col]
            
    # Normalize
    for row in range(len(temp_world)):
        for col in range(len(temp_world[row])):
            temp_world[row][col] /= sum
                
                
    return temp_world
    


# attempt to move to new block
def move(p_distribution, world, motion, p_move):
    temp_world = [[0.0 for row in range(len(world[0]))] for col in range(len(world))]
    p_stay = 1.0 - p_move
    
    for row in range(len(p_distribution)):
        for col in range(len(p_distribution[row])):
            # circular world
            temp_world[row][col] = (p_move * p_distribution[(row-motion[0]) % len(p_distribution)][(col-motion[1]) % len(p_distribution[row])]) + (p_stay * p_distribution[row][col])
    
    return temp_world


def show_world(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print ('[' + ',\n '.join(rows) + ']')
    
    
def check_answer(matrix1, matrix2, allowable_error):
    if len(matrix1) != len(matrix2):
        return False
    
    for row in range(len(matrix1)):
        for col in range(len(matrix1[row])):
            if len(matrix1[row]) != len(matrix2[row]):
                return False
            elif abs(matrix1[row][col] - matrix2[row][col]) > allowable_error:
                return False
            
    return True
