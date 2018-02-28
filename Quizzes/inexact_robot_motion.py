# Basic localization intuition
# Modeled as a circular world with five blocks that the robot can reside in

pDistribution = [0.2, 0.2, 0.2, 0.2, 0.2] # Initial probability distribution
world = ['green', 'red', 'red', 'green', 'green']   # the markings on each of the five blocks in the world
measurements = ['red', 'green'] # sequential sensor measurements taken by the robot
motions = [1,1] # sequental motions by the robot (1 block to the right)

pHit = 0.7     # probability that the sensor reading is a true positive 
pMiss = 0.1    # probability that the sensor reading is a false negative

pExact = 0.8        # probability that the robot moves the same number of blocks that it was commanded to
pOvershoot = 0.1    # probability that the robot overshoots the command by 1 block
pUndershoot = 0.1   # probability that the robot undershoots the command by 1 block


# Sense the color of the block
def sense(p, Z):
    q = []
    # if the sensed color is the same color as the world block, 
    # update the belief function to show that we are more likely in that block
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss)) 
    s = sum(q)
    
    # normalize
    for i in range(len(q)):
        q[i] = q[i] / s
    return q


def move(p, numBlocks):
    q = []
    # update probability for each block in the world given a small probability
    # of overshoot and undershoot
    for i in range(len(p)):
        s = pExact * p[(i-numBlocks) % len(p)]
        s = s + pOvershoot * p[(i-numBlocks-1) % len(p)]
        s = s + pUndershoot * p[(i-numBlocks+1) % len(p)]
        q.append(s)
    return q

# the localization cycle: sense, move, sense, move, etc.
for i in range(min(len(motions), len(measurements))):
    pDistribution = sense(pDistribution, measurements[i])
    pDistribution = move(pDistribution, motions[i])

print (pDistribution)
