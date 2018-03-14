import robot_class as rc
import random
from math import sqrt

world_size = 100.0
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]

def evaluate(r, p):
    """ Given a robot and a list of particles, returns the mean error (Euclidean distance)
        of all the particles in the list """
    
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))


## MAIN =======================================================================
if __name__ == "__main__":
        
    N = 1000    # number of particles
    localizing_robot = rc.robot(world_size, landmarks)
    
    # initialize N random particles
    particles = []
    for i in range(N):
        x = rc.robot(world_size, landmarks)
        sensor_noise = 3.0 
        x.set_noise(0.05, 0.05, sensor_noise)
        particles.append(x)
     
    # simulate the robot moving about its environment until the solution converges
    allowable = 0.3 * sensor_noise
    mean_error = 100.0
    print('STARTING SIMULATION...\n Initial robot position: ', end='')
    print(localizing_robot)
    print('\nTimestep | Mean Error\n', end='')
    
    t = 0
    while mean_error > allowable:
        
        # Robot moves
        turn_cmd = random.random() * 0.2
        forward_cmd = random.random() * 8
        
        localizing_robot = localizing_robot.move(turn_cmd, forward_cmd)
        Z = localizing_robot.sense()
        
        # simulate that motion update for all N particles
        for i in range(N):
            particles[i] = particles[i].move(turn_cmd, forward_cmd)
    
        # Calculate importance weights    
        weights = []
        for i in range(N):
            weights.append(particles[i].measurement_prob(Z))
            
        
        # Resample using the resampling wheel algorithm
        resampled_particles = []
        
        max_weight = max(weights)
        beta = 0
        index = int(random.random() * N)
        
        for i in range(N):
            beta += random.random()*2*max_weight
            
            while beta > weights[index]:
                beta -= weights[index]
                index = (index+1) % N
                
            resampled_particles.append(particles[index])
        
        
        particles = resampled_particles
        
        mean_error = evaluate(localizing_robot, particles)
        print('{: ^10}'.format('%d' %(t)), end='')
        print('   %2.4f' %mean_error)
        
        t += 1
        
    print('Solution Converged\n')

        
        
        



