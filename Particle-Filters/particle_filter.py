import robot_class as rc
import random
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

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
    x_plot = []
    y_plot = []
    for i in range(N):
        part = rc.robot(world_size, landmarks)
        sensor_noise = 3.0 
        part.set_noise(0.05, 0.05, sensor_noise)
        particles.append(part)
        x_plot.append(part.x)
        y_plot.append(part.y)
        
    # Setup for plotting
    robot_x = []
    robot_y = []
    robot_x.append(localizing_robot.x)
    robot_y.append(localizing_robot.y)
    landmarks_x = []
    landmarks_y = []
    for i in range(len(landmarks)):
        landmarks_x.append(landmarks[i][0])
        landmarks_y.append(landmarks[i][1])

        
     
    print('STARTING SIMULATION...\n Initial robot position: ', end='')
    print(localizing_robot)
    print('\nTimestep | Mean Error\n', end='')
    
    t = 0
    allowable = 0.33 * sensor_noise
    mean_error = 100.0
    
    # simulate the robot moving about its environment until the solution converges
    while mean_error > allowable:
        
        # Robot moves
        forward_cmd = 1 + random.random() * 6
        if (random.random() > 0.5):
            turn_cmd = random.random() * 0.3
        else:
            turn_cmd = -random.random() * 0.3
            
        
        localizing_robot = localizing_robot.move(turn_cmd, forward_cmd)
        Z = localizing_robot.sense()
        
        # Plotting
        robot_x.append(localizing_robot.x)
        robot_y.append(localizing_robot.y)
        
        # simulate that same motion update for all N particles
        for i in range(N):
            particles[i] = particles[i].move(turn_cmd, forward_cmd)
            
            # Plotting
            x_plot.append(particles[i].x)
            y_plot.append(particles[i].y)
            
        
    
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
  
  
    
# Animate the solution convergence
    
def update(frame):
    plt.scatter(robot_x[frame], robot_y[frame], color='r', s=size*3)
    
    plot_particles['position'][:, 0] = x_plot[frame*N:frame*N+N]
    plot_particles['position'][:, 1] = y_plot[frame*N:frame*N+N]

    scat.set_offsets(plot_particles['position'])
    
    return scat
    
    
    
  
plot_particles = np.zeros(N, dtype=[('position', float, 2)])    

fig = plt.figure()
plt.xlim(0, world_size)
plt.ylim(0, world_size)

size = 20
plt.scatter(landmarks_x, landmarks_y, color='k', s=size*8, marker='^')
scat = plt.scatter(x_plot, y_plot, s=size)

animation = FuncAnimation(fig, update, interval=N)
plt.show()
