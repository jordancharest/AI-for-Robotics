
 
## --------
## TEST CASES:
## 
##1) Calling the particle_filter function with the following
##    motions and measurements should return a [x,y,orientation]
##    vector near [x=93.476 y=75.186 orient=5.2664], that is, the
##    robot's true location.

from math import pi
import car_class as car


if __name__ == "__main__":

    world_size = 100.0
    landmarks  = [[20.0, 30.0], [80.0, 60.0], [20.0, 90.0], [80.0, 20.0]]
    
    ## 2) You can generate test cases by generating
    ##    measurements using the generate_ground_truth function.
    ##    It will print the robot's last location when calling it.
    ##
    number_of_iterations = 10
    motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
    
    x = car.generate_ground_truth(motions, world_size, landmarks)
    final_robot = x[0]
    measurements = x[1]
    estimated_position = car.particle_filter(motions, measurements, world_size, landmarks)

    car.print_measurements(measurements)
    print('\nGround truth:    ',end='')
    print(final_robot)
    print('Particle filter: ', end='')
    print(estimated_position)
    print('Code check:      ', end='')
    print(car.check_output(final_robot, estimated_position))
    
    
    
