
import random
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------
# 
# this is the PID-controlled Robot class
#

class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)



def reset_robot():
    """
    Resets the robot back to the initial position and drift.
    """
    robot = Robot()
    robot.set(0.0, 1.0, 0.0)
    robot.set_steering_drift(10.0 / 180.0 * np.pi)
    return robot



def P_control(robot, p_gain, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)
    
    for i in range(n):
        CTE = robot.y  # crosstrack error; desired trajectory is x-axis (so error is simply the robot's y value)
        steering = -p_gain * CTE
        robot.move(steering, speed)
        
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

        

    return x_trajectory, y_trajectory


def PD_control(robot, p_gain, d_gain, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)
    
    last_CTE = robot.y
    
    for i in range(n):
        CTE = robot.y  # crosstrack error; desired trajectory is x-axis (so error is simply the robot's y value)
        steering = -p_gain * CTE - d_gain * (CTE - last_CTE)
        robot.move(steering, speed)
        
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        last_CTE = CTE
        

    return x_trajectory, y_trajectory

def PID_control(robot, gains, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)
    
    p_gain = gains[0]
    d_gain = gains[1]
    i_gain = gains[2]
    
    last_CTE = robot.y
    error_sum = 0.0
    err = 0.0
    
    for i in range(n * 2):
        CTE = robot.y  # crosstrack error; desired trajectory is x-axis (so error is simply the robot's y value)
        error_sum += robot.y
        steering = -p_gain * CTE  -  d_gain * (CTE - last_CTE)  -  i_gain * error_sum 
        robot.move(steering, speed)
        
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        last_CTE = CTE
        
        if i >= n:
            err += CTE ** 2
        
    avg_error = err/n
    return x_trajectory, y_trajectory, avg_error


# Coordinate Ascension - algorithm to find the optimal parameters
def twiddle(tol=0.2): 
    params = [0.0, 0.0, 0.0]
    updates = [0.5, 1.0, 0.1]   # make educated guesses on the likely update size for better results
    robot = reset_robot()
    x_trajectory, y_trajectory, best_error = PID_control(robot, params)
    
    # we are finished when the updates across all parameters are very small (we are very close to the optimum)
    while sum(updates) > tol:
        for i in range(len(params)):
            # increment the parameter and get the error
            params[i] += updates[i]
            robot = reset_robot()
            x_trajectory, y_trajectory, error = PID_control(robot, params)
            
            # if we improved the error, increase update size for next loop and move to next parameter
            if error < best_error:
                best_error = error
                updates[i] *= 1.1
            
            # if we didn't, try decreasing the parameter
            else:
                params[i] -= 2 * updates[i]
                robot = reset_robot()
                x_trajectory, y_trajectory, error = PID_control(robot, params)
                
                # if we improved the error, increase update size for next loop and move to next parameter
                if  error < best_error:
                    best_error = error
                    updates[i] += 1.1
                
                # if neither worked, then the step size is probably too large, decrease and try again next after the other parameters
                else:
                    params[i] += updates[i]
                    updates[i] *= 0.9
                    
    return params, best_error
            


if __name__ == "__main__":
    
    robot = Robot()
    robot.set(0.0, 1.0, 0.0)
    #robot.set_steering_drift(10.0/180 * np.pi)
    
    # Test proportional control    
    x_trajectory, y_trajectory = P_control(robot, 0.1)
    n = len(x_trajectory)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
    ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

    
    # Test proportional-derivative control
    robot.set(0.0, 1.0, 0.0)
    x_trajectory, y_trajectory = PD_control(robot, 0.1, 1.2)
    n = len(x_trajectory)
    
    ax2.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
    ax2.plot(x_trajectory, np.zeros(n), 'r', label='reference')
    
    # Test proportional-integral-derivative control
    robot.set(0.0, 1.0, 0.0)
    gains = [0.2, 3.0, 0.01]
    x_trajectory, y_trajectory, error = PID_control(robot, gains)
    n = len(x_trajectory)
    
    fig, (ax3, ax4) = plt.subplots(2, 1, figsize=(8, 8))
    ax3.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
    ax3.plot(x_trajectory, np.zeros(n), 'r', label='reference')
    
    
    # optimize the controller gains
    params, err = twiddle()
    print("Final twiddle error = {}".format(err))
    print('Optimal gain values: \n Proportional: {:.6}\n Derivative: {:.6}\n Integral: {:.6}'.format(params[0], params[1], params[2]))
    robot = reset_robot()
    x_trajectory, y_trajectory, err = PID_control(robot, params)
    n = len(x_trajectory)
    
    ax4.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
    ax4.plot(x_trajectory, np.zeros(n), 'r', label='reference')
    
    
    
    
