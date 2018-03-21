
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

def PID_control(robot, p_gain, d_gain, i_gain, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    x_trajectory.append(robot.x)
    y_trajectory.append(robot.y)
    
    last_CTE = robot.y
    error_sum = 0.0
    
    for i in range(n):
        CTE = robot.y  # crosstrack error; desired trajectory is x-axis (so error is simply the robot's y value)
        error_sum += robot.y
        steering = -p_gain * CTE  -  d_gain * (CTE - last_CTE)  -  i_gain * error_sum 
        robot.move(steering, speed)
        
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        last_CTE = CTE
        

    return x_trajectory, y_trajectory

if __name__ == "__main__":
    
    robot = Robot()
    robot.set(0.0, 1.0, 0.0)
    #robot.set_steering_drift(10.0/180 * np.pi)
    
    # Test proportional control    
    x_trajectory, y_trajectory = P_control(robot, 0.1)
    n = len(x_trajectory)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))
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
    x_trajectory, y_trajectory = PID_control(robot, 0.2, 3.0, 0.01)
    n = len(x_trajectory)
    
    ax3.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
    ax3.plot(x_trajectory, np.zeros(n), 'r', label='reference')
    
    
    
