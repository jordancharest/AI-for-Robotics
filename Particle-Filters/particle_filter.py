import robot_class as rc

if __name__ == "__main__":
        
    N = 1000
    myrobot = rc.robot()
    myrobot = myrobot.move(0.1, 5.0)
    Z = myrobot.sense()
    
    particles = []
    
    # initialize N random particles
    for i in range(N):
        x = rc.robot()
        x.set_noise(0.05, 0.05, 5.0)
        particles.append(x)
    
    # simulate motion for all N particles
    for i in range(N):
        particles[i] = particles[i].move(0.1, 5.0)
    
    weights = []
    # Calculate importance weights
    for i in range(N):
        weights.append(particles[i].measurement_prob(Z))
    
    print (weights)



