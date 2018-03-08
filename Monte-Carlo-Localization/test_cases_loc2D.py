import localization2D as L2D

error = 0.001
# test 0 ----------------------------------------------------------------------
print('Test 0...')
world =  [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]
p_distribution = L2D.localize(world,measurements,motions,sensor_right = 0.7, p_move = 0.8)
# L2D.show_world(p_distribution) # displays your answer
correct_answer = [[0.01105, 0.02464, 0.06799, 0.04472, 0.02465],
                  [0.00715, 0.01017, 0.08696, 0.07988, 0.00935],
                  [0.00739, 0.00894, 0.11272, 0.35350, 0.04065],
                  [0.00910, 0.00715, 0.01434, 0.04313, 0.03642]]


if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)

# test 1 ----------------------------------------------------------------------
print('Test 1...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'G'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0,0]]
sensor_right = 1.0
p_move = 1.0
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 0.0]])


if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)
    

# test 2 ----------------------------------------------------------------------
print('Test 2...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0,0]]
sensor_right = 1.0
p_move = 1.0
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.0, 0.0, 0.0],
     [0.0, 0.5, 0.5],
     [0.0, 0.0, 0.0]])


if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)
    

# test 3 ----------------------------------------------------------------------
print('Test 3...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0,0]]
sensor_right = 0.8
p_move = 1.0
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.06666666666, 0.06666666666, 0.06666666666],
     [0.06666666666, 0.26666666666, 0.26666666666],
     [0.06666666666, 0.06666666666, 0.06666666666]])


if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)
    

# test 4 ----------------------------------------------------------------------
print('Test 4...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 0.8
p_move = 1.0
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.03333333333, 0.03333333333, 0.03333333333],
     [0.13333333333, 0.13333333333, 0.53333333333],
     [0.03333333333, 0.03333333333, 0.03333333333]])

if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)


# test 5 ----------------------------------------------------------------------
print('Test 5...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 1.0
p_move = 1.0
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.0, 0.0, 0.0],
     [0.0, 0.0, 1.0],
     [0.0, 0.0, 0.0]])

if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)
    
    
# test 6 ----------------------------------------------------------------------
print('Test 6...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 0.8
p_move = 0.5
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.0289855072, 0.0289855072, 0.0289855072],
     [0.0724637681, 0.2898550724, 0.4637681159],
     [0.0289855072, 0.0289855072, 0.0289855072]])


if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)
    
    
# test 7 ----------------------------------------------------------------------
print('Test 7...')
world = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 1.0
p_move = 0.5
p_distribution = L2D.localize(world,measurements,motions,sensor_right,p_move)
correct_answer = (
    [[0.0, 0.0, 0.0],
     [0.0, 0.33333333, 0.66666666],
     [0.0, 0.0, 0.0]])


if L2D.check_answer(p_distribution, correct_answer, error):
    print('Pass\n')
else:
    print('Fail')
    L2D.show_world(p_distribution)