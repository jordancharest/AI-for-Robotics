# Implementation of the gradient descent path smoothing algorithm

from copy import deepcopy

def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print ('['+ ', '.join('%.3f'%x for x in old) + \
               '] -> ['+ ', '.join('%.3f'%x for x in new) +']')

    
def smooth_w_fixed_points(path, fixed, weight_data = 0.1, weight_smooth = 0.1, tolerance = 0.000001, cyclic = False):

    # Make a deep copy of path into newpath
    new_path = deepcopy(path)
    
    if (cyclic):
        iter_range = range(len(path))
    else:
        iter_range = range(1, len(path)-1)  # don't smooth the endpoints
    
    change = tolerance
    
    while change >= tolerance:
        change = 0.0
        
        # iterate over each coordinate individually (treat x and y as independent)
        
        for i in iter_range:
            if not fixed[i]:
                for j in range(len(path[0])):
                    start = new_path[i][j]  # cache to compute how much we have changed (break after convergence)
                    
                    # gradient descent update
                    new_path[i][j] += weight_smooth * (new_path[(i-1)%len(path)][j] + new_path[(i+1)%len(path)][j] - \
                                        2.0 * new_path[i][j]) + \
                                        (weight_smooth / 2.0) * (2.0 * new_path[(i-1)%len(path)][j] - \
                                        new_path[(i-2)%len(path)][j] - new_path[i][j]) + \
                                        (weight_smooth / 2.0) * (2.0 * new_path[(i+1)%len(path)][j] - \
                                        new_path[(i+2)%len(path)][j] - new_path[i][j])
                    
                    # mod len(path) above will be irrelevant in the non-cyclic case
                    
                    change = abs(new_path[i][j] - start)                                
                                 
    return new_path



def smooth(path, weight_data = 0.1, weight_smooth = 0.1, tolerance = 0.000001, cyclic = False):

    # Make a deep copy of path into newpath
    new_path = deepcopy(path)
    
    if (cyclic):
        iter_range = range(len(path))
    else:
        iter_range = range(1, len(path)-1)  # don't smooth the endpoints
    
    change = tolerance
    
    while change >= tolerance:
        change = 0.0
        
        # iterate over each coordinate individually (treat x and y as independent)
        
        for i in iter_range:     
            for j in range(len(path[0])):
                start = new_path[i][j]  # cache to compute how much we have changed (break after convergence)
                
                # gradient descent update
                new_path[i][j] += weight_data * (path[i][j] - new_path[i][j]) +  \
                                    weight_smooth * (new_path[(i+1)%len(path)][j] + new_path[(i-1)%len(path)][j] - 2*new_path[i][j])
                
                # mod len(path) above will be irrelevant in the non-cyclic case
                
                change = abs(new_path[i][j] - start)                                
                                 
    return new_path


## MAIN =======================================================================
if __name__ == "__main__":
    
    print('Smoothing')
    path = [[0, 0],
            [0, 1],
            [0, 2],
            [1, 2],
            [2, 2],
            [3, 2],
            [4, 2],
            [4, 3],
            [4, 4]]
    printpaths(path,smooth(path))
    
    # Test smoothing with fixed points
    print('\nSmoothing with fixed points')
    path = [[0, 0],
            [0, 1],
            [0, 2],
            [1, 2],
            [2, 2],
            [3, 2],
            [4, 2],
            [4, 3],
            [4, 4]]
    fixed = [0,0,1,0,0,0,1,0,0]
    printpaths(path,smooth_w_fixed_points(path, fixed))
    
    # Test cyclic smoothing
    print('\nCyclic smoothing')
    path=[[0, 0], 
          [1, 0],
          [2, 0],
          [3, 0],
          [4, 0],
          [5, 0],
          [6, 0],
          [6, 1],
          [6, 2],
          [6, 3],
          [5, 3],
          [4, 3],
          [3, 3],
          [2, 3],
          [1, 3],
          [0, 3],
          [0, 2],
          [0, 1]]
    printpaths(path,smooth(path, cyclic=True))
    
    
    # Test cyclic smoothing with fixed points
    print('\nCyclic smoothing with fixed points')
    path=[[0, 0], 
          [1, 0],
          [2, 0],
          [3, 0],
          [4, 0],
          [5, 0],
          [6, 0],
          [6, 1],
          [6, 2],
          [6, 3],
          [5, 3],
          [4, 3],
          [3, 3],
          [2, 3],
          [1, 3],
          [0, 3],
          [0, 2],
          [0, 1]]
    fixed = [1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0]
    printpaths(path,smooth_w_fixed_points(path, fixed, cyclic=True))