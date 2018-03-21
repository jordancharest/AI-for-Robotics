# Implementation of the gradient descent path smoothing algorithm

from copy import deepcopy

def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print ('['+ ', '.join('%.3f'%x for x in old) + \
               '] -> ['+ ', '.join('%.3f'%x for x in new) +']')


def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):

    # Make a deep copy of path into newpath
    new_path = deepcopy(path)
    
    change = tolerance
    
    while change >= tolerance:
        change = 0.0
        
        # iterate over each coordinate individually (treat x and y as independent)
        for i in range(1, len(path)-1):     # don't smooth the endpoints
            for j in range(len(path[0])):
                start = new_path[i][j]  # cache to compute how much we have changed (break after convergence)
                
                # gradient descent update
                new_path[i][j] += weight_data * (path[i][j] - new_path[i][j]) +  \
                                    weight_smooth * (new_path[i+1][j] + new_path[i-1][j] - 2*new_path[i][j])
                
                change = abs(new_path[i][j] - start)                                
                                 
    return new_path


## MAIN =======================================================================
if __name__ == "__main__":
    
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
