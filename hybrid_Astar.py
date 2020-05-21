import numpy as np
import matplotlib.pyplot as plt

NUM_THETA_CELLS = 45

class State:
    def __init__(self, x=0, y=0, theta=0, g=0, f=0, parent=None):
        self.x, self.y, self.theta, self.g, self.f, self.parent = x, y, theta, g, f, parent
    
    def permitted(self, grid):
        '''
        True if permitted
        '''
        condition = not ((self.x < 0 or self.x >= grid.shape[1]) or (self.y < 0 or self.y >= grid.shape[0]))
        return condition


def bicycle(state, delta_rad, speed=1, length=4.5):
    '''
    The implementation of the bicycle model.
    '''
    next_theta = state.theta + speed/length * np.tan(delta_rad)
    if next_theta < 0 or next_theta > 2*np.pi: next_theta = (next_theta + 2*np.pi) % (2*np.pi)
    next_x = state.x + speed*np.cos(state.theta)
    next_y = state.y + speed*np.sin(state.theta)
    return next_x, next_y, next_theta

def heuristic(x, y, goal):
    '''
    A* heuristic function
    resembles the distance to the goal location
    '''
    return (abs(y - goal.y) + abs(x - goal.x))
    # return np.sqrt((y-goal.y)**2 + (x-goal.x)**2)

def expand(state, goal):
    '''
    Returns a list of possible next states for a range of steering angles.
    Calls the implementation of the bicycle model and the A* heuristic function.
    '''
    next_states = []
    for delta in range(-35,35+15,15):
        next_x, next_y, next_theta = bicycle(state, delta*np.pi/180)
        next_g = state.g + 1
        next_f = next_g + heuristic(next_x, next_y, goal)
        next_states.append(State(next_x, next_y, next_theta, next_g, next_f, parent=state))
    
    return next_states

def theta_to_stack_number(theta):
    '''
    Takes an angle in radians and returns which stack in the 3D config. space
    this angle corresponds to.
    Angles near 0 go in the lower stack while angles near 2*pi go to the higher stack.
    '''
    new_theta = (theta+2*np.pi) % (2*np.pi)
    stack_number = int(round(new_theta * NUM_THETA_CELLS / (2*np.pi))) % NUM_THETA_CELLS
    return stack_number

def idx(num):
    '''
    Returns the index in the grid
    '''
    # map_size = 25   # size of map in meters, assuming map is square. map is map_size x map_size
    # grid_size = 500 # number of grid cells in each dimension, assuming grid is square. grid is grid_size x grid_size
    # index = int(round(num*grid_size/map_size))
    # return index
    return int(np.floor(num))

def path_reconstruction(expand_node):
    path = []
    current = expand_node

    while current.parent:
        elmt = [current.x, current.y, current.theta]
        path.append(elmt)
        current = current.parent

    elmt = [current.x, current.y, current.theta]
    path.append(elmt)

    return np.array(path[::-1])

def search(grid, start, goal):
    '''
    '''
    opened = []
    closed = np.zeros((NUM_THETA_CELLS, grid.shape[1], grid.shape[0]))

    state = State(start.x, start.y, start.theta, 0, heuristic(start.x, start.y, goal))
    opened.append(state)
    stack_number = theta_to_stack_number(state.theta)
    closed[stack_number][idx(state.x)][idx(state.y)] = 1

    while len(opened) != 0:
        opened.sort(key=lambda state:state.f)   # sorting the opened states to start with the lowest f value.
        current = opened.pop(0)     # returns the first state and removes it from opened list
        path = path_reconstruction(current)
        # Plotting
        # plt.plot(current.x, current.y, '.k')
        plt.quiver(current.x, current.y, np.cos(current.theta), np.sin(current.theta), units='inches', 
                    scale=5, zorder=3, color='green', width=0.007, headwidth=3, headlength=4)
        plt.plot(path[:,0], path[:,1], '--')
        plt.draw()
        plt.pause(0.03)
        # 
        if (idx(current.x) == goal.x) and (idx(current.y) == goal.y):   # goal is reached
            path = path_reconstruction(current)
            return path
        next_states = expand(current, goal)
        for next_state in next_states:
            if not next_state.permitted(grid):   # ignoring states outside of grid
                continue
            stack_number = theta_to_stack_number(next_state.theta)
            if closed[stack_number][idx(next_state.x)][idx(next_state.y)] == 0 and grid[idx(next_state.x)][idx(next_state.y)] == 0:    # if not visited before and not obstacle
                opened.append(next_state)
                closed[stack_number][idx(next_state.x)][idx(next_state.y)] = 1    # mark as closed/visited
    print("Couldn't Find Path")
    return path

if __name__ == "__main__":
    start = State(x=5, y=5, theta=0)
    goal = State(x=50, y=20, theta=0)
    grid = np.zeros((100,100))
    # plt.figure()
    plt.show()
    plt.grid(b=True, which='major', color='#666666', linestyle='-')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    ax = plt.gca()
    # ax.set_xlim(0, grid.shape[0])
    # ax.set_ylim(0, grid.shape[1])
    path = search(grid, start, goal)
    plt.figure()
    plt.plot(path[:,0], path[:,1], 'k')
    plt.show()
    print(path)