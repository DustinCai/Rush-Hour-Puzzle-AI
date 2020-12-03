from dataclasses import dataclass, field
import copy
import queue

# BOTTLENECK: continous checks in states_in_frontier without remove the states after it's been used
# remove the states from frontier if already checked? make a check in the beginning of bfs then remove
# it from the states in frontier list

# testing bottleneck purposes
import cProfile
import pstats

# Main function (can also just call best_first_search func)
def rushhour(heuristic, start):
    best_first_search(heuristic, start)

# Best First Search Function
# Arguments: User input for heuristic and start state
def best_first_search(heuristic, start):
    frontier = queue.PriorityQueue()    # list of unexplored states, sorted in a priority queue
    explored_states = []                # list of explored states to not explore again
    states_in_frontier = []             # list of states in frontier to not explored if a duplicate is encountered
    depth = 0

    board = Board(heuristic)
    create_all_vehicles(start, board, depth)    # convert the start state into vehicle objs and store in board obj
    frontier.put(board)

    while not frontier.empty():
        curr_state = frontier.get()             # pop the first state and add to the explored states
        explored_states.append(curr_state)
        if curr_state in states_in_frontier:
            states_in_frontier.remove(curr_state)

        if is_goal_state(curr_state):           # if goal state is reached, print out the path
            path = trace_path(curr_state)
            for state in path:
                state.print_state()
            print("Total moves: ", len(path) - 1)
            print("Total states explored: ", len(explored_states))
            return
        else:
            new_states = curr_state.generate_new_states(explored_states, states_in_frontier) # generate children
            for state in new_states:
                frontier.put(state)                 # put new states into frontier
                states_in_frontier.append(state)    # also put in states_in_frontier

    print("Total moves: ", 0)                       # if the frontier is empty, that means a goal state is not possible
    print("Total states explored: ", len(explored_states))
    return []


# Board Class
# Description: Class to represent each state. Each board object will hold a list of vehicle object,
# the heuristic value for that state, depth of the tree, and pointers to children nodes and/or parent node.
# Argument: Takes the user input value (0 or 1) as the heuristic to use for the state search: 0 for
# blocking heuristic, 1 for the custom heuristic.
class Board:
    def __init__(self, heuristic_to_use):
        self.vehicle_list = []      # list of vehicles objects, contains all we need to know for a state
        self.heuristic_to_use = heuristic_to_use
        self.priority = None        # our priority is the heuristic for the current board
        self.depth = 0
        self.parent = None
        self.child = None

    # Description: Overloads '==' operator for board objects for priority queue.
    # Compares if 2 board objects are equal by comparing each vehicle in their respective vehicle list
    def __eq__(self, other):
        if not isinstance(other, Board):
            return NotImplemented
        for vehicle in self.vehicle_list:
            vehicle_matched = False
            for other_vehicle in other.vehicle_list:
                if vehicle == other_vehicle:
                    vehicle_matched = True
            if vehicle_matched == False: return False
        return True

    # Description: Overloads '<' operator for board objects for priority queue.
    def __lt__(self, other):
        return self.priority < other.priority

    # Description: Prints the heuristic for the board (if needed)
    def print_priority(self):
        print("Heuristic: ", self.priority)

    # Description: Prints the board
    def print_state(self):
        res = []
        for i in range(6):
            row = []
            for j in range(6):
                row.append(self.is_occupied((i, j)))
            res.append(row)
        for i in range(6):
            for j in range(6):
                print(res[i][j], end = " ")
            print()
        print()

    # Description: Increment the depth, used for heuristics
    def incr_depth(self):
        self.depth += 1

    # Description: Adds a vehicle object to the board's list of vehicles
    # Argument: Vehicle: Object
    def add_vehicle(self, vehicle):
        self.vehicle_list.append(vehicle)

    # Description: Returns a vehicle object, given the name of the vehicle
    # Argument: Name of the vehicle: string
    # Returns: Vehicle Object
    def get_vehicle(self, name):
        for vehicle in self.vehicle_list:
            if vehicle.name == name:
                return vehicle

    # Description: Checks whether a position on the board is occupied by a vehicle
    # Argument: (x, y) coordinate to check
    # Returns: Name of the vehicle occupying the position, if no vehicle, then return '-' to signify an empty position
    def is_occupied(self, pos):
        for vehicle in self.vehicle_list:
            if vehicle.orientation == 'vertical':
                if(vehicle.pos[0] <= pos[0] and (vehicle.pos[0] + vehicle.length) > pos[0] and vehicle.pos[1] == pos[1]):
                    return vehicle.name
            elif vehicle.orientation == 'horizontal':
                if(vehicle.pos[1] <= pos[1] and (vehicle.pos[1] + vehicle.length) > pos[1] and vehicle.pos[0] == pos[0]):
                    return vehicle.name
        return '-'


    # Description: Generates new board states by applying each operator (up,down,left,right) on each vehicle in list.
    # Apply the 4 operators on each vehicle on the board. If the operator can be applied (i.e in bound of the board
    # and the next position is not occupied by another vehicle), then make a copy of current board and apply the
    # movement changes (i.e move the vehicle and calculate new heuristic). Check if that new board object is in
    # either the explored_states list or frontier. If it is, do not add to the new_states list. Else, add to list.
    # Argument: Explored States: list, States in frontier: list
    # Returns: New states: list
    def generate_new_states(self, explored_states, states_in_frontier):
        new_states = [] # create list of new states to be added to the frontier

        for vehicle in self.vehicle_list:
            if vehicle.orientation == 'vertical':   # vertical vehicles can only move up or down
                if vehicle.can_move_up() and self.is_occupied((vehicle.pos[0] - 1, vehicle.pos[1])) == '-':
                    new_board = self.copy_self()            # create a copy of the current board
                    new_board.parent = self                 # link between parent & child node for path
                    self.child = new_board
                    new_board.incr_depth()                  # increment depth since this is a new node
                    new_board.move_vehicle(vehicle, 'up')   # apply movement
                    if new_board.heuristic_to_use == 0:     # apply heuristic depending on user input
                        new_board.priority = blocking_heuristic(new_board, new_board.depth)
                    else:
                        new_board.priority = custom_heuristic(new_board, new_board.depth)
                    # check if the new state is in either explored states or the frontier
                    if not self.state_explored(new_board, explored_states, states_in_frontier): new_states.append(new_board)
                if vehicle.can_move_down() and self.is_occupied((vehicle.pos[0] + vehicle.length, vehicle.pos[1])) == '-':
                    new_board = self.copy_self()
                    new_board.parent = self
                    self.child = new_board
                    new_board.incr_depth()
                    new_board.move_vehicle(vehicle, 'down')
                    if new_board.heuristic_to_use == 0:
                        new_board.priority = blocking_heuristic(new_board, new_board.depth)
                    else:
                        new_board.priority = custom_heuristic(new_board, new_board.depth)
                    if not self.state_explored(new_board, explored_states, states_in_frontier): new_states.append(new_board)
            elif vehicle.orientation == 'horizontal':
                if vehicle.can_move_left() and self.is_occupied((vehicle.pos[0], vehicle.pos[1] - 1)) == '-':
                    new_board = self.copy_self()
                    new_board.parent = self
                    self.child = new_board
                    new_board.incr_depth()
                    new_board.move_vehicle(vehicle, 'left')
                    if new_board.heuristic_to_use == 0:
                        new_board.priority = blocking_heuristic(new_board, new_board.depth)
                    else:
                        new_board.priority = custom_heuristic(new_board, new_board.depth)
                    if not self.state_explored(new_board, explored_states, states_in_frontier): new_states.append(new_board)
                if vehicle.can_move_right() and self.is_occupied((vehicle.pos[0], vehicle.pos[1] + vehicle.length)) == '-':
                    new_board = self.copy_self()
                    new_board.parent = self
                    self.child = new_board
                    new_board.incr_depth()
                    new_board.move_vehicle(vehicle, 'right')
                    if new_board.heuristic_to_use == 0:
                        new_board.priority = blocking_heuristic(new_board, new_board.depth)
                    else:
                        new_board.priority = custom_heuristic(new_board, new_board.depth)
                    if not self.state_explored(new_board, explored_states, states_in_frontier): new_states.append(new_board)

        return new_states


    # Description: Move a vehicle by changing the start position of the vehicle. X coordinate for
    # vertical vehicles and Y coordinate for horizontal vehicles
    # Arguments: Vehicle: Object, Type of movement: String
    # Returns: None
    def move_vehicle(self, vehicle_to_move, movement):
        for vehicle in self.vehicle_list:
            if vehicle.name == vehicle_to_move.name:
                if movement == 'up': vehicle.pos = (vehicle.pos[0] - 1, vehicle.pos[1])
                if movement == 'down': vehicle.pos = (vehicle.pos[0] + 1, vehicle.pos[1])
                if movement == 'left': vehicle.pos = (vehicle.pos[0], vehicle.pos[1] - 1)
                if movement == 'right': vehicle.pos = (vehicle.pos[0], vehicle.pos[1] + 1)

    # Description: Checks if a state is in either explored states or frontier
    # Arguments: Explored states: list, States in frontier: list
    # Returns: True if the state is in either list, else False
    def state_explored(self, new_state, explored_states, states_in_frontier):
        if new_state in explored_states:
            return True
        if new_state in states_in_frontier:
            return True
        return False

    # Description: Makes a new board copy of itself
    # Arguments: None
    # Returns: New board: obj
    def copy_self(self):
        new_board = Board(self.heuristic_to_use)
        new_board.priority = self.priority
        new_board.depth = self.depth

        for vehicle in self.vehicle_list:
            new_board.add_vehicle(Vehicle(vehicle.name, vehicle.orientation, vehicle.length, vehicle.pos))
        return new_board

    # Description: Makes a copy of a vehicle
    # Arguments: Vehicle object to copy
    # Returns: New vehicle: obj
    def copy_vehicle(self, vehicle_to_copy):
        for vehicle in self.vehicle_list:
            if vehicle == vehicle_to_copy:
                return Vehicle(vehicle.name, vehicle.orientation, vehicle.length, vehicle.pos)


# Vehicle Class
# Description: Class to represent each vehicle on the board. Attributes include
# the vehicle's name, orientation, length of the vehicle, and starting position
# on the board (top most tile for vertical vehicles, left most tile for horizontal)
# Arguments: Name: string, orientation: string, length: int, position: tuple
class Vehicle:
    def __init__(self, name, orientation, length, pos):
        self.name = name
        self.orientation = orientation
        self.length = length
        self.pos = pos

    # Description: Overloads '==' operator for vehicle objects.
    # Compares if two vehicles are the same by comparing all class attributes
    def __eq__(self, other):
        if not isinstance(other, Vehicle):
            return NotImplemented
        return (self.name == other.name and
                self.orientation == other.orientation and
                self.length == other.length and
                self.pos == other.pos)

    # Description: Increments the length of the vehicle
    def incr_length(self):
        self.length = self.length + 1

    # Description: Checks if the vehicle is within bounds to move up
    def can_move_up(self):
        if self.pos[0] - 1 >= 0:
            return True
        else:
            return False

    # Description: Checks if the vehicle is within bounds to move left
    def can_move_left(self):
        if self.pos[1] - 1 >= 0:
            return True
        else:
            return False

    # Description: Checks if the vehicle is within bounds to move down
    def can_move_down(self):
        if self.pos[0] + self.length <= 5:
            return True
        else:
            return False

    # Description: Checks if the vehicle is within bounds to move right
    def can_move_right(self):
        if self.pos[1] + self.length <= 5:
            return True
        else:
            return False



# Function: Blocking heuristic
# Description: Calculates the blocking heuristic: f(n) = g(n) + h(n)
# Arguments: Board: object, Depth: int
# Return: 0, if the board is a goal state, else the num of blocked cars + depth + 1
def blocking_heuristic(board, depth): # h(n)
    blocked_cars = 0
    blocked_cars_dict = {}

    curr_tile = 0 # find the Y coordinate of the first tile past the X car
    while board.is_occupied((2,curr_tile)) != 'X':
        curr_tile = curr_tile + 1
    curr_tile = curr_tile + 2

    while curr_tile <= 5: # scan the row past the X car
        if board.is_occupied((2,curr_tile)) != '-': # if a letter is encountered
            if board.is_occupied((2,curr_tile)) in blocked_cars_dict:
                pass
            else:
                blocked_cars = blocked_cars + 1 # increment the num of blocked cars
                blocked_cars_dict[board.is_occupied((2,curr_tile))] = 1 # place in dict so we know we encountered the same car
        curr_tile = curr_tile + 1

    if blocked_cars == 0 and is_goal_state(board): # if there are no blocked cars, then return 0
        return 0
    else:
        return (depth + (blocked_cars + 1)) # f(n) = g(n) + h(n)


# Function: Custom Heuristic
# Description: My heuristic deals with the number of shifts it takes for blocked vehicles to move
# away from the (2,Y) row blocking the X car. For each blocked vehicle, it calculates the number
# of shifts needed move up or down away from the goal row. The minimum amount of shifts is taken
# between the two (because we want the least amount of shifts). If there are other vehicles blocking
# that vehicle from moving up or down then we also count those number of vehicles as shifts. We add
# all the shifts needed between all the blocked vehicles.
# The heuristic will then be the f(n) = depth + (num of shifts for blocked cars + 1)
# Or 0 when there is a goal state.
# This heuristic is just as good or better than the blocking heuristic because it considers the LEAST
# number of shifts for each blocked vehicle to be moved since we take the minimum between the up and
# down shifts. We only consider up and down as movements since horizontal vehicles on the 'X' row will
# result in no goal anyway.
# Arguments: Board: object, Depth: int
# Returns: 0, if board is a goal state, else the num of shifts + depth
def custom_heuristic(board, depth):
    # count the amount of moves necessary to move out of row 2
    blocked_cars = 0
    blocked_cars_dict = {}

    curr_tile = 0 # find the Y coordinate of the first tile past the X car
    while board.is_occupied((2,curr_tile)) != 'X':
        curr_tile = curr_tile + 1
    curr_tile = curr_tile + 2

    shifts = 0    # total amount of shifts for every blocked vehicle to move away from X's path to goal
    while curr_tile <= 5:
        if board.is_occupied((2,curr_tile)) != '-': # if the current tile is not empty
            curr_vehicle = board.is_occupied((2,curr_tile))
            if curr_vehicle in blocked_cars_dict:   # if the letter has already been encountered
                pass
            else:
                vehicle = board.copy_vehicle(board.get_vehicle(curr_vehicle))
                if vehicle.orientation == 'vertical': # only check vertical because horizontal vehicles result in no goal
                    head = copy.deepcopy(vehicle.pos[0]) # head of the vehicle
                    down_shifts = 0
                    # down
                    while(head + vehicle.length - 1) <= 5:
                        if vehicle.can_move_down() and board.is_occupied((head - 1, vehicle.pos[1])): # if adj tile is blocked
                            down_shifts += 2
                        elif vehicle.can_move_down() and not board.is_occupied((head - 1, vehicle.pos[1])): # if adj tile is empty
                            down_shifts += 1
                        head += 1
                        if head > 2: # if head of the vehicle is below the row, then we found the num of down shifts needed
                            break
                    # up
                    head = copy.deepcopy(vehicle.pos[0])
                    up_shifts = 0
                    while(head - 1) >= 0:
                        if vehicle.can_move_up() and board.is_occupied((head + 1, vehicle.pos[1])):
                            up_shifts += 2
                        elif vehicle.can_move_up() and not board.is_occupied((head + 1, vehicle.pos[1])):
                            up_shifts += 1
                        head += 1
                        if (head + vehicle.length - 1) > 2: # if the tail of the vehicle is above the row, we found the num of up's
                            break

                    # take the minimum shifts between the two operators because we want the least amount of shifts to do so this
                    # will result in a lower heuristic for the search to go to
                    shifts += min(down_shifts, up_shifts)
        curr_tile = curr_tile + 1

    if shifts == 0 and is_goal_state(board):
        return 0
    else:
        return (depth + shifts + 1)



# Description: Traverses through the start state and converts all the vehicle characters
# into vehicle objects to be added to the board. Having a board object that holds a list
# of vehicle objects allows for better representation and operation of each state.
# The board's heuristic is calculated at the end depending on the user's input.
# Arguments: Start state: string, Board: object, Depth: int
# Returns: None
def create_all_vehicles(start, board, depth):
    vehicle_list = []
    vehicle_dict = {}

    for x, row in enumerate(start):
        for y, tile in enumerate(row):
            if tile != '-':
                if tile in vehicle_dict:    # if the letter is already in list, incr length
                    vehicle_dict[tile].incr_length()
                else:                       # create a new obj and add to the list
                    orientation = check_orientation(start, x, y)
                    car = Vehicle(tile, orientation, 1, (x, y))
                    vehicle_list.append(car)
                    vehicle_dict[tile] = car
    for vehicle in vehicle_list:
        board.add_vehicle(vehicle)
    if board.heuristic_to_use == 0:
        board.priority = blocking_heuristic(board, depth)
    else:
        board.priority = custom_heuristic(board, depth)


# Description: Checks the orientation of a vehicle given the first encountered position.
# Arguments: X: int, Y: int
# Returns: Orientation name: string
def check_orientation(state, x, y):
    # If the next adjacent tile is of the same letter, then the vehicle is horizontal, else vertical
    if y <= 4 and state[x][y] == state[x][y + 1]:
        return "horizontal"
    else:
        return "vertical"


# Description: Check if a given board state is equal to the goal state
# Arguments: Board: object
# Returns: True if the board is a goal state, else False
def is_goal_state(board):
    if board.is_occupied((2,4)) == 'X' and board.is_occupied((2,5)) == 'X':
        return True
    else:
        return False


# Description: Traces a path from a node to the root. Adds the nodes to a list and
# reverses it so the first item in the list starts with the root. This is used to print
# out the states from start to goal.
# Arguments: Goal State: object
# Return: Path: list
def trace_path(goal_state):
    path = []
    path.append(goal_state)
    while goal_state.parent != None:
        path.append(goal_state.parent)
        goal_state = goal_state.parent
    path = path[::-1]
    return path


# Dustin Cai 5/1/2020

# testing bottleneck purposes
profile = cProfile.Profile()
#profile.runcall(rushhour, 1, ["--B---","--B---","XXB---","--AA--","------","------"])
#bprofile.runcall(rushhour, 1, ["--BC--","--BC-T","XXBC-T","--AA--","------","------"])
profile.runcall(rushhour, 1, ["AKKI--","A--I--","XXO---","--OPPP","--O--D","--QQQD"])
ps = pstats.Stats(profile)
ps.print_stats()


# To test run:
# python3 rushhour_FASTER_w_test.py



# profile.runcall(rushhour, 0, ["AKKI--","A--I--","XXO---","--OPPP","--O--D","--QQQD"])
# A K K I - D
# A - - I - D
# - - - - X X
# - - O P P P
# - - O - - -
# - - O Q Q Q
#
# Total moves:  22
# Total states explored:  437

# profile.runcall(rushhour, 1, ["AKKI--","A--I--","XXO---","--OPPP","--O--D","--QQQD"])
# A K K I - D
# A - - I - D
# - - - - X X
# - - O P P P
# - - O - - -
# - - O Q Q Q
#
# Total moves:  22
# Total states explored:  393


# rushhour.rushhour(0, ["--AABB","--CDEF","XXCDEF","--GGHH","------","------"])
# A A B B E F
# - - - - E F
# - - - - X X
# G G C D H H
# - - C D - -
# - - - - - -
#
# Total moves:  16
# Total states explored:  2212
