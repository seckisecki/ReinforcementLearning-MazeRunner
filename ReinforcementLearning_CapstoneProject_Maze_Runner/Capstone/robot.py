import numpy as np
from globalvariables import *

# python tester.py test_maze_01.txt
# python tester.py test_maze_02.txt
# python tester.py test_maze_03.txt

class Robot(object):
    def __init__(self, maze_dim):
        #location
        self.location = [0, 0]
        
        #direction
        self.heading = 'up'
        
        self.maze_dim = maze_dim
       
    
        self.maze_area = maze_dim ** 2.
        
        #wall locations for each maze
        self.maze_grid = np.zeros((maze_dim, maze_dim), dtype=np.int) 
        self.discovered_goal = False
        
        #grid for path
        self.path_grid = np.zeros((maze_dim, maze_dim), dtype=np.int) 
        
        #value score from goal backwards
        self.path_value = [[99 for row in range(maze_dim)] for col in range(maze_dim)] 
        
        #mapping optimal route
        self.policy_grid = [[' ' for row in range(maze_dim)] for col in range(maze_dim)] 
        
        #goal area
        self.found_goal = [maze_dim/2 - 1, maze_dim/2] or [maze_dim/2, maze_dim/2] or [maze_dim/2, maze_dim/2 - 1] or [maze_dim/2 - 1, maze_dim/2 - 1] 
        
        #heuristic grid
        self.heuristic = [[min(abs(row-maze_dim/2+1), abs(row-maze_dim/2))+min(abs(col-maze_dim/2+1), abs(col-maze_dim/2)) for row in range(maze_dim)] for col in range(maze_dim)] 
        
        #initial value
        self.backwards = 0 
        
        #step countin
        self.step_count = 0 
        
        #exploration trial or optimization trial
        self.run = 0 
        
    def next_move(self, sensors):
        
        if self.run == 0 :
             rotation, movement = self.exploration_trial(sensors)          
        elif self.run == 1:
            rotation, movement = self.optimization_trial(sensors)
        
        return rotation, movement
        
    def exploration_trial(self,sensors): 
        
        print "steps taken in exploration trial ", self.step_count, sensors
        self.step_count +=1
  
        x1 = self.location[0]
        y1 = self.location[1]
        print "robot is at location: ", self.location
        self.path_grid[x1][y1] += 1
        
        expl = 0
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                if self.path_grid[x][y] > 0:
                    expl += 1

        discovered = (expl/self.maze_area) * 100

        print "robot has explored %.2f%% of the maze.\n" % discovered
        
        # Draw the map of the maze from the sensor readings     
        num = self.wall_locations(sensors, self.backwards) 
        self.maze_grid[x1][y1] = num
        
        # robot's next move
        rotation, movement = self.determine_next_move(x1, y1, sensors)
 
        # Update backwards value
        if movement == 0:
            self.backwards = 0
        elif movement == 1:
            self.backwards = 1
            # Robot hit a dead end
        elif movement == -1 or movement == -2: 
            for move in range(2):
                #facing left, 9, 12, & 13 have a wall on right side
                if self.heading == 'l' or self.heading == 'left': 
                    if self.maze_grid[x1+move][y1] == 9 or self.maze_grid[x1+move][y1] == 12 or self.maze_grid[x1+move][y1] == 13:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                #facing right, 3, 6, & 7 have a wall on left side        
                elif self.heading == 'r' or self.heading == 'right': 
                    if self.maze_grid[x1-move][y1] == 3 or self.maze_grid[x1-move][y1] == 6 or self.maze_grid[x1-move][y1] == 7:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                #facing up, 3, 9, & 11 have a wall on bottom side        
                elif self.heading == 'u' or self.heading == 'up':
                    if self.maze_grid[x1][y1-move] == 3 or self.maze_grid[x1][y1-move] == 9 or self.maze_grid[x1][y1-move] == 11:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                #facing down, 6, 12, & 14 have a wall on top side        
                if self.heading == 'd' or self.heading == 'down': 
                    if self.maze_grid[x1][y1+move] == 6 or self.maze_grid[x1][y1+move] == 12 or self.maze_grid[x1][y1+move] == 14:
                        self.backwards = 0
                    else:
                        self.backwards = 1

        # Update robot's direction and location
        self.update_heading(rotation, movement)
        
        # new location
        x2 = self.location[0]
        y2 = self.location[1]
        
        # new location is in the goal?
        if x2 in self.found_goal and y2 in self.found_goal:
            
            if self.path_grid[x2][y2] == 0:
                print " robot found the goal position after {} steps.\n".format(self.step_count)
                print "robot mouse is exploring."
                self.discovered_goal = True
            
        elif self.discovered_goal == True and discovered >= 100:
            print "robot ended exploration trial. Optimization trial starting."
            goal = self.found_goal
            # Compute the value function and find optimal path
            self.value(goal) 
            print "\n Maze Grid \n", self.maze_grid
            print "\n Path Grid \n", self.path_grid
            print "\n Path Value \n", self.path_value
            print "\n Policy Grid \n", self.policy_grid
            
            #default settings
            rotation = 'Reset'
            movement = 'Reset'
            self.run = 1
            self.location = [0,0]
            self.heading = 'up'
            self.step_count = 0
            self.discoverd_goal = False
            
        return rotation, movement


    def optimization_trial(self,sensors):
        # Optimization Trial
    
        print "Optimization Trial Step #: ", self.step_count, sensors, self.location
        self.step_count +=1
           
        movement = 1
        
        # current location 
        x1 = self.location[0]
        y1 = self.location[1]
        
        # Rotate to the optimal path
        heading_angle = delta_degrees[self.heading]
        optimal_heading_angle = delta_degrees[self.policy_grid[x1][y1]]
        rotation = optimal_heading_angle - heading_angle
        
        # Correct for 270 degrees
        if rotation == -270:
            rotation = 90
        elif rotation == 270:
            rotation = -90
            
        # Change the angles to an index [0,1,2]    
        rot_key = rotation/90 + 1  
        direction = dir_sensors[self.heading][rot_key]  # Change direction
        
        # Move up to 3 consecutive steps
        while movement < 3: 
            location = self.policy_grid[x1][y1]
            x1 += dir_move[direction][0]
            y1 += dir_move[direction][1] 

            if self.policy_grid[x1][y1] == location:
                movement += 1      
            else: 
                break
        
        # robot is direction & location
        self.update_heading(rotation, movement)
        
        # Retrieve new location
        x2 = self.location[0]
        y2 = self.location[1]
       
        return rotation, movement

    
    def update_heading(self, rotation, movement):
        
        if rotation == -90 or rotation == 270:
            rotation = 0
        elif rotation == 0 or rotation == 360:
            rotation = 1
        elif rotation == 90 or rotation == -270:
            rotation = 2
        
        self.heading  = dir_sensors[self.heading][rotation]
        
        # update current Location 
        self.location[0] += dir_move[self.heading][0]*movement
        self.location[1] += dir_move[self.heading][1]*movement

   
    def determine_next_move(self,x1,y1,sensors):
        # determine next movement for the robot
        
        # If robot runs into a dead-end, back up.
        if sensors == [0,0,0] and self.heading == 'u' and self.maze_grid[x1][y1 - 1] == 5:
            movement = -2
            rotation = 0
            
            print "robot dead end. reverse"
        elif sensors == [0,0,0] and self.heading == 'r' and self.maze_grid[x1 - 1][y1] == 10:
            movement = -2
            rotation = 0
            print "robot dead end. reverse"
        elif sensors == [0,0,0] and self.heading == 'd' and self.maze_grid[x1][y1 + 1] == 5:
            movement = -2
            rotation = 0
            print "robot dead end. reverse"
        elif sensors == [0,0,0] and self.heading == 'l' and self.maze_grid[x1 + 1][y1] == 10:
            movement = -2
            rotation = 0
            print "robot dead end. reverse"
        elif sensors == [0,0,0]:
            movement = -1
            rotation = 0
            print "robot dead end. reverse"

        else: # implement A*
            possible_moves = [] 
            rotate = [-90, 0, 90]
            # all sensor readings
            for sensor in range(len(sensors)): 
                if sensors[sensor] > 0: 
                    sensors[sensor] = 1  
                    
                    # Move to next open space
                    x2 = x1 + dir_move[dir_sensors[self.heading][sensor]][0]   
                    y2 = y1 + dir_move[dir_sensors[self.heading][sensor]][1] 
                    # Make sure robot next space is in maze
                    if x2>=0 and x2<self.maze_dim and y2>=0 and y2<self.maze_dim:  
                        # number of times this cell has been visited
                        t2 = self.path_grid[x2][y2]  
                        # close or far to the goal area
                        h2 = self.heuristic[x2][y2]  
                        # list of possible moves
                        possible_moves.append([t2,h2,x2,y2,sensor]) 
                        
            possible_moves.sort()
            possible_moves.reverse()
            # Move to cell that is closest to the center and/or hasn't been explored.
            min_move = possible_moves.pop()  
            # Break into separate variables
            t,h,x,y,sensor = min_move 
            # Rotate to opening and move 1 space
            rotation,movement = rotate[sensor], 1  
        return rotation, movement                    
         

    def value(self, goal):
        # Initialize boolean to start while-loop
        change = True     
        # Stop if nothing has changed
        while change:
            change = False  
            
            for x in range(self.maze_dim):
                for y in range(self.maze_dim):                                       
                    if goal[0] == x and goal[1] == y:   
                        # Prevent endless loop
                        if self.path_value[x][y] > 0: 
                            # Assign 0 to goal position
                            self.path_value[x][y] = 0 
                            # Assign ~ to goal position
                            self.policy_grid[x][y] = '~' 
                            print "Goal: {}\n".format(goal)
                            
                            change = True
                    else:                        
                        # Convert the wall value into a 4-bit number 
                        wall = self.maze_grid[x][y]
                        binary = int(bin(wall)[2:])
                        four_bit = '%0*d' % (4,binary)

                        # Start at goal and increase incrementally by 1 in open spaces
                        for direction in range(len(delta)): 
                            if (four_bit[0] == '1' and direction == 0) or (four_bit[1] == '1' and direction == 1) or (four_bit[2] == '1' and direction == 2) or (four_bit[3] == '1' and direction == 3):
                            # four_bit = 0000 = left,down,right,up Direction = left,down,right,up
                                x2 = x + delta[direction][0]
                                y2 = y + delta[direction][1]
                                 # assure inside maze
                                if x2 >= 0 and x2 < self.maze_dim and y2 >= 0 and y2 < self.maze_dim:  
                                    # Add 1 to path value
                                    v2 = self.path_value[x2][y2] + 1   
                                    
                                    if v2 < self.path_value[x][y]:
                                        change = True
                                        # Update path_value with new count number
                                        self.path_value[x][y] = v2 
                                        # Add movement symbol to policy_grid (<, v, >, or ^)
                                        self.policy_grid[x][y] = delta_name[direction]  
                                        
    def wall_locations(self, sensors, backwards):
        # binary number describing surrounding walls
        # sensor reading open or closed. 1 for open. 0 for closed. 
        # Sensor reading will be [left, front, right], so if it senses a wall, set that to 0.
        for sensor in range(len(sensors)):
            if sensors[sensor] > 0:
                sensors[sensor] = 1
                
        # 1 = North, 2 = East, 4 = South, 8 = West. Each sensor will give a reading of 1 (open) or 0 (closed)       
        if self.heading == 'u' or self.heading == 'up':
            num = (sensors[0]*8) + (backwards*4) + (sensors[2]*2) + sensors[1]
        elif self.heading =='d' or self.heading == 'down':
            num = (sensors[2]*8) + (sensors[1]*4) + (sensors[0]*2) + backwards
        elif self.heading == 'l' or self.heading == 'left':
            num = (sensors[1]*8) + (sensors[0]*4) + (backwards*2) + sensors[2]
        elif self.heading == 'r' or self.heading == 'right':
            num = (backwards*8) + (sensors[2]*4) + (sensors[1]*2) + sensors[0]
        return num
