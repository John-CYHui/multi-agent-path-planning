from argparse import Action
from abstract_classes import BaseAgent
import numpy as np
import cv2
import random
from queue import Queue
from astar import PathPlanner
from globalParams import *

class AstarAgent(BaseAgent):
    def agent_init(self, agent_info={}):
        # Define agent cuurent position
        self.current_x = None
        self.current_y = None
        
        # Define agent previous position
        self.prev_x = None
        self.prev_y = None

        # Define zoneLock related variables 
        self.zoneLockQueue = Queue(maxsize=3)
        self.missionType = None


        # Define Path Planner
        self.aStarPlanner = PathPlanner(False)

        if agent_info.get("mode") == 'simulation':
            self.world = agent_info.get("world")
            self.agent_id = agent_info.get("agent_id")
            self.start_x, self.start_y, self.goal_x, self.goal_y = self.init_valid_start_goal_pos(self.world, self.agent_id)
        else:
            agent_start_goal_dict = {1:{"start_x":17, "start_y":11, "goal_x":7, "goal_y":10}, 2:{"start_x":19, "start_y":11, "goal_x":7, "goal_y":10}}
            self.world = agent_info.get("world")
            self.agent_id = agent_info.get("agent_id")
            self.start_x = agent_start_goal_dict[self.agent_id]["start_x"]
            self.start_y = agent_start_goal_dict[self.agent_id]["start_y"]
            self.goal_x = agent_start_goal_dict[self.agent_id]["goal_x"]
            self.goal_y = agent_start_goal_dict[self.agent_id]["goal_y"]
            self.world.maze[self.start_y, self.start_x] = self.agent_id

            # Set initial position as current position
            self.current_x, self.current_y = self.start_x, self.start_y
            self.reach_goal = False

            # Set initial mission type to pickup
            self.missionType = MissionType.PICKUP
        
        # Register agent to world
        self.world.agent_dict[self.agent_id] = self

        # Assign a random color to agent for visualize
        r = lambda: random.randint(0,255)
        self.color = '#{:02x}{:02x}{:02x}'.format(r(), r(), r())

    def agent_start(self, state):
        action = None
        return action

    def agent_step(self, permissionGranted):
        if permissionGranted:
            action = self.action_seq.pop(0)
        else:
            action = Actions.WAIT
        return action

    def agent_end(self, reward):
        pass

    def agent_cleanup(self):
        pass

    def agent_message(self):
        pass
    
    def agent_plan(self, state):
        """
        Construct an action plan
        """
        self.path_seq, self.full_path_length = self.aStarPlanner.a_star(state[0], state[1], state[2])
        # Remove initial point
        self.path_seq = self.path_seq[1:]
        self.full_path_length = self.full_path_length - 1
        self.action_seq = self.path_to_action(self.path_seq)

    #region Helper functions

    def init_valid_start_goal_pos(self, world, agent_id):
        list_freespace = world.freespace
        num_of_freespaces = len(list_freespace)
        start_idx, goal_idx = random.sample(range(num_of_freespaces), 2)
        start_yx, goal_yx = list_freespace[start_idx], list_freespace[goal_idx]
        world.update_freespace([start_idx, goal_idx])    
        #start_yx, goal_yx = random.sample(list_freespace, 2)
        start_y,start_x = start_yx
        goal_y, goal_x = goal_yx
        
        # Update maze with start_x, start_y occuiped
        world.maze[start_y, start_x] = agent_id

        # Set initial position as current position
        self.current_x, self.current_y = start_x, start_y

        self.reach_goal = False

        return start_x, start_y, goal_x, goal_y


    def img_fill(self, im_in, n):  # n = binary image threshold
        th, im_th = cv2.threshold(im_in, n, 1, cv2.THRESH_BINARY)

        # Copy the thresholded image.
        im_floodfill = im_th.copy()
        # Mask used to flood filling.
        # Notice the size needs to be 2 pixels than the image.
        h, w = im_th.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)

        # Floodfill from point (0, 0)
        cv2.floodFill(im_floodfill, mask, (0, 0), 255)

        # Invert floodfilled image
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)
        # print(im_floodfill_inv)
        # Combine the two images to get the foreground.
        fill_image = im_th | im_floodfill_inv

        return fill_image

    def path_to_action(self, path_seq):
        actions = []
        cy, cx = self.current_y, self.current_x
        for step in path_seq:
            ty, tx = step[0], step[1]
            if(tx - cx == 1): action = Actions.RIGHT
            elif(tx - cx == -1): action = Actions.LEFT
            elif(ty - cy == 1): action = Actions.DOWN
            elif(ty - cy == -1): action = Actions.UP
            else: action = Actions.WAIT
            # print('ToAction: ', cy, cx, ty, tx, tt, action) 
            actions.append(action)
            cy, cx = ty, tx
        return actions
    #endregion Helper functions

    def closetPossibleRoute(self):
        # First filter available freespace
        open = []
        close = []
        open.append((self.current_y, self.current_x))

        while len(open) > 0:
            current_node = open.pop(0)
            close.append(current_node)
            y, x = current_node
            if self.world.zoneLock[y,x] != self.agent_id and self.world.maze[y, x] == FREE_SPACE:
                print("Possible Node: ", y,x)
                # Try to plan a route with a star
                path_seq_to_temp_node, full_path_length_to_temp_node = self.aStarPlanner.a_star(self.world.maze, (self.current_x, self.current_y), (x,y))
                path_seq_to_temp_node.pop(0)
                full_path_length_to_temp_node -= 1
                print("Temp node seq: ", path_seq_to_temp_node)
                if full_path_length_to_temp_node > 0:
                    self.path_seq = path_seq_to_temp_node
                    self.action_seq = self.path_to_action(self.path_seq)
                break
            if not self.world.is_blocked(y+1,x) and (y+1, x) not in close:
                open.append((y+1, x))
            if not self.world.is_blocked(y-1,x) and (y-1, x) not in close:
                open.append((y-1, x))
            if not self.world.is_blocked(y,x+1) and (y, x+1) not in close:
                open.append((y, x+1))
            if not self.world.is_blocked(y,x-1) and (y, x-1) not in close:
                open.append((y, x-1))