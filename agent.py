from abstract_classes import BaseAgent
import numpy as np
import cv2
import random
import itertools
from globalParams import *

class AstarAgent(BaseAgent):
    def agent_init(self, agent_info={}):
        # Define agent cuurent position
        self.current_x = None
        self.current_y = None

        if agent_info.get("mode") == 'simulation':
            self.world = agent_info.get("world")
            maze = self.world.maze
            agent_id = agent_info.get("agent_id")
            self.start_x, self.start_y, self.goal_x, self.goal_y = self.init_valid_start_goal_pos(self.world, agent_id)
        else:
            self.start_x = agent_info.get("start_x")
            self.start_y = agent_info.get("start_y")
            self.goal_x = agent_info.get("goal_x")
            self.goal_y = agent_info.get("goal_y")
        
        # Register agent to world
        self.world.agent_dict[agent_id] = self


    def agent_start(self, state):
        pass

    def agent_step(self, reward, state):
        action = None
        return action

    def agent_end(self, reward):
        pass

    def agent_cleanup(self):
        pass

    def agent_message(self):
        pass
    

    #region Helper functions

    def init_valid_start_goal_pos(self, world, agent_id):

        # self.size_load_map = np.shape(maze)

        # # use flood-fill to ensure the connectivity of node in maze
        # #map_env = self.img_fill(maze.astype(np.uint8), 0.5)
        # map_env = maze

        # array_freespace = np.argwhere(map_env == 0)
        # num_freespace = array_freespace.shape[0]
        # array_obstacle = np.transpose(np.nonzero(map_env))
        # num_obstacle = array_obstacle.shape[0]


        # print("###### Map Size: [{},{}] - #Obstacle: {}".format(self.size_load_map[0], self.size_load_map[1],
        #                                                         num_obstacle))

        # list_freespace = []
        # list_obstacle = []

        # # transfer into list (tuple)
        # for id_FS in range(num_freespace):
        #     list_freespace.append((array_freespace[id_FS, 0], array_freespace[id_FS, 1]))

        # for id_Obs in range(num_obstacle):
        #     list_obstacle.append((array_obstacle[id_Obs, 0], array_obstacle[id_Obs, 1]))
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