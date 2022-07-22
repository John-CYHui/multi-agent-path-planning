from abstract_classes import BaseEnvironment
import numpy as np
import cv2
from globalParams import *

class RandomMaze(BaseEnvironment):
    def env_init(self, env_info = {}):
        # Create maze
        self.w = env_info.get("maze_w", 10)
        self.h = env_info.get("maze_h", 10)
        complexity = env_info.get("maze_complexity", 0.01)
        density = env_info.get("density", 0.1)
        maze_type = env_info.get("map_type", "maze")
        path_size = env_info.get("path_size", 2)
        central_path_size = env_info.get("central_path_size", 2)

        self.maze = self._mapGen(self.w, self.h, complexity, density, maze_type, path_size, central_path_size)

        # Maintain list of freespaces and obstacles
        self.freespace, self.obstacle = self.get_freespace(self.maze)

        # Define agent dictionary -> {agent_id: agent obj}
        self.agent_dict = dict()

        # Define visualize parameter for registering in visualize module
        self.visualize = None

    def env_start(self):
        pass

    def env_step(self, agent_id, action):
        # Check whether agent is registered in agent dict
        if(agent_id in self.agent_dict):
            agent_obj = self.agent_dict[agent_id]
            current_x, current_y = agent_obj.current_x, agent_obj.current_y
            goal_x, goal_y = agent_obj.goal_x, agent_obj.goal_y
        else:
            raise Exception('Agent id: ' + str(agent_id) + ' does not exist!')

        # Remember original location before doing anything
        original_y, original_x = current_y, current_x
        
        #update neighbors status
        #nbors = self.check_nbors(current_y, current_x)

        #if(nbors[action] == FREE_SPACE):
        current_y += int(action == Actions.DOWN) - int(action == Actions.UP)
        current_x += int(action == Actions.RIGHT) - int(action == Actions.LEFT)
        agent_obj.current_y, agent_obj.current_x = current_y, current_x
        self.maze[original_y, original_x] = 0
        self.maze[current_y, current_x] = agent_id
        # Check if maze world is registered to visualize module
        if(self.visualize): 
            self.visualize.update_agent_pos_in_maze(agent_id)

        elif(action == Actions.WAIT):
            return -1
        #else:
        # TODO the order of action between agents matter
            #raise Exception('Cell is not unoccupied! : (' + str(current_y) + ',' + str(current_x) + ') --> ' + str(action) )

        return 0 if (current_x, current_y)  == (goal_x, goal_y) else -1
        
    def env_cleanup(self):
        pass


    def _mapGen(self, width, height, complexity, density, map_type, path_size, central_path_size):
        world_size = (height, width)
        if map_type == 'maze':
            # Only odd shapes
            
            # number of components
            complexity = int(complexity * (5 * (world_size[0] + world_size[1])))
            # size of components
            density = int(density * ((world_size[0] // 2) * (world_size[1] // 2)))
            # Build actual maze
            maze = np.zeros(world_size, dtype=np.int64)

            # Make aisles
            for i in range(density):
                # pick a random position
                x, y = np.random.randint(0, world_size[1] // 2) * 2, np.random.randint(0, world_size[0] // 2) * 2

                maze[y, x] = 1
                for j in range(complexity):
                    neighbours = []
                    if x > 1:             neighbours.append((y, x - 2))
                    if x < world_size[1] - 2:  neighbours.append((y, x + 2))
                    if y > 1:             neighbours.append((y - 2, x))
                    if y < world_size[0] - 2:  neighbours.append((y + 2, x))
                    if len(neighbours):
                        y_, x_ = neighbours[np.random.randint(0, len(neighbours) - 1)]
                        if maze[y_, x_] == 0:
                            maze[y_, x_] = 1
                            maze[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                            x, y = x_, y_
            maze *= OBSTACLES
            maze = self.img_fill(maze.astype(np.uint8), 0.5)
            return maze

        elif map_type == 'warehouse':
            # Build actual maze
            maze = np.ones(world_size, dtype=np.int64)
            if path_size == 0: # no regulatoin on path size
                maze[0] = 0
                maze[width-1] = 0
                maze[:, 0] = 0
                maze[:, height-1] = 0
                total_area = width * height
                current_density = np.count_nonzero(maze)/total_area
                while current_density > density:
                    if np.random.randint(2, size=1) == 0: # delete row
                        maze[np.random.randint(height, size=1)] = 0
                    else: # delete column
                        maze[:, np.random.randint(width, size=1)] = 0
                    current_density = np.count_nonzero(maze)/total_area
                return maze
            
            else:
                maze[:central_path_size] = 0
                maze[height-central_path_size:] = 0
                maze[:, :central_path_size] = 0
                maze[:, width-central_path_size:] = 0

                occupied_row = np.zeros(height)
                occupied_col = np.zeros(width)
                occupied_row[:central_path_size + 1] = 1
                occupied_row[height-central_path_size-1:] = 1
                occupied_col[:central_path_size + 1] = 1
                occupied_col[width-central_path_size-1:] = 1

                center_x = (
                int((width - central_path_size) / 2), int((width - central_path_size) / 2) + central_path_size)
                center_y = (
                    int((height - central_path_size) / 2), int((height - central_path_size) / 2) + central_path_size)

                maze[center_y[0]:center_y[1]] = 0
                maze[:, center_x[0]:center_x[1]] = 0

                occupied_row[center_y[0] - 1:center_y[1] + 1] = 1
                occupied_col[center_x[0] - 1:center_x[1] + 1] = 1


                total_area = width * height
                current_density = np.count_nonzero(maze) / total_area
                fail_count = 0
                while current_density > density:
                    fail_count+=1
                    if fail_count>100:
                        print('Timeout to find solution. Please check whether the path '
                              'size is too strict. Density at {}, target is {}'.format(current_density, density))
                        break
                    if np.random.randint(2, size=1) == 0: # delete row
                        # print('row')
                        chosen = int(np.random.randint(height, size=1))
                        # print(chosen, occupied_row[chosen: chosen+path_size])
                        if np.count_nonzero(occupied_row[chosen: chosen+path_size]) == 0:
                            occupied_row[chosen - 1: chosen + path_size + 1] = 1
                            maze[chosen: chosen + path_size] = 0
                            fail_count = 0
                    else: # delete column
                        # print('col')
                        chosen = int(np.random.randint(width, size=1))
                        # print(chosen, occupied_col[chosen: chosen + path_size])
                        if np.count_nonzero(occupied_col[chosen: chosen+path_size]) == 0:
                            occupied_col[chosen - 1: chosen + path_size + 1] = 1
                            maze[:, chosen: chosen + path_size] = 0
                            fail_count = 0
                    current_density = np.count_nonzero(maze)/total_area

                maze *= OBSTACLES
                return maze


    # region Helper functions

    def is_blocked(self, y, x):
        if not self.is_outbound(y, x): return True
        if(self.maze[y, x] == OBSTACLES): return True
        return False
    
    def is_outbound(self, y, x):
        # Check if query position is out bound
        if x < 0 or x > self.w - 1 or y < 0 or y > self.h - 1:
            return False
        else:
            return True

    def check_nbors(self, y, x):
        '''
        Return the status of neighbors of given cell
        return: array [ RIGHT, UP, LEFT, DOWN, WAIT ]
        '''
        nbors = np.ones(5, dtype = int ) * OBSTACLES

        if(x > 0):
            nbors[Actions.LEFT] = self.maze[y,x-1]
        if(x < self.w - 1):
            nbors[Actions.RIGHT] = self.maze[y, x+1]
        if(y > 0):
            nbors[Actions.UP] = self.maze[y-1, x]
        if(y < self.h - 1):
            nbors[Actions.DOWN] = self.maze[y+1, x]

        nbors[Actions.WAIT] = self.maze[y, x]
        return nbors

    def img_fill(self, im_in, n):  # n = binary image threshold
        th, im_th = cv2.threshold(im_in, n, 1, cv2.THRESH_BINARY)

        # Copy the thresholded image.
        im_floodfill = im_th.copy()
        # Mask used to flood filling.
        # Notice the size needs to be 2 pixels than the image.
        h, w = im_th.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)

        # Floodfill from point (0, 0)
        cv2.floodFill(im_floodfill, mask, (int(w/2), int(h/2)), 255)

        # Invert floodfilled image
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)
        # print(im_floodfill_inv)
        # Combine the two images to get the foreground.
        fill_image = im_th | im_floodfill_inv
        fill_image = np.where(fill_image == 255, -99, 0)
        return fill_image
    
    def get_freespace(self, maze):
        self.size_load_map = np.shape(maze)
        map_env = maze
        array_freespace = np.argwhere(map_env == 0)
        num_freespace = array_freespace.shape[0]
        array_obstacle = np.transpose(np.nonzero(map_env))
        num_obstacle = array_obstacle.shape[0]


        print("###### Map Size: [{},{}] - #Obstacle: {}".format(self.size_load_map[0], self.size_load_map[1],
                                                                num_obstacle))

        list_freespace = []
        list_obstacle = []

        # transfer into list (tuple)
        for id_FS in range(num_freespace):
            list_freespace.append((array_freespace[id_FS, 0], array_freespace[id_FS, 1]))

        for id_Obs in range(num_obstacle):
            list_obstacle.append((array_obstacle[id_Obs, 0], array_obstacle[id_Obs, 1]))
    
        return list_freespace, list_obstacle

    def update_freespace(self, idx_ls):
        for element in sorted(idx_ls, reverse = True):
            del self.freespace[element]

    # endregion Helper functions