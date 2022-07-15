from abstract_classes import BaseEnvironment
import numpy as np
from globalParams import *

class RandomMaze(BaseEnvironment):
    def env_init(self, env_info = {}):
        # Create maze
        self.w = env_info.get("maze_w", 10)
        self.h = env_info.get("maze_h", 10)
        complexity = env_info.get("maze_complexity", 0.01)
        density = env_info.get("density", 0.1)
        self.maze = self._mapGen(self.w, self.h, complexity, density)

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
        nbors = self.check_nbors(current_y, current_x)

        if(nbors[action] == FREE_SPACE):
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
        else:
            raise Exception('Cell is not unoccupied! : (' + str(current_y) + ',' + str(current_x) + ') --> ' + str(action) )

        return 0 if (current_x, current_y)  == (goal_x, goal_y) else -1
        
    def env_cleanup(self):
        pass


    def _mapGen(self, width=10, height=10, complexity=0.01, density=0.1):
        # Only odd shapes
        world_size = (height, width)
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
    
    # endregion Helper functions