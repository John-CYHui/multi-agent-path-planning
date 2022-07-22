from globalParams import *
import numpy as np
from tkinter import *

class Visualize:
    def __init__(self, world):
        self.frame = Tk()
        self.canvas = Canvas(self.frame, width=FRAME_WIDTH, height=FRAME_HEIGHT)
        self.canvas.grid()
        self.world = world
        
        self.cell_h, self.cell_w = self.get_cell_size()
        self.agent_h, self.agent_w = self.get_agent_size()
        self.vis_cells = np.zeros_like(self.world.maze, dtype = int)

        # Register Visualize module to maze world
        world.visualize = self

        # Keep track of agent visualiztion object [agent_id : agent visualization object]
        self.agent_obj_dict = dict()
        self.agent_obj_num_dict = dict()

    def draw_world(self):
        nrows, ncols = self.world.maze.shape
        for row in range(nrows):
            for col in range(ncols):
                self.vis_cells[row][col] = self.canvas.create_rectangle(FRAME_MARGIN + self.cell_w * col, FRAME_MARGIN + self.cell_h * row, FRAME_MARGIN + self.cell_w * (col+1), FRAME_MARGIN + self.cell_h * (row+1) )
                if self.world.maze[row][col] == OBSTACLES:
                    self.canvas.itemconfig(self.vis_cells[row][col], fill='gray', width=2)

    def draw_agent(self, agent_id):
        agent_obj = self.world.agent_dict[agent_id]
        start_x, start_y= agent_obj.start_x, agent_obj.start_y
        goal_x, goal_y = agent_obj.goal_x, agent_obj.goal_y
        cell = self.world.maze[agent_obj.start_y, agent_obj.start_x]
        if(cell != FREE_SPACE and not self.world.is_blocked(start_y,start_x)):
            y1, x1, y2, x2 = self.get_agent_pos_in_maze(start_y,start_x)

            # Add agent cell to aindx_obj for tracking later
            self.agent_obj_dict[agent_id] = self.canvas.create_oval(x1, y1, x2, y2, fill=agent_obj.color)
            self.agent_obj_num_dict[agent_id] = self.canvas.create_text(x1+self.agent_w/2,y1+self.agent_h/2,text=str(agent_id))
            # Find the corresponding goal and color it
            goal_cell = self.vis_cells[goal_y, goal_x]
            self.canvas.itemconfig(goal_cell, outline=agent_obj.color, width=4)
            

    # region Helper function

    def get_cell_size(self):
        avail_h = FRAME_HEIGHT - 2 * FRAME_MARGIN
        avail_w = FRAME_WIDTH - 2 * FRAME_MARGIN
        nrows, ncols = self.world.maze.shape
        cell_h = avail_h / nrows
        cell_w = avail_w / ncols
        return (cell_h, cell_w)

    def get_agent_size(self):
        '''
        Calculate agent size when rendering the maze
        '''
        agent_h = self.cell_h - 2 * CELL_MARGIN
        agent_w = self.cell_w - 2 * CELL_MARGIN
        return (agent_h, agent_w)

    def get_pos_in_cell(self, crow, ccol):
        agent_h = self.agent_h
        agent_w = self.agent_w
        agent_y1 = FRAME_MARGIN + (crow * self.cell_h) + CELL_MARGIN
        agent_y2 = agent_y1 + agent_h
        agent_x1 = FRAME_MARGIN + (ccol * self.cell_w) + CELL_MARGIN
        agent_x2 = agent_x1 + agent_w
        return (agent_y1, agent_x1, agent_y2, agent_x2)

    def get_agent_pos_in_maze(self, crow, ccol):
        '''
        Calculate agent position when rendering the maze
        '''
        agent_h = self.agent_h
        agent_w = self.agent_w
        agent_y1 = FRAME_MARGIN + (crow * self.cell_h) + CELL_MARGIN
        agent_y2 = agent_y1 + agent_h
        agent_x1 = FRAME_MARGIN + (ccol * self.cell_w) + CELL_MARGIN
        agent_x2 = agent_x1 + agent_w
        return (agent_y1, agent_x1, agent_y2, agent_x2)

    def update_agent_pos_in_maze(self, agent_id):
        agent_obj = self.world.agent_dict[agent_id]
        current_y, current_x = agent_obj.current_y, agent_obj.current_x
        y1, x1, y2, x2 = self.get_pos_in_cell(current_y, current_x)
        self.canvas.coords(self.agent_obj_dict[agent_id], x1, y1, x2, y2)
        self.canvas.coords(self.agent_obj_num_dict[agent_id], (x1+self.agent_w/2), (y1+self.agent_h/2))
    
    # endregion Helper function