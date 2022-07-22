from maze_env import RandomMaze
from agent import AstarAgent
from astar import PathPlanner
from solver.cbs import CBSSolver
from solver.icbs import ICBS_Solver
from visualize import Visualize
import numpy as np


if __name__ == "__main__":
    # Initialize environment
    env = RandomMaze()
    env.env_init({"maze_w":20, "maze_h":20, "maze_complexity":0.01, "density": 0.5, "map_type": "warehouse"})

    # Save grid only world for CBS algorithm
    grid_only_maze = env.maze.copy()
    #print(grid_only_maze)

    # Visualize maze
    vis = Visualize(env)
    vis.draw_world()

    # Initialize agent
    starts = []
    goals = []
    for agent_id in range(1,10):
        agent = AstarAgent()
        agent.agent_init({"mode": "simulation", "world": env, "agent_id": agent_id})
        print("Agent start position: ", (agent.start_y,agent.start_x))
        print("Agent goal position: ", (agent.goal_y, agent.goal_x))
        starts.append((agent.start_y,agent.start_x))
        goals.append((agent.goal_y, agent.goal_x))
        # Visualize agents
        vis.draw_agent(agent_id)
    #print(env.maze)


    # Render initial condition
    vis.canvas.pack()
    vis.canvas.update()

    hlsolver = "CBS"
    if hlsolver == "CBS":
        print("***Run CBS solver***")
        cbs = CBSSolver(grid_only_maze, starts, goals)
        disjoint = True
        llsolver = "a_star"
        solution = cbs.find_solution(disjoint, llsolver)

    elif hlsolver == "ICBS":
        print("***Run Improved CBS solver***")
        cbs = ICBS_Solver(grid_only_maze, starts, goals)
        disjoint = True
        llsolver = "a_star"
        solution = cbs.find_solution(disjoint, llsolver)

    if solution is not None:
        # print(solution)
        path_sequnces= solution[0]
        if path_sequnces is None:
            raise BaseException('No solutions')  
    else:
        raise BaseException('No solutions')

    # Convert path sequence to action sequence
    action_seq_dict = dict()
    # Calculate max path length for rendering later
    max_path_len = 0
    for idx, path_seq in enumerate(path_sequnces):
        path_len =len(path_seq)
        max_path_len = path_len if (path_len > max_path_len) else max_path_len

        agent_id = idx + 1
        agent_obj = env.agent_dict[agent_id]
        # Convert path sequence to action
        action_seq = agent_obj.path_to_action(path_seq)
        action_seq_dict[agent_id] = action_seq

    # Execute action sequence for each agent and render on maze
    for step in range(max_path_len):
        for agent_id in env.agent_dict.keys():
                if(action_seq_dict[agent_id]):
                    action_seq = action_seq_dict[agent_id]
                    action = action_seq.pop(0)
                    status = env.env_step(agent_id, action)

        vis.canvas.update()
        vis.canvas.after(350)



    something = input('Press enter to continue...')
    # Close after 0.1s
    vis.canvas.after(100)
