from torch import full
from maze_env import RandomMaze
from agent import AstarAgent
from astar import PathPlanner
from globalParams import *
from visualize import Visualize
import numpy as np


if __name__ == "__main__":
    # Initialize environment
    env = RandomMaze()
    env.env_init({"maze_w":50, "maze_h":50, "maze_complexity":0.01, "density": 0.5, "map_type": "warehouse"})

    # Save grid only world for CBS algorithm
    grid_only_maze = env.maze.copy()
    #print(grid_only_maze)

    # Visualize maze
    vis = Visualize(env)
    vis.draw_world()

    # Initialize agent
    starts = []
    goals = []
    for agent_id in range(1,20):
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

    hlsolver = "ASTAR"
    if hlsolver == "ASTAR":
        print("***Run ASTAR solver***")
        astar_planner = PathPlanner(False)

        # Keep looping until all agent reaches its goal
        all_reach_goal = False
        path_dict = {}
        iteration = 0
        while all_reach_goal == False:
            iteration += 1
            print("Iteration: ", iteration)
            for agent_id in env.agent_dict.keys():
                if agent_id not in path_dict:
                    curr_path_seq = []
                    curr_full_path_length = 0
                    action_is_valid = False
                    path_dict[agent_id] = {"curr_path_seq": curr_path_seq, "curr_path_len": curr_full_path_length,"action_is_valid":action_is_valid}

                agent_obj = env.agent_dict[agent_id]
                if ((agent_obj.current_x, agent_obj.current_y) == (agent_obj.goal_x, agent_obj.goal_y)):
                    agent_obj.reach_goal = True

                # First check if agent reaches its goal already
                if (agent_obj.reach_goal == True):
                    continue
                else:
                    # Only plan a path when is_valid = False
                    if (path_dict[agent_id]["action_is_valid"] == False):
                        # Plan a path with a*
                        path_seq, full_path_length = astar_planner.a_star(env.maze, [agent_obj.current_x, agent_obj.current_y], [agent_obj.goal_x, agent_obj.goal_y])
                        # Remove initial point
                        path_seq = path_seq[1:]
                        full_path_length = full_path_length - 1
                        # Update agent curr plan path if it is longer than existing plan path
                        #if full_path_length > path_dict[agent_id]["curr_path_len"]:
                        path_dict[agent_id]["curr_path_len"] = full_path_length
                        path_dict[agent_id]["curr_path_seq"] = path_seq

                    # Path may not open up at this very moment, skip this agent if this is the case
                    if (len(path_dict[agent_id]["curr_path_seq"]) == 0):
                        continue

                    # Retreive curr path plan
                    print(f'AgentID: {agent_id}, A* path sequence: {path_dict[agent_id]["curr_path_seq"]}')

                    # Convert to action plan
                    action_seq = agent_obj.path_to_action(path_dict[agent_id]["curr_path_seq"])
                    print(f'AgentID: {agent_id}, A* action sequence: {action_seq}')
                    plan_action = action_seq[0]

                    # Update neighbor status before executing the action
                    nbors = env.check_nbors(agent_obj.current_y, agent_obj.current_x)

                    if(nbors[plan_action] == FREE_SPACE):
                        path_dict[agent_id]["action_is_valid"] = True
                        action = action_seq.pop(0)
                        path_dict[agent_id]["curr_path_seq"].pop(0)
                        env.env_step(agent_id, action)
                    else:
                        # not valid action, proceed to next agent first
                        path_dict[agent_id]["action_is_valid"] = False
                        continue
            
            for agent_id in env.agent_dict.keys():
                agent_obj = env.agent_dict[agent_id]
                if agent_obj.reach_goal == False:
                    all_reach_goal = False
                    break
            else:
                all_reach_goal = True
                    

            vis.canvas.update()
            vis.canvas.after(500)

    something = input('Press enter to continue...')
    # Close after 0.1s
    vis.canvas.after(100)
