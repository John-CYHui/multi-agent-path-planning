from argparse import Action
from maze_env import RandomMaze
from agent import AstarAgent
from astar import PathPlanner
from globalParams import *
from visualize import Visualize
import numpy as np


if __name__ == "__main__":
    # Initialize environment
    env = RandomMaze()
    env.env_init({"maze_w":20, "maze_h":15, "maze_complexity":0.01, "density": 0.5, "map_type": "self_defined"})
    env.locateDeadEnd()
    env.deadEndLockSet()

    # Visualize maze
    vis = Visualize(env)
    vis.draw_world()

    # Initialize agent
    for agent_id in range(1,3):
        agent = AstarAgent()
        agent.agent_init({"mode": "self_defined", "world": env, "agent_id": agent_id})
        print(f"Agent {agent_id} start position: {(agent.start_y,agent.start_x)}")
        print(f"Agent {agent_id} goal position: {(agent.goal_y, agent.goal_x)}")

        state = (env.maze, [agent.current_x, agent.current_y], [agent.goal_x, agent.goal_y])
        agent.agent_plan(state)
        # Visualize agents
        vis.draw_agent(agent_id)

    # Render initial condition
    vis.canvas.pack()
    vis.canvas.update()


    print("***Run ASTAR solver***")
    # Keep looping until all agent reaches its goal
    all_reach_goal = False
    count = 2
    iteration = 0
    while all_reach_goal == False:
        iteration += 1
        print("Iteration: ", iteration)
        for agent_id in env.agent_dict.keys():
            agent_obj = env.agent_dict[agent_id]

            # First check if agent reaches its goal already
            if ((agent_obj.current_x, agent_obj.current_y) == (agent_obj.goal_x, agent_obj.goal_y)):
                agent_obj.reach_goal = True
                if count > 0:
                    count -= 1
                    agent_obj.goal_y = agent_obj.start_y
                    agent_obj.goal_x = agent_obj.start_x
                    agent_obj.reach_goal = False
                    agent_obj.missionType = MissionType.IDLE
                    state = (env.maze, [agent_obj.current_x, agent_obj.current_y], [agent_obj.goal_x, agent_obj.goal_y])
                    agent_obj.agent_plan(state)


            if (agent_obj.reach_goal == True):
                continue
            else:
                if len(agent_obj.action_seq) > 0:
                    plan_action = agent_obj.action_seq[0]
                else:
                    state = (env.maze, [agent_obj.current_x, agent_obj.current_y], [agent_obj.goal_x, agent_obj.goal_y])
                    agent_obj.agent_plan(state)
                    if len(agent_obj.action_seq) == 0:
                        plan_action = Actions.WAIT
                        agent_obj.closetPossibleRoute()

                isGranted = env.isPermissionGranted(plan_action, agent_obj)
                print(f"Agent {agent_obj.agent_id} take {Actions.action_dict[plan_action]} action,  permission granted = {isGranted}")

                action = agent_obj.agent_step(isGranted)
                env.applyZoneLock(agent_obj, action)

                env.env_step(agent_id, action)

                env.releaseZoneLock(agent_obj)

        
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
