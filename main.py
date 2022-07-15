from maze_env import RandomMaze
from agent import AstarAgent
from a_star import PathPlanner
from visualize import Visualize
import numpy as np


if __name__ == "__main__":
    # Initialize environment
    env = RandomMaze()
    env.env_init({"maze_w":30, "maze_h":30, "maze_complexity":0.01, "density": 0.5})
    print(env.maze)

    # Visualize maze
    vis = Visualize(env)
    vis.draw_world()

    # Initialize agent
    agent = AstarAgent()
    agent_id = 1
    agent.agent_init({"mode": "simulation", "world": env, "agent_id": agent_id})
    print("Agent start position: ", (agent.start_y,agent.start_x))
    print("Agent goal position: ", (agent.goal_y, agent.goal_x))
    print(env.maze)

    # Visualize agents
    vis.draw_agents(agent_id)

    # Render initial condition
    vis.canvas.pack()
    vis.canvas.update()


    # Calculate Path using A*
    astar_planner = PathPlanner(False)

    # Plan a path.
    path_seq, full_path_length = astar_planner.a_star(env.maze, [agent.start_x, agent.start_y], [agent.goal_x, agent.goal_y])
    print('A* path sequence: ', path_seq)
    print('Full path length: ', full_path_length)

    # Convert path sequence to action
    action_seq = agent.path_to_action(path_seq)

    # Execute path and render on maze
    for step in range(full_path_length):
        action = action_seq.pop(0)
        env.env_step(agent_id, action)
        vis.canvas.update()
        vis.canvas.after(350)


    something = input('Press enter to continue...')
    # Close after 1s
    vis.canvas.after(100)