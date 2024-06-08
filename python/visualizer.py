import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import matplotlib.widgets as widgets

def load_map(map_file_path):
    with open(map_file_path, 'r') as file:
        lines = file.readlines()
    
    # Parse the map dimensions and data
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    map_data = lines[4:4+height]
    
    # Create the map grid
    grid = np.zeros((height, width))
    for i in range(height):
        for j in range(width):
            if map_data[i][j] == '@':
                grid[i][j] = 1
    
    return grid

def load_paths(all_txt_path):
    with open(all_txt_path, 'r') as file:
        lines = file.readlines()
    
    # Parse agent paths
    agent_paths = []
    current_agent_path = []
    for line in lines:
        line = line.strip()
        if line.startswith("Agent"):
            if current_agent_path:
                agent_paths.append(current_agent_path)
                current_agent_path = []
        elif line and line[0].isdigit():
            x, y = map(int, line.split())
            current_agent_path.append((x, y))
    if current_agent_path:
        agent_paths.append(current_agent_path)
    
    return agent_paths

def animate(i):
    global current_timestep
    if i is not None:
        current_timestep = i
    for j, path in enumerate(agent_paths):
        if current_timestep < len(path):
            x, y = path[current_timestep]
            agents[j].set_data(x, y)
    timestep_text.set_text(f'Timestep: {current_timestep}/{max_timesteps}')
    return agents

def start_animation(event):
    global anim
    anim = FuncAnimation(fig, animate, frames=max_timesteps, interval=200, blit=True, repeat=False)
    plt.draw()

def stop_animation(event):
    global anim
    if anim:
        anim.event_source.stop()

def reset_animation(event):
    global anim, current_timestep
    anim = None
    current_timestep = 0
    for agent, path in zip(agents, agent_paths):
        if path:
            x, y = path[0]
            agent.set_data(x, y)
    timestep_text.set_text(f'Timestep: {current_timestep}/{max_timesteps}')
    plt.draw()

def next_step(event):
    global current_timestep
    if current_timestep < max_timesteps - 1:
        current_timestep += 1
        animate(current_timestep)
        plt.draw()

def prev_step(event):
    global current_timestep
    if current_timestep > 0:
        current_timestep -= 1
        animate(current_timestep)
        plt.draw()

if __name__ == "__main__":
    # Set the file paths
    map_file_path = "/mnt/Topics/Learning/MAPF/LocalHeuristics/map/random-32-32-10.map"
    all_txt_path = "/mnt/Topics/Learning/MAPF/LocalHeuristics/astar-algorithm-cpp/build/all.txt"
    
    # Load map and paths
    grid = load_map(map_file_path)
    agent_paths = load_paths(all_txt_path)
    max_timesteps = max(len(path) for path in agent_paths)
    
    # Create plot
    fig, ax = plt.subplots()
    ax.imshow(grid, cmap='binary')

    colors = ['r', 'g', 'b', 'y', 'm', 'c']  # Colors for different agents
    agents = []

    for i, path in enumerate(agent_paths):
        agent, = ax.plot([], [], marker='o', color=colors[i % len(colors)], label=f'Agent {i}')
        agents.append(agent)

    # Create a separate figure for buttons and timestep display
    fig_buttons, ax_buttons = plt.subplots()
    plt.axis('off')

    # Add buttons for controlling animation
    ax_play = plt.axes([0.1, 0.5, 0.1, 0.075])
    ax_stop = plt.axes([0.21, 0.5, 0.1, 0.075])
    ax_reset = plt.axes([0.32, 0.5, 0.1, 0.075])
    ax_next = plt.axes([0.43, 0.5, 0.1, 0.075])
    ax_prev = plt.axes([0.54, 0.5, 0.1, 0.075])

    btn_play = widgets.Button(ax_play, 'Play')
    btn_stop = widgets.Button(ax_stop, 'Stop')
    btn_reset = widgets.Button(ax_reset, 'Reset')
    btn_next = widgets.Button(ax_next, 'Next')
    btn_prev = widgets.Button(ax_prev, 'Prev')

    btn_play.on_clicked(start_animation)
    btn_stop.on_clicked(stop_animation)
    btn_reset.on_clicked(reset_animation)
    btn_next.on_clicked(next_step)
    btn_prev.on_clicked(prev_step)

    # Add timestep display
    current_timestep = 0
    timestep_text = ax_buttons.text(0.05, 0.9, f'Timestep: {current_timestep}/{max_timesteps}', fontsize=12)

    anim = None

    plt.show()
