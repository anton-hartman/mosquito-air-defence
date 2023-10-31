import re
import matplotlib.pyplot as plt
import numpy as np

def plot_laser_position(file_path):
    with open(file_path, 'r') as file:
        # Read set point and measurements from file
        set_point_str = file.readline().strip()
        measurements_str = file.readline().strip()

    # Parse set point
    set_point = tuple(map(int, re.findall(r"\d+", set_point_str)))

    # Parse measurements
    measurements = re.findall(r"(\d+),\((\d+), (\d+)\);", measurements_str)
    measurements = [(int(time), (int(x), int(y))) for time, x, y in measurements]

    # Adjust time to start from zero
    start_time = measurements[0][0]
    measurements = [(time - start_time, position) for time, position in measurements]

    # Extracting time, x positions, and y positions from measurements
    times, positions = zip(*measurements)
    x_positions, y_positions = zip(*positions)

    # Creating the plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Plotting X positions
    ax1.plot(times, x_positions, marker='o', linestyle='-', color='blue')
    ax1.axhline(y=set_point[0], color='r', linestyle='--')
    ax1.set_ylabel('X Position')
    ax1.legend(['Laser Position', 'Set Point'])
    ax1.grid(True)

    # Plotting Y positions
    ax2.plot(times, y_positions, marker='o', linestyle='-', color='blue')
    ax2.axhline(y=set_point[1], color='r', linestyle='--')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Y Position')
    ax2.legend(['Laser Position', 'Set Point'])
    ax2.grid(True)

    plt.suptitle('Laser Position Over Time')
    plt.show()

# Example usage (you need to replace 'path_to_your_file.txt' with the actual file path)
# plot_laser_position("/home/anton/mosquito-air-defence/laser_quali.txt")

def plot_laser_progression_from_file(file_path):
    with open(file_path, 'r') as file:
        # Read set point from the first line
        set_point_str = file.readline().strip()
        
        # Read the rest of the lines as measurement lists
        measurements_list = [line.strip() for line in file if line.strip()]
    
    # Call the previous function to generate the plot
    plot_laser_progression(set_point_str, measurements_list)


def plot_laser_progression(set_point_str, measurements_list):
    # Parse set point
    set_point = tuple(map(int, re.findall(r"\d+", set_point_str)))

    # Creating the plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    # Define a color cycle for the runs
    color_cycle = plt.cm.viridis(np.linspace(0, 1, len(measurements_list)))
    
    for idx, measurements_str in enumerate(measurements_list):
        # Parse measurements
        measurements = re.findall(r"(\d+),\((\d+), (\d+)\);", measurements_str)
        measurements = [(int(time), (int(x), int(y))) for time, x, y in measurements]

        # Adjust time to start from zero
        start_time = measurements[0][0]
        measurements = [(time - start_time, position) for time, position in measurements]

        # Extracting time, x positions, and y positions from measurements
        times, positions = zip(*measurements)
        x_positions, y_positions = zip(*positions)

        # Plotting X positions
        ax1.plot(times, x_positions, marker='o', linestyle='-', color=color_cycle[idx], markersize=3, label=f'Run {idx + 1}')
        
        # Plotting Y positions
        ax2.plot(times, y_positions, marker='o', linestyle='-', color=color_cycle[idx], markersize=3, label=f'Run {idx + 1}')
    
    # Adding set point lines and to the legend
    ax1.axhline(y=set_point[0], color='red', linestyle='--', label='Set Point')
    ax2.axhline(y=set_point[1], color='red', linestyle='--', label='Set Point')
    
    # Setting labels and legends
    ax1.set_ylabel('X Position')
    ax1.legend()
    ax1.grid(True)
    # ax1.set_xlim([0, 1500])
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Y Position')
    ax2.legend()
    ax2.grid(True)

    plt.suptitle('Laser Position Over Time')
    plt.show()

plot_laser_progression_from_file("/home/anton/mosquito-air-defence/laser_quali_4.txt")