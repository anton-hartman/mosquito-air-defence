import re
import matplotlib.pyplot as plt
import numpy as np


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

# plot_laser_progression_from_file("/home/anton/mosquito-air-defence/laser_quali_4.txt")


# def plot_positions_from_file(file_path):
#     with open(file_path, 'r') as file:
#         data_str = file.read()
    
#     # Parse data
#     pattern = re.compile(r"(\d+),\((\d+), (\d+)\),\[(.*?)\];")
#     matches = pattern.findall(data_str)

#     # Adjust time to start from zero and convert positions to integers
#     start_time = int(matches[0][0])
#     data = [(int(time) - start_time, 
#              (int(laser_x), int(laser_y)),
#              [(int(track_id), (int(detected_x), int(detected_y)), (int(predicted_x), int(predicted_y))) 
#               for track_id, detected_x, detected_y, predicted_x, predicted_y in re.findall(r"\((\d+),\((\d+), (\d+)\),\((\d+), (\d+)\)\)", tracks_str)] )
#             for time, laser_x, laser_y, tracks_str in matches]
    
#     # Extracting and plotting the data
#     times = [entry[0] for entry in data]
#     laser_positions = np.array([entry[1] for entry in data])
    
#     # Creating the plot
#     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)


#     mark_size = 3
#     # Plotting laser positions
#     ax1.plot(times, laser_positions[:, 0], marker='o', linestyle='-', color='red', markersize=mark_size, label='Laser X')
#     ax2.plot(times, laser_positions[:, 1], marker='o', linestyle='-', color='red', markersize=mark_size, label='Laser Y')

#     # Define a set of distinct colors for the mosquitoes
#     distinct_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']  
    
#     # Track all mosquitoes separately
#     mosquitoes_data = {}
#     for entry in data:
#         for track_id, detected_pos, predicted_pos in entry[2]:
#             if track_id not in mosquitoes_data:
#                 mosquitoes_data[track_id] = {'times': [], 'detected_positions': [], 'predicted_positions': []}
#             mosquitoes_data[track_id]['times'].append(entry[0])
#             mosquitoes_data[track_id]['detected_positions'].append(detected_pos)
#             mosquitoes_data[track_id]['predicted_positions'].append(predicted_pos)
    

#     # Plotting mosquito positions
#     for idx, (mosquito_id, mosquito_data) in enumerate(mosquitoes_data.items()):
#         color = distinct_colors[idx % len(distinct_colors)]

#         times = mosquito_data['times']
#         detected_positions = np.array(mosquito_data['detected_positions'])
#         predicted_positions = np.array(mosquito_data['predicted_positions'])

#         # Plotting detected mosquito positions
#         ax1.plot(times, detected_positions[:, 0], marker='x', linestyle='-', color=color, markersize=mark_size, label=f'Detected Mosquito {mosquito_id} X')
#         ax2.plot(times, detected_positions[:, 1], marker='x', linestyle='-', color=color, markersize=mark_size, label=f'Detected Mosquito {mosquito_id} Y')

#         # Plotting predicted mosquito positions with dashed line
#         ax1.plot(times, predicted_positions[:, 0], marker='s', linestyle='--', color=color, markersize=mark_size, alpha=0.6, label=f'Predicted Mosquito {mosquito_id} X')
#         ax2.plot(times, predicted_positions[:, 1], marker='s', linestyle='--', color=color, markersize=mark_size, alpha=0.6, label=f'Predicted Mosquito {mosquito_id} Y')


#     # Setting labels and legends
#     ax1.set_ylabel('X Position')
#     ax1.legend()
#     ax1.grid(True)
#     ax2.set_xlabel('Time (ms)')
#     ax2.set_ylabel('Y Position')
#     ax2.legend()
#     ax2.grid(True)

#     plt.suptitle('Laser and Mosquito Positions Over Time')
#     plt.show()

def plot_positions_from_file(file_path, threshold=5, test_num=0):
    with open(file_path, 'r') as file:
        data_str = file.read()
    
    # Parse data
    pattern = re.compile(r"(\d+),\((\d+), (\d+)\),\[(.*?)\];")
    matches = pattern.findall(data_str)

    # Adjust time to start from zero and convert positions to integers
    start_time = int(matches[0][0])
    data = [(int(time) - start_time, 
             (int(laser_x), int(laser_y)),
             [(int(track_id), (int(detected_x), int(detected_y)), (int(predicted_x), int(predicted_y))) 
              for track_id, detected_x, detected_y, predicted_x, predicted_y in re.findall(r"\((\d+),\((\d+), (\d+)\),\((\d+), (\d+)\)\)", tracks_str)] )
            for time, laser_x, laser_y, tracks_str in matches]
    
    
    # Extracting and plotting the data
    times = [entry[0] for entry in data]
    laser_positions = np.array([entry[1] for entry in data])
    
    # Creating the plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    mark_size = 3
    # Plotting laser positions
    ax1.plot(times, laser_positions[:, 0], marker='o', linestyle='-', color='red', markersize=mark_size, label='Laser X')
    ax2.plot(times, laser_positions[:, 1], marker='o', linestyle='-', color='red', markersize=mark_size, label='Laser Y')

    # Define a set of distinct colors for the mosquitoes
    distinct_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    # Track all mosquitoes separately
    mosquitoes_data = {}
    for entry in data:
        for track_id, detected_pos, predicted_pos in entry[2]:
            if track_id not in mosquitoes_data:
                mosquitoes_data[track_id] = {'times': [], 'detected_positions': [], 'predicted_positions': []}
            mosquitoes_data[track_id]['times'].append(entry[0])
            mosquitoes_data[track_id]['detected_positions'].append(detected_pos)
            mosquitoes_data[track_id]['predicted_positions'].append(predicted_pos)

    # Plotting mosquito positions
    for idx, (mosquito_id, mosquito_data) in enumerate(mosquitoes_data.items()):
        color = distinct_colors[idx % len(distinct_colors)]
        times = mosquito_data['times']
        detected_positions = np.array(mosquito_data['detected_positions'])
        predicted_positions = np.array(mosquito_data['predicted_positions'])

        # Plotting detected mosquito positions
        ax1.plot(times, detected_positions[:, 0], marker='x', linestyle='-', color=color, markersize=mark_size, label=f'Detected Mosquito {mosquito_id} X')
        ax2.plot(times, detected_positions[:, 1], marker='x', linestyle='-', color=color, markersize=mark_size, label=f'Detected Mosquito {mosquito_id} Y')

        # Plotting predicted mosquito positions with dashed line
        ax1.plot(times, predicted_positions[:, 0], marker='s', linestyle='--', color=color, markersize=mark_size, alpha=0.6, label=f'Predicted Mosquito {mosquito_id} X')
        ax2.plot(times, predicted_positions[:, 1], marker='s', linestyle='--', color=color, markersize=mark_size, alpha=0.6, label=f'Predicted Mosquito {mosquito_id} Y')

    # # Finding and plotting intersections
    # for time, laser_pos in zip(times, laser_positions):
    #     for _, mosquito_data in mosquitoes_data.items():
    #         for detected_pos in mosquito_data['detected_positions']:
    #             if all(np.abs(np.array(laser_pos) - np.array(detected_pos)) < threshold):
    #                 ax1.scatter(time, detected_pos[0], color='green', s=mark_size * 10)
    #                 ax2.scatter(time, detected_pos[1], color='green', s=mark_size * 10)

    # Setting labels and legends
    ax1.set_ylabel('X Position')
    ax1.legend()
    ax1.grid(True)
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Y Position')
    ax2.legend()
    ax2.grid(True)

    plt.suptitle('Laser and Mosquito Positions Over Time' + f' (Test {test_num})')
    # plt.show()


for i in range(1, 8):
    try: 
        path = f'quali_data/main_quali//main_quali_{i} (some id switching).txt'
        plot_positions_from_file(path, test_num=i)
    except:
        print(f'Failed to plot test {i}')

plt.show()