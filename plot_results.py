import glob
import re
import matplotlib.pyplot as plt
import numpy as np
import ast
import itertools

#region Laser Quali
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
    ax1.set_ylabel('X Position (pixels)')
    ax1.legend()
    ax1.grid(True)
    # ax1.set_xlim([0, 1200])
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Y Position (pixels)')
    ax2.legend(loc='upper right')
    ax2.grid(True)
    # plt.suptitle('Laser Position Over Time')


def parse_laser_measurements(data_str):
    pattern = re.compile(r"(\d+),\((\d+), (\d+)\);")
    matches = pattern.findall(data_str)
    if not matches:
        return []

    # Adjust time to start from zero
    start_time = int(matches[0][0])
    measurements = [(int(time) - start_time, (int(x), int(y))) for time, x, y in matches]
    return measurements


def plot_all_errors_on_single_axes_with_points(files, error_threshold=10, max_time=2200, min_time=1200):
    plt.figure(figsize=(12, 6))

    for file_path in files:
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Parse set point
        set_point = np.array(tuple(map(int, re.findall(r'\d+', lines[0].strip()))), dtype=int)

        # Process each test separately
        for i, line in enumerate(lines[1:], start=1):
            measurements = parse_laser_measurements(line)
            if not measurements:
                print(f"File: {file_path}, Start Point: {measurements[0][1] if measurements else 'N/A'}, Set Point: {set_point}: No valid measurements found.")
                continue

            # Get initial laser position (start point)
            initial_laser_position = measurements[0][1]

            # Filter measurements based on time
            measurements = [(time, position) for time, position in measurements if min_time <= time <= max_time]

            # Calculate Euclidean distances (errors) to set point at each time step
            times = [time for time, _ in measurements]
            errors = [np.linalg.norm(np.array(position) - set_point) for _, position in measurements]

            # Filter data to only include points where error is less than the threshold
            near_setpoint_times = [time for time, error in zip(times, errors) if error < error_threshold]
            near_setpoint_errors = [error for error in errors if error < error_threshold]

            if not near_setpoint_times:
                print(f"File: {file_path}, Start Point: {initial_laser_position}, Set Point: {set_point}: No data points found near the set point (Error < {error_threshold}).")
                continue

            # Plotting error near set point
            plt.plot(near_setpoint_times, near_setpoint_errors, marker='o', linestyle='-', markersize=3, label=f'{tuple(initial_laser_position)} → {tuple(set_point)}')

    plt.xlabel('Time (ms)')
    plt.ylabel('Error (Euclidean Distance)')
    # plt.title(f'Error vs. Time (Near Set Point, Error < {error_threshold}, {min_time}ms <= Time <= {max_time}ms)')
    plt.grid(True)
    plt.legend(loc='right')

# file_paths = glob.glob("quali_data/laser_quali/laser_quali*")
# plot_all_errors_on_single_axes_with_points(file_paths, error_threshold=10, max_time=2200, min_time=1200)
# for path in file_paths:
    # plot_laser_progression_from_file(path)
# plt.show()

#endregion Laser Quali


#region Main Quali
def calculate_frame_rate_metrics(times):
    # Calculate frame durations (time between consecutive frames)
    frame_durations = np.diff(times) / 1000.0  # Convert from milliseconds to seconds

    # Calculate average frame rate
    average_frame_rate = 1 / np.mean(frame_durations)

    # Calculate frame rate variability (standard deviation of frame durations)
    frame_rate_variability = 1 / np.std(frame_durations)

    return average_frame_rate, frame_rate_variability

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

    # Calculate frame rate metrics
    average_frame_rate, frame_rate_variability = calculate_frame_rate_metrics(times)

    # Print the frame rate metrics
    print("Average Frame Rate: {:.2f} Hz".format(average_frame_rate))
    print("Frame Rate Variability: {:.2f} Hz".format(frame_rate_variability))
    
    # Creating the plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    mark_size = 3
    # Plotting laser positions
    ax1.plot(times, laser_positions[:, 0], marker='o', linestyle='-', color='red', markersize=mark_size, label='Laser')
    ax2.plot(times, laser_positions[:, 1], marker='o', linestyle='-', color='red', markersize=mark_size, label='Laser')

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

    id = 1
    # Plotting mosquito positions
    for idx, (mosquito_id, mosquito_data) in enumerate(mosquitoes_data.items()):
        color = distinct_colors[idx % len(distinct_colors)]
        times = mosquito_data['times']
        detected_positions = np.array(mosquito_data['detected_positions'])
        predicted_positions = np.array(mosquito_data['predicted_positions'])

        # Plotting detected mosquito positions
        ax1.plot(times, detected_positions[:, 0], marker='x', linestyle='-', color=color, markersize=mark_size, label=f'Detected Mosquito {id}')# {mosquito_id}')
        ax2.plot(times, detected_positions[:, 1], marker='x', linestyle='-', color=color, markersize=mark_size, label=f'Detected Mosquito {id}')# {mosquito_id}')

        # Plotting predicted mosquito positions with dashed line
        ax1.plot(times, predicted_positions[:, 0], marker='s', linestyle='--', color=color, markersize=mark_size, alpha=0.6, label=f'Predicted Mosquito {id}')# {mosquito_id}')
        ax2.plot(times, predicted_positions[:, 1], marker='s', linestyle='--', color=color, markersize=mark_size, alpha=0.6, label=f'Predicted Mosquito {id}')# {mosquito_id}')
        id += 1

    # # Finding and plotting intersections
    # for time, laser_pos in zip(times, laser_positions):
    #     for _, mosquito_data in mosquitoes_data.items():
    #         for detected_pos in mosquito_data['detected_positions']:
    #             if all(np.abs(np.array(laser_pos) - np.array(detected_pos)) < threshold):
    #                 ax1.scatter(time, detected_pos[0], color='green', s=mark_size * 10)
    #                 ax2.scatter(time, detected_pos[1], color='green', s=mark_size * 10)

    # Setting labels and legends
    ax1.set_ylabel('X Position (pixels)')
    ax1.legend()
    ax1.grid(True)
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Y Position (pixels)')
    ax2.legend()
    ax2.grid(True)

    # plt.suptitle('Laser and Mosquito Positions Over Time' + f' (Test {test_num})')


# file_paths = glob.glob('quali_data/main_quali/main_quali*')
# for path in file_paths:
#     plot_positions_from_file(path)
# plt.show()

#endregion Main Quali


#region Tracking Quali
def plot_tracking_data_from_file(file_path):
    with open(file_path, 'r') as f:
        tests_data = f.read().strip().split('\n')
    
    for test_num, data_str in enumerate(tests_data, start=1):
        # Splitting the data string into individual data entries
        entries = data_str.split(';')[:-1]  # The last element is empty due to the trailing ';', so we ignore it
        
        # Parsing and organizing the data
        tracking_data = []
        for entry in entries:
            entry_data = ast.literal_eval(entry)
            time = entry_data[0]
            track_data = entry_data[1:]
            tracks = {track[0]: {'detected': track[1], 'predicted': track[2]} for track in track_data}
            tracking_data.append({'time': time, 'tracks': tracks})
        
        # Setting up the plot
        plt.figure(figsize=(10, 6))
        plt.title(f'Test {test_num}: Tracking Data - Detected vs Predicted')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        
        # Colors and line styles for different tracks
        colors = itertools.cycle(['r', 'g', 'b', 'c', 'm', 'y', 'k'])
        linestyles = {'detected': '-', 'predicted': '--'}
        
        # Plotting the data
        for track_id in tracking_data[0]['tracks'].keys():
            color = next(colors)
            x_detected, y_detected = [], []
            x_predicted, y_predicted = [], []
            
            for entry in tracking_data:
                track = entry['tracks'].get(track_id, None)
                if track is None:
                    continue
                
                # Detected points
                x_detected.append(track['detected'][0])
                y_detected.append(track['detected'][1])
                
                # Predicted points
                x_predicted.append(track['predicted'][0])
                y_predicted.append(track['predicted'][1])
            
            # Plotting detected points
            if x_detected and y_detected:
                plt.plot(x_detected, y_detected, linestyle=linestyles['detected'], color=color, label=f'Track {track_id} (Detected)')
                plt.scatter([x_detected[0]], [y_detected[0]], marker='o', color=color)
            
            # Plotting predicted points
            if x_predicted and y_predicted:
                plt.plot(x_predicted, y_predicted, linestyle=linestyles['predicted'], color=color, label=f'Track {track_id} (Predicted)')
                plt.scatter([x_predicted[0]], [y_predicted[0]], marker='o', color=color)
        
        # Adding a legend
        plt.legend()
        plt.grid(True)

plot_tracking_data_from_file('tracking_quali.txt')
plt.show()
#endregion Tracking Quali
