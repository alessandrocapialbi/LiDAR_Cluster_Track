import numpy as np
from scipy.optimize import linear_sum_assignment
from kalman_filter import KalmanFilter

# Initialize a dictionary to store Kalman filters for each vehicle
kalman_filters = {}

def compute_velocity(curr_position, predicted_position, delta_time):
    """
    Calculate the velocity based on the previous and current positions and the elapsed time.
    """
    vx = (predicted_position[0] - curr_position[0]) / delta_time
    vy = (predicted_position[1] - curr_position[1]) / delta_time
    return np.array([vx, vy])

def compute_acceleration(curr_velocity, predicted_velocity, delta_time):
    """
    Calculate the acceleration based on the previous and current velocities and the elapsed time.
    """
    ax = (predicted_velocity[0] - curr_velocity[0]) / delta_time
    ay = (predicted_velocity[1] - curr_velocity[1]) / delta_time
    return np.array([ax, ay])

def compute_distance_matrix(prev_boxes, curr_boxes):
    distance_matrix = np.zeros((len(prev_boxes), len(curr_boxes)))

    for i, prev in enumerate(prev_boxes):
        for j, curr in enumerate(curr_boxes):
            distance_matrix[i, j] = np.linalg.norm(np.array(prev) - np.array(curr))

    return distance_matrix

def track_vehicles(prev_centroids, curr_centroids, prev_ids, curr_ids, threshold, sensor_frequency):
    global kalman_filters

    delta_time = 1 / sensor_frequency  # Time step in seconds

    # Initialize Kalman filters for vehicles in prev_ids if not already initialized
    for vehicle_id in prev_ids:
        if vehicle_id not in kalman_filters:
            kf = KalmanFilter(delta_time)
            # Initialize with the previous position and zero velocity/acceleration
            kf.X[:3] = prev_centroids[prev_ids.index(vehicle_id)]  # Initial position [x, y, z]
            kf.X[3:5] = np.zeros(2)  # Initial velocity [vx, vy]
            kf.X[5:] = np.zeros(2)  # Initial acceleration [ax, ay]
            kalman_filters[vehicle_id] = kf

    # Predict the next position of each vehicle using Kalman Filter
    predicted_centroids = []
    for i, vehicle_id in enumerate(prev_ids):
        if vehicle_id in kalman_filters:
            kf = kalman_filters[vehicle_id]
            kf.predict()  # Predict the next state
            predicted_centroids.append(kf.get_state()[:3])  # Append the predicted position [x, y, z]

    # Compute distance matrix between predicted centroids and current centroids
    distance_matrix = compute_distance_matrix(predicted_centroids, curr_centroids)

    # Solve the linear assignment problem
    row_ind, col_ind = linear_sum_assignment(distance_matrix)

    matches = []
    unmatched_prev = set(range(len(prev_centroids)))
    unmatched_curr = set(range(len(curr_centroids)))

    # Perform matching based on the assignment and threshold
    for r, c in zip(row_ind, col_ind):
        if distance_matrix[r, c] < threshold:
            matches.append((prev_ids[r], curr_ids[c]))
            unmatched_prev.discard(r)
            unmatched_curr.discard(c)

            # Update the Kalman filter for the matched vehicle
            kf = kalman_filters[prev_ids[r]]
            prev_pos = prev_centroids[r]
            curr_pos = curr_centroids[c]
            prev_velocity = kf.X[3:5]
            curr_velocity = compute_velocity(prev_pos, curr_pos, delta_time)
            acceleration = compute_acceleration(prev_velocity, curr_velocity, delta_time)

            # Update the Kalman filter with the actual measurement
            kf.update(curr_pos)
            # Update velocity and acceleration in the Kalman filter's state vector
            kf.X[3:5] = curr_velocity
            kf.X[5:] = acceleration

    # Handle unmatched vehicles
    exited_vehicles = [prev_ids[i] for i in unmatched_prev]
    entered_vehicles = [curr_ids[i] for i in unmatched_curr]

    # Initialize Kalman filters for new vehicles
    for vehicle_id in entered_vehicles:
        curr_pos = curr_centroids[curr_ids.index(vehicle_id)]
        kf = KalmanFilter(delta_time)
        kf.X[:3] = curr_pos  # Set the initial position [x, y, z]
        kf.X[3:5] = np.zeros(2)  # Initial velocity [vx, vy]
        kf.X[5:] = np.zeros(2)  # Initial acceleration [ax, ay]
        kalman_filters[vehicle_id] = kf

    # Remove Kalman filters for exited vehicles
    for vehicle_id in exited_vehicles:
        if vehicle_id in kalman_filters:
            del kalman_filters[vehicle_id]

    return matches, exited_vehicles, entered_vehicles, predicted_centroids


def calculate_threshold(df, sensor_frequency, percentage_margin):
    vx = df['vx']
    vy = df['vy']
    v_max = (np.sqrt(pow(vx, 2) + pow(vy, 2))).max()
    threshold = v_max * (1 / sensor_frequency)
    return threshold + threshold * (percentage_margin / 100)
