import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance

from kalman_filter import KalmanFilter

# Global dictionaries to store Kalman filters and MSE values
kalman_filters = {}
mse_values = {}


def compute_velocity(curr_position, predicted_position, delta_time):
    """
    Compute velocity as the difference between two positions over time.

    Args:
        curr_position (np.ndarray): Current [x, y] position.
        predicted_position (np.ndarray): Next predicted [x, y] position.
        delta_time (float): Time interval between the two positions.

    Returns:
        np.ndarray: Velocity vector [vx, vy].
    """
    vx = (predicted_position[0] - curr_position[0]) / delta_time
    vy = (predicted_position[1] - curr_position[1]) / delta_time
    return np.array([vx, vy])


def compute_acceleration(curr_velocity, predicted_velocity, delta_time):
    """
    Compute acceleration as the difference between two velocities over time.

    Args:
        curr_velocity (np.ndarray): Current velocity [vx, vy].
        predicted_velocity (np.ndarray): Next velocity [vx, vy].
        delta_time (float): Time interval between the two velocities.

    Returns:
        np.ndarray: Acceleration vector [ax, ay].
    """
    ax = (predicted_velocity[0] - curr_velocity[0]) / delta_time
    ay = (predicted_velocity[1] - curr_velocity[1]) / delta_time
    return np.array([ax, ay])


def compute_distance_matrix(prev_boxes, curr_boxes):
    """
    Compute the pairwise Euclidean distance matrix between previous and current positions.

    Args:
        prev_boxes (list): List of previous centroid coordinates.
        curr_boxes (list): List of current centroid coordinates.

    Returns:
        np.ndarray: Distance matrix of shape (len(prev_boxes), len(curr_boxes)).
    """
    distance_matrix = np.zeros((len(prev_boxes), len(curr_boxes)))
    for i, prev in enumerate(prev_boxes):
        for j, curr in enumerate(curr_boxes):
            distance_matrix[i, j] = np.linalg.norm(np.array(prev) - np.array(curr))
    return distance_matrix


def track_vehicles(prev_centroids, curr_centroids, prev_ids, curr_ids, threshold, sensor_frequency):
    """
    Match, predict, and update tracked vehicles using Kalman Filters.

    Args:
        prev_centroids (list): List of vehicle centroids at time t-1.
        curr_centroids (list): List of vehicle centroids at time t.
        prev_ids (list): List of IDs corresponding to prev_centroids.
        curr_ids (list): List of IDs corresponding to curr_centroids.
        threshold (float): Maximum distance for a valid match.
        sensor_frequency (float): Frequency of sensor readings (Hz).

    Returns:
        tuple: (matches, exited_vehicles, entered_vehicles, predicted_centroids)
    """
    global kalman_filters
    delta_time = 1 / sensor_frequency

    # Initialize Kalman filters for previous vehicles if missing
    for vehicle_id in prev_ids:
        if vehicle_id not in kalman_filters:
            kf = KalmanFilter(delta_time)
            kf.X[:3] = prev_centroids[prev_ids.index(vehicle_id)]  # position [x, y, z]
            kf.X[3:5] = np.zeros(2)  # velocity [vx, vy]
            kf.X[5:] = np.zeros(2)   # acceleration [ax, ay]
            kalman_filters[vehicle_id] = kf

    # Predict next positions using the Kalman filter
    predicted_centroids = []
    for vehicle_id in prev_ids:
        if vehicle_id in kalman_filters:
            kf = kalman_filters[vehicle_id]
            kf.predict()
            predicted_centroids.append(kf.get_state()[:3])

    # Compute distance matrix and perform assignment
    distance_matrix = compute_distance_matrix(predicted_centroids, curr_centroids)
    row_ind, col_ind = linear_sum_assignment(distance_matrix)

    matches = []
    unmatched_prev = set(range(len(prev_centroids)))
    unmatched_curr = set(range(len(curr_centroids)))

    for r, c in zip(row_ind, col_ind):
        if distance_matrix[r, c] < threshold:
            matches.append((prev_ids[r], curr_ids[c]))
            unmatched_prev.discard(r)
            unmatched_curr.discard(c)

            kf = kalman_filters[prev_ids[r]]
            prev_pos = prev_centroids[r]
            curr_pos = curr_centroids[c]
            prev_velocity = kf.X[3:5]
            curr_velocity = compute_velocity(prev_pos, curr_pos, delta_time)
            acceleration = compute_acceleration(prev_velocity, curr_velocity, delta_time)

            kf.update(curr_pos)
            kf.X[3:5] = curr_velocity
            kf.X[5:] = acceleration

    # Handle new and lost vehicles
    exited_vehicles = [prev_ids[i] for i in unmatched_prev]
    entered_vehicles = [curr_ids[i] for i in unmatched_curr]

    # Add Kalman filters for new vehicles
    for vehicle_id in entered_vehicles:
        curr_pos = curr_centroids[curr_ids.index(vehicle_id)]
        kf = KalmanFilter(delta_time)
        kf.X[:3] = curr_pos
        kf.X[3:5] = np.zeros(2)
        kf.X[5:] = np.zeros(2)
        kalman_filters[vehicle_id] = kf

    # Remove filters for lost vehicles
    for vehicle_id in exited_vehicles:
        if vehicle_id in kalman_filters:
            del kalman_filters[vehicle_id]

    return matches, exited_vehicles, entered_vehicles, predicted_centroids


def calculate_threshold(df, sensor_frequency, percentage_margin):
    """
    Compute a dynamic matching threshold based on vehicle speed.

    Args:
        df (pd.DataFrame): DataFrame with 'vx' and 'vy' velocity columns.
        sensor_frequency (float): Sensor frequency (Hz).
        percentage_margin (float): Additional margin percentage.

    Returns:
        float: Adaptive distance threshold.
    """
    vx, vy = df['vx'], df['vy']
    v_max = np.sqrt(vx**2 + vy**2).max()
    threshold = v_max / sensor_frequency
    return threshold + threshold * (percentage_margin / 100)


def calculate_mse(predicted_trajectories, real_trajectories, tracking_threshold, scan):
    """
    Calculate Mean Squared Error (MSE) between matched predicted and real trajectories.

    Args:
        predicted_trajectories (dict): Predicted trajectories indexed by ID.
        real_trajectories (dict): Ground-truth trajectories indexed by ID.
        tracking_threshold (float): Maximum distance for a valid match.
        scan (int): Frame index for evaluation.

    Returns:
        None
    """
    matched_ids = {}
    for real_id, real_traj in real_trajectories.items():
        min_distance = float('inf')
        matched_pred_id = None

        for pred_id, pred_traj in predicted_trajectories.items():
            dist = distance.euclidean(np.mean(real_traj, axis=0), np.mean(pred_traj, axis=0))
            if dist < tracking_threshold and dist < min_distance:
                min_distance = dist
                matched_pred_id = pred_id

        if matched_pred_id is not None:
            matched_ids[real_id] = matched_pred_id

            if len(real_traj) > scan and len(predicted_trajectories[matched_pred_id]) > (scan - 1):
                real_points = np.array(real_traj[scan])
                pred_points = np.array(predicted_trajectories[matched_pred_id][scan - 1])
                mse = np.mean((real_points - pred_points) ** 2)
                mse_values[(real_id, matched_pred_id)] = mse
                print(f"Matched Real ID {real_id} ↔ Predicted ID {matched_pred_id} | MSE: {mse:.4f}")
            else:
                print(f"Skipping MSE for Real ID {real_id} ↔ Predicted ID {matched_pred_id}: insufficient data.")
        else:
            print(f"No match found for Real ID {real_id}")
