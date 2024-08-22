import numpy as np
from scipy.optimize import linear_sum_assignment


def compute_distance_matrix(prev_boxes, curr_boxes):
    distance_matrix = np.zeros((len(prev_boxes), len(curr_boxes)))

    for i, prev in enumerate(prev_boxes):
        for j, curr in enumerate(curr_boxes):
            distance_matrix[i, j] = np.linalg.norm(np.array(prev) - np.array(curr))

    return distance_matrix


def track_vehicles(prev_centroids, curr_centroids, prev_ids, curr_ids, threshold):
    distance_matrix = compute_distance_matrix(prev_centroids, curr_centroids)
    row_ind, col_ind = linear_sum_assignment(distance_matrix)

    matches = []
    unmatched_prev = set(range(len(prev_centroids)))
    unmatched_curr = set(range(len(curr_centroids)))

    for r, c in zip(row_ind, col_ind):
        if distance_matrix[r, c] < threshold:
            matches.append((prev_ids[r], curr_ids[c]))
            unmatched_prev.discard(r)
            unmatched_curr.discard(c)

    # Convert to actual IDs
    exited_vehicles = [prev_ids[i] for i in unmatched_prev]
    entered_vehicles = [curr_ids[i] for i in unmatched_curr]

    return matches, exited_vehicles, entered_vehicles


def calculate_threshold(df):
    vx = df['vx']
    vy = df['vy']

    v_max = (np.sqrt(vx ** 2 + vy ** 2)).max()

    return v_max
