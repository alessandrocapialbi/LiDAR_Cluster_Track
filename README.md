# Clustering and Vehicle Tracking of vehicles in Pittsburgh with Visualization

This project involves clustering and tracking vehicles using point cloud data from multiple sensors taken by the dataset **Argoverse 2**. The data is processed to transform coordinates, perform clustering, tracking and visualize the results using Open3D. The original files are not stored, but they have columns: timestamp, yaw, pitch, distance, distance_noise,  x, y, z, which contain the coordinates of each point extracted by the LIDAR sensor; x_noise, y_noise, z_noise contain the same data with added noise to simulate the imperfections of a real sensor; object_id, color (R), color (G), color (B).

The file name follows this convention: sensor_X_YY.csv provides the output of sensor X at scan YY.

The file pitt_trajectories.csv contains the ground-truth trajectories of the vehicles, while the file pitt_sensor_positions.csv contains the positions of the sensors.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Modules](#modules)
- [License](#license)

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/alessandrocapialbi/LiDAR_Cluster_Track
    cd LiDAR_Cluster_Track
    ```

2. Create a virtual environment and activate it:
    ```sh
    python3 -m venv venv
    source venv/bin/activate
    ```

3. Install the required packages:
    ```sh
    pip install -r requirements.txt
    ```

## Usage

1. Run the main script:
    ```sh
    python main.py
    ```

2. Follow the prompts to enter the number of sensors.

## Project Structure

```plaintext
LiDAR_Cluster_Track/
├── data_loader.py
├── main.py
├── clustering.py
├── simulation.py
├── transform_coordinates.py
├── tracking.py
├── sensor_selection.py
├── sensors_data/
├── filtered_sensors_data/
├── sensors_positions/
│   └── pitt_sensor_positions.csv
└── trajectories/
    └── pitt_trajectories.csv
└── output/
    └── point_clouds.csv
    └── bounding_boxes.csv
    └── predicted_trajectories.csv

```

## Modules

### `main.py`
The main script that orchestrates the loading, processing, and visualization of point cloud data.

### `data_loader.py`
Contains functions to load data from files.

### `clustering.py`
Performs DBSCAN clustering on the point cloud data and creates bounding boxes.

### `simulation.py`
Handles the visualization of point clouds and trajectories using Open3D.

### `transform_coordinates.py`
Transforms the coordinates of the point cloud data to a global coordinate system, considering translations and rotations.

### `kalman_filter.py`
Implements a LKF (Linear Kalman Filter) to predict the trajectories of the vehicles.

### `tracking.py`
Tracks vehicles between scans based on bounding box centroids.

### `sensor_selection.py`
Allows the user to select the sensors to use for the tracking.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.
