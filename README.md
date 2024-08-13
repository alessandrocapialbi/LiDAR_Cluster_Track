# Clustering and Tracking on LiDAR data of vehicles in Pittsburgh

## Description
This project uses clustering algorithms to analyze and visualize 3D point clouds obtained from scans of traffic by LiDAR sensors taken by the **Argoverse 2** dataset. Moreover, it implements tracking algorithms to keep identifying new vehicles during time. The program loads sensor scans, transforms the coordinates into a global system, performs clustering, and visualizes the results with bounding boxes.

## Requirements
- Python 3.x
- pip

## Installation
1. Clone the repository:
    ```sh
    git clone <https://github.com/alessandrocapialbi/LiDAR_Cluster_Track/>
    cd <repository_directory>
    ```

2. Install the dependencies:
    ```sh
    pip install -r requirements.txt
    ```

## Usage
1. Run the main program:
    ```sh
    python main.py
    ```

2. Follow the on-screen instructions to enter the number of available sensors.

## Main Files
- `main.py`: Contains the main program flow, including sensor selection, point cloud loading, and result visualization.
- `clustering.py`: Contains functions for performing DBSCAN clustering and creating bounding boxes.

## Main Functions
### `main.py`
- `select_sensors(num_sensors)`: Selects the sensors to use.
- `load_point_clouds_from_sensors(directory, selected_sensors, num_scans)`: Loads point clouds from the selected sensors.
- `load_sensor_positions(path)`: Loads sensor positions.
- `calculate_sensors_centroid(df)`: Calculates the centroid of the sensor positions.
- `load_and_transform_scan(file_path, sensors_positions_df, centroid, sensor_id)`: Transforms scan coordinates into a global system.
- `merge_point_clouds(point_clouds)`: Merges the point clouds.
- `downsample_pcd(pcd, voxel_size)`: Downsamples the point cloud.
- `dbscan_clustering(pcd, plot_k_distance)`: Performs DBSCAN clustering.
- `create_bounding_boxes(clusters)`: Creates bounding boxes for the clusters.

### `clustering.py`
- `dbscan_clustering(pcd, eps, min_points, plot_k_distance)`: Performs DBSCAN clustering and calculates evaluation scores.
- `create_bounding_boxes(clusters)`: Creates bounding boxes for the clusters.

## Clustering Evaluation
The program calculates three evaluation scores for the clustering:
- **Silhouette Score**
- **Davies-Bouldin Index**
- **Calinski-Harabasz Index**

## Visualization
The program uses `open3d` to visualize the point clouds and the bounding boxes of the clusters.

## Contributions
Contributions are welcome! Feel free to open issues or pull requests to improve the project.

## License
This project is distributed under the MIT license. See the `LICENSE` file for more details.
