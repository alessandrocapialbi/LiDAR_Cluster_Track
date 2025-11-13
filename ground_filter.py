import os
import pandas as pd

# Define directories
current_directory = os.path.dirname(os.path.abspath(__file__))
point_cloud_directory = os.path.join(current_directory, 'sensors_data')
output_directory = os.path.join(current_directory, 'filtered_sensors_data')


def ground_filter(z_threshold):
    """
    Filter ground points from LiDAR point cloud CSV files based on a Z-axis threshold.

    Any point with a z-coordinate below the threshold is considered ground and removed.

    Args:
        z_threshold (float): Minimum z-coordinate value to keep (points below are discarded).

    Process:
        - Creates the output directory if it does not exist.
        - Iterates through all CSV files in the point cloud directory.
        - Reads each CSV into a pandas DataFrame.
        - Filters out rows where 'z' <= z_threshold.
        - Saves the filtered points to the output directory with the same filename.
    """
    # Create output directory if it does not exist
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    # Iterate over all CSV files in the input directory
    for file_name in os.listdir(point_cloud_directory):
        if file_name.endswith('.csv'):
            input_file_path = os.path.join(point_cloud_directory, file_name)
            output_file_path = os.path.join(output_directory, file_name)

            # Read CSV file into a DataFrame
            df = pd.read_csv(input_file_path)

            # Filter out ground points based on z_threshold
            filtered_df = df[df['z'] > z_threshold]

            # Save the filtered points to a new CSV file
            filtered_df.to_csv(output_file_path, index=False)
            print(f'Processed and saved: {output_file_path}')
