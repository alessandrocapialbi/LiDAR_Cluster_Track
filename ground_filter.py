import os
import pandas as pd

point_cloud_directory = "\\sensors\\"

output_directory = "\\filtered_data_sensors\\"


# All the z points below the threshold are considered as ground points.
def filter_ground(z_threshold):
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    for file_name in os.listdir(point_cloud_directory):
        if file_name.endswith('.csv'):
            input_file_path = os.path.join(point_cloud_directory, file_name)
            output_file_path = os.path.join(output_directory, file_name)

            # Read CSV file into a DataFrame
            df = pd.read_csv(input_file_path)

            # Filter out the ground based on a z_threshold
            filtered_df = df[df['z'] > z_threshold]

            # Save the filtered DataFrame to a new CSV file
            filtered_df.to_csv(output_file_path, index=False)
            print(f'Processed and saved: {output_file_path}')
