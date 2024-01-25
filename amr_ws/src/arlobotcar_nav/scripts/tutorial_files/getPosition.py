#! /usr/bin/env python
import pandas as pd
import rospkg
rospack = rospkg.RosPack()
package ='arlobotcar_nav'
package_path = rospack.get_path(package)

# Read the CSV file into a DataFrame

# Define the function
def get_position_by_id(map,search_id):
    file_path = package_path + '/coordinates/' + map + '.csv'
    print(file_path)
    positions_df = pd.read_csv(file_path)
    print(positions_df)
    # Filter the DataFrame for the given ID
    position_row = positions_df[positions_df['estacion'] == search_id]

    # Check if the ID exists in the DataFrame
    if not position_row.empty:
        # Extract the X and Y positions
        x_position = position_row['xpos'].iloc[0]
        y_position = position_row['ypos'].iloc[0]
        return x_position, y_position
    else:
        # Return None if the ID is not found
        return None, None
def hello():
    print("hello")