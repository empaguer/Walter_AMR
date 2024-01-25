import pandas as pd
import buscar_orientacion
import move_base_test
# Path to the CSV file
file_path = '/home/cidis-laptop/positions.csv' 


# Read the CSV file into a DataFrame
positions_df = pd.read_csv(file_path)

# Print each line of the DataFrame
for index, row in positions_df.iterrows():
    print(f"ID: {row[0]}, X Position: {row[1]}, Y Position: {row[2]}")

if __name__ == '__main__':
    try:
        move_to_desired_position(0)

    except rospy.ROSInterruptException:
        pass