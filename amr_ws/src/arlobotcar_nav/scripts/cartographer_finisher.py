import rospy
import os
#from cartographer_ros_msgs.srv import MyCartographerService  # Replace with the actual service message type
from cartographer_ros_msgs.srv import FinishTrajectory 
from cartographer_ros_msgs.srv import WriteState
import rospkg
from rosnode import kill_nodes
rospack = rospkg.RosPack()
package ='arlobotcar_nav'
package_path = rospack.get_path(package)
import argparse
import subprocess
def main():
    rospy.init_node('finish_trajectory_service_node')

    # Create a client to call the Cartographer service
    rospy.wait_for_service('/finish_trajectory')  # Replace with the actual service name
    rospy.wait_for_service('/write_state')
    write_state = rospy.ServiceProxy('/write_state', WriteState)

    cartographer_service = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)  # Replace with the actual service message type

    try:
        # Call the service
        #request = WriteState()
       # request.filename = "${HOME}/Downloads/testingpython.bag.pbstream"
       # request.include_unfinished_submaps = True
        parser = argparse.ArgumentParser(description='Move the robot to a goal (x, y, yaw).')
        parser.add_argument('map', type=str, help='map name for csv')
        args = parser.parse_args()
        file_path = package_path + '/maps/' + args.map + '.bag.pbstream'
        file_map = package_path + '/maps/' + args.map 
        coomand = ["rosrun", "map_server", "map_saver", "-f", file_map]
        #file_path = "/home/cidis-laptop/amr/amr_ws/src/arlobot_sim_navigation/maps/test.bag.pbstream"
        response = cartographer_service(0)  # Pass the desired request here
        response1 = write_state(file_path, True)
        subprocess.run(coomand, check=True)
        #response_write = write_state(request)
        print("Service response:", response)
        # Handle the service response if needed
        # For example, you can print the response
        print("Service response:", response1)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

    kill_nodes('finish_trajectory_service_node')

if __name__ == '__main__':
    main()