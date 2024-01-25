#! /usr/bin/env python
import pandas as pd
import rospy
import time
#import buscar_orientacion
#import move_base_test
#import move_base_real
import argparse
from tutorial_package.getPosition import hello, get_position_by_id
from tutorial_package.move_base_real import move_to_goal
#from tutorial_package.buscar_orientacion import move_to_orientation
from tutorial_package.new_orientation import OrientationReacher
# Example usage
id_to_search = 2  # Replace with the ID you want to search

if __name__ == '__main__':
    #hello()
    rospy.init_node('map_navigation', anonymous=False)
    parser = argparse.ArgumentParser(description='Move the robot to a goal (x, y, yaw).')
    parser.add_argument('map', type=str, help='map name for csv')
    parser.add_argument('station', type=float, help='The Station that you want the robot to move')
    
    args = parser.parse_args()
    x_base , y_base = get_position_by_id(args.map,0)
    x_pos, y_pos = get_position_by_id(args.map, args.station)
    #print(x_pos,y_pos)
    if x_pos is not None and y_pos is not None:
        print(f"ID: {args.station}, X Position: {x_pos}, Y Position: {y_pos}")
        rospy.loginfo("Start go to goal")
        succes_position =move_to_goal(x_pos,y_pos)
        if succes_position:
            rospy.loginfo("Goal reached, searching orientation")
            orientation_class = OrientationReacher(args.station)
            orientation_class.run()
            if(args.station ==0):
                if(orientation_class.is_position_reached):
                    rospy.loginfo("Walter succesfully placed in BASE")
                else:
                    rospy.loginfo("Apriltag not in base, Somebody Stole it???")
            else:
                #Si la estacion deseada no es la base
                if(orientation_class.is_position_reached):
                    time.sleep(5)
                    #this time.sleep simulates the hand feedback
                    final_position = move_to_goal(x_base,y_base)
                    if final_position:
                        rospy.loginfo("Base reached, searching orientation")
                        orientation_class2 = OrientationReacher(0)
                        orientation_class2.run()
                        if(orientation_class2.is_position_reached):
                            rospy.loginfo("Excelent Performance")
                        else:
                            rospy.loginfo("SOmebody Stole the BASE ID!!!")
                    else:
                        rospy.loginfo(f"Something happened in the way Home from ID {args.station}, go and Check!!")
                else:
                    rospy.loginfo(f"ID: {args.station} not at sight, somebody stole it? Returning to BASE.")
                    move_to_goal(x_base, y_base)
        else:
            rospy.loginfo("Walter can not reach the desired location")
            if(args.station!=0):
                #If the order was to move to a table and not to the base, walter will return to base
                move_to_goal(x_base,y_base)
    else:
        print(f"ID: {args.station} does not exist, did you forget to input the coordinates?.")
        #APAGAR TOODO
    rospy.signal_shutdown("Goal reached or failed")