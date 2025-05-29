#!/usr/bin/env python3
import rospy
import actionlib
from multi_map_nav.msg import NavigateToGoalAction, NavigateToGoalGoal

def send_goal():
    client = actionlib.SimpleActionClient('navigate_to_goal', NavigateToGoalAction)
    client.wait_for_server()
    
    goal = NavigateToGoalGoal()
    # goal.target_map = "map3"
    # goal.target_x = -1.5
    # goal.target_y = 8.0
    
    goal.target_map = "map1"
    goal.target_x = 10.4
    goal.target_y = -2.0
    
    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('navigation_client')
    result = send_goal()
    print("Result:", result.success, result.message)
