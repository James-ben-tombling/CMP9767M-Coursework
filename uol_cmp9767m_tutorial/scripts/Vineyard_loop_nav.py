#! /usr/bin/env python
# ----------------------------------
# @author: James Tombling 
# @email:
# @date:
# ----------------------------------

import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint0"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    
    # send second goal
    goal.target = "WayPoint7"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)

    # send Third  goal to complete scircuit 
    goal.target = "WayPoint0"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)