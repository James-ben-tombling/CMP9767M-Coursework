#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import actionlib
from std_msgs.msg import String
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    camera = rospy.Publisher('/camera', String, queue_size=10)
    client.wait_for_server()

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint1"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    
    # send second goal
    goal.target = "WayPointSnap1"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15):
        camera.publish('go')
        rospy.sleep(0.2)
    rospy.sleep(10)

    # send Third  goal 
    goal.target = "WayPoint2"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15):
        camera.publish('go')
        rospy.sleep(0.2)
    rospy.sleep(10)

    # send fourth goal
    goal.target = "WayPoint7"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15):
        camera.publish('go')
        rospy.sleep(0.2)
    rospy.sleep(10)

    # send fifth  goal  
    goal.target = "WayPointSnap2"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15):
        camera.publish('go')
        rospy.sleep(0.2)
    rospy.sleep(10)
    
    # send sixth goal to complete scircuit 
    goal.target = "WayPoint0"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)