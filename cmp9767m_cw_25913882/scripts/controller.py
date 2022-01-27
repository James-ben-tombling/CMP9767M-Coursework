#! /usr/bin/env python
# controller and goal scripts

# @date: 24/01/22

# this code is based upon code created by:
# @author: gpdas
# @email: pdasgautham@gmail.com


#import libs
import rospy
import actionlib
from std_msgs.msg import String
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    # initialise Pubs, Subs and nodes 
    rospy.init_node('topological_navigation_client')
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    camera = rospy.Publisher('/camera', String, queue_size=10)
    vine_camera = rospy.Publisher('/vine_camera', String, queue_size=10)
    client.wait_for_server()

    # set goto node 
    goal = GotoNodeGoal()
    # send first goal
    goal.target = "WayPoint1"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status) # log info 
    rospy.loginfo("result is %s", result)
    
    # send second goal
    goal.target = "WayPointSnap1"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15): # take 15 snap shots of the vine
        camera.publish('go') # pub to camera in identifier.py to analyses No. of grape bunch in frame 
        rospy.sleep(0.2) 
    vine_camera.publish('go') # pub to vine mapper in identifier.py 
    rospy.sleep(20)
    
    # code repeats just edits the goal target 
    
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
    vine_camera.publish('go')
    rospy.sleep(20)


    # send fourth goal
    goal.target = "WayPoint3"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15):
        camera.publish('go')
        rospy.sleep(0.2)
    vine_camera.publish('go')
    rospy.sleep(20)

    # send fifth goal
    goal.target = "WayPoint6"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    for i in range(15):
        camera.publish('go')
        rospy.sleep(0.2)
    vine_camera.publish('go')
    rospy.sleep(20)

    # send fifth goal
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
    vine_camera.publish('go')
    rospy.sleep(20)

    # send sixth  goal  
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
    vine_camera.publish('go')
    rospy.sleep(20)

    # send seventh goal to complete scircuit 
    goal.target = "WayPoint0"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
