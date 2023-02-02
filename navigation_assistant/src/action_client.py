#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the NavAssistant
from navigation_assistant.msg import nav_assistantAction, nav_assistantGoal

def navAssistant_client():
    # Creates the SimpleActionClient, passing the type of the action
    rospy.loginfo("[NavClient] In...")
    client = actionlib.SimpleActionClient('nav_assistant', nav_assistantAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("[NavClient] Waiting for server")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    rospy.loginfo("[NavClient] Setting goal")
    goal = nav_assistantGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -6.0
    goal.target_pose.pose.position.y = 3.3
    goal.target_pose.pose.position.z = 0.0 
    goal.target_pose.pose.orientation.w = 1
    goal.turn_before_nav = True
    #quat = quaternion_from_euler(0, 0, 0.0)
    #goal.target_pose.pose.orientation = Quaternion(*quat)
    
    # Sends the goal to the action server.
    rospy.loginfo("[NavClient] sending goal")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    rospy.loginfo("[NavClient] waiting for result")
    client.wait_for_result()

    # Prints out the result of executing the action
    rospy.loginfo("[NavClient] done")
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('nav_assistant_client')
    result = navAssistant_client()

