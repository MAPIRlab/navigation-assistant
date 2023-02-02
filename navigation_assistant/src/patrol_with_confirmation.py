#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int8, Bool
from tf.transformations import quaternion_from_euler

from task_manager.srv import addTask
from sensor_msgs.msg import Joy
from topology_graph.srv import graph

# Bring in the messages used by the NavAssistant
import actionlib
from actionlib_msgs.msg import GoalStatus
from navigation_assistant.msg import nav_assistantAction, nav_assistantGoal
from navigation_assistant.srv import nav_assistant_point
from nav_msgs.srv import GetPlan

import copy
import numpy as np
from numpy.linalg import inv
import cv2 as cv
import matplotlib
import matplotlib.pyplot as plt
import math



class Patrol(object):
    # ---------------------------------------------------------------------
    #                                INIT
    # ---------------------------------------------------------------------
    def __init__(self):
        rospy.init_node("patrol_with_confirmation")
        
        # Read params
        self.verbose = rospy.get_param("~verbose",True)
        self.topic_buttons = rospy.get_param('~topic_buttons',"/giraff_node/buttons")
        self.topic_start_navigation = rospy.get_param('~topic_start_navigation',"/e-nose/start")
        self.topic_completed_navigation = rospy.get_param('~topic_completed_navigation',"/e-nose/done")
        self.talk = False

        # init vars
        self.button_pressed = False
        self.last_button_state = None
        self.go_to_next_point = True
        
        self.nodes = []
        self.cancel_request = False
        
        # Subscribers
        rospy.Subscriber(self.topic_buttons, Joy, self.buttons_msg_cb)                      # Green/Reg buttons
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amclPoseCallback)
        rospy.Subscriber(self.topic_start_navigation, Bool, self.startNavigationCallback)
        

        #Publishers
        self.pub_completed_nav = rospy.Publisher(self.topic_completed_navigation, PoseWithCovarianceStamped,  queue_size=1)
        

        # Service clients
        rospy.wait_for_service('/bt_manager/add_new_task')
        rospy.wait_for_service('/topology_graph/graph')
        self.add_task_srv_proxy = rospy.ServiceProxy('/bt_manager/add_new_task', addTask)   # Talk
        self.graph_srv_proxy = rospy.ServiceProxy('/topology_graph/graph', graph)           # Topo-Graph
        
        # START TALK
        if self.verbose: rospy.loginfo( "[Patrol] Ready for Action." )
        if self.talk: self.say_voice("Hello. I am initiating an inspection task to measure the CO2 levels in this area.", 5.0)
        if self.talk: self.say_voice("Place on the map all the locations that you want me to visit and measure, and press the Green button to start the action.", 5.0)

        # Action Server clients
        self.nav_client = actionlib.SimpleActionClient('nav_assistant', nav_assistantAction)
        self.nav_client.wait_for_server()


        # 0. Wait for User Confirmation
        #------------------------------
        #if self.verbose: rospy.loginfo( "[Patrol] Waiting GREEN button press." )
        #self.wait_for_green_button()

        #if self.cancel_request:
        #    self.close_and_return()
        #    return
          
          
        # 1. Get All Spaces/Nodes (from graph)
        #------------------------
        rospy.sleep(2)
        res = self.graph_srv_proxy(cmd="GetNodesbyType", params=["space"])
        self.nodes = res.result

        # Talk
        if self.verbose: rospy.loginfo("[Patrol] I have detected a total of " + str(len(self.nodes)) + " measurement points in this environment.")
        if self.talk: self.say_voice("I have detected a total of " + str(len(self.nodes)) + "measurement points in this environment.", 4.0)
        if self.talk: self.say_voice("I'll go now and start my round.", 2.0)

        #sort the nodes by (x,y) I wouldn't do this unless the environment is totally empty
        #tempnodes= sorted(self.nodes, key= lambda p: (float(p.split()[3]), float(p.split()[4])))
        #self.nodes=tempnodes


        while not rospy.is_shutdown():
            # 2. Navigate to all nodes and wait for measurement confirmation
            #---------------------------------------------------------------
            n = 0
            max_attemps = 3

            while n < len(self.nodes):
                n_data = self.nodes[n].split()  # n = [id, label, type, x, y, yaw]
     
                # Navigate to node
                #------------------------                
                nav_completed = False
                num_attemp = 1
                
                while (not nav_completed) and (num_attemp <= max_attemps):
                    if self.talk: self.say_voice("Navigating to " + n_data[1] + " . Attemp " + str(num_attemp), 2.0)
                    if self.verbose: rospy.loginfo( "[Patrol] Navigating to " + n_data[1] + " . Attemp " + str(num_attemp) )

                    # Command the navigation!
                    nav_completed = self.navigate_to_node(n_data, True)

                    if self.cancel_request:
                        self.close_and_return()
                        return

                    if nav_completed:
                        if self.talk: self.say_voice("Navigation comleted with success.", 3.0)
                        if self.verbose: rospy.loginfo( "[Patrol] Navigation comleted with success." )
                        
                        # Inform the system to start measuring
                        self.pub_completed_nav.publish(self.robot_pose)
                        self.go_to_next_point = False

                    if self.cancel_request:
                        self.close_and_return()
                        return
                      
                    num_attemp += 1
                #end-while
              
                # Wait for confirmation to navigate to next point
                while not self.go_to_next_point:
                    rospy.sleep(0.5)
                
                # Lets move to the next node!                        
                n += 1
                
            #end while nodes   

            # Go back to dock and recharge till full
            self.add_task_srv_proxy(task_name="dock", task_priority=5, task_permanence=False, task_impact="None", task_args=['[-1,0,0]','false'])
            sleep_time = 1800
            current_sleep_time = 0
            while (not rospy.is_shutdown() and (sleep_time > current_sleep_time)):
                rospy.sleep(1)
                current_sleep_time += 1
        #end while main loop

    # ====================================================== #
    #               NAVIGATION REQUEST                       #
    # ====================================================== #
    def navigate_to_node(self, node_data, record_path):
        # Creates a goal to be send to the Navigation action server.
        # node_data = [id, label, type, x, y, yaw] -> string[]
        goal = nav_assistantGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(node_data[3])
        goal.target_pose.pose.position.y = float(node_data[4])
        goal.target_pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, float(node_data[5]))
        goal.target_pose.pose.orientation = Quaternion(*quat)

        goal.turn_before_nav = True

        # Keep track
        if record_path:
            self.record_robot_pose = True

        # Sends the goal to the action server.
        self.nav_client.cancel_all_goals()
        rospy.sleep(0.5)
        self.nav_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        if self.verbose: rospy.loginfo("[Patrol] Navigation requested. Waiting for navigation to complete...")
        while not self.nav_client.wait_for_result(rospy.Duration(2.0)):
            if self.cancel_request:
                self.close_and_return()
                break


        # Stop tracking
        self.record_robot_pose = False
        rospy.sleep(0.5)

        # Prints out the result of executing the action
        if self.nav_client.get_state() == 3:
            if self.verbose: rospy.loginfo("[Patrol] Navigation succes.")
            return True
        else:
            if self.verbose: rospy.loginfo("[Patrol] Navigation failed.")
            return False





    # ====================================================== #
    #                    AMCL CALLBACK                       #
    # ====================================================== #
    def amclPoseCallback(self, msg):
        # geometry_msgs/PoseWithCovarianceStamped (time, frame_id, pose)
        self.robot_pose = msg
        
        #quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
        #data.pose.pose.orientation.z, data.pose.pose.orientation.w )
        #amclth = tf.transformations.euler_from_quaternion(quaternion)[2]
        
    # ====================================================== #
    #                    START NAVIGATION CALLBACK           #
    # ====================================================== #
    def startNavigationCallback(self, msg):
        # Bool
        self.go_to_next_point = True
        
    # ====================================================== #
    #                    CLOSE & RETURN                      #
    # ====================================================== #
    def close_and_return(self):

        # STOP THE ROBOT
        self.nav_client.cancel_all_goals()
        self.say_voice("Canceling patrol by request.", 2.0)

        return

    # ====================================================== #
    #                   BUTTON CALLBACK                      #
    # ====================================================== #
    def buttons_msg_cb(self, joy_msg):
        # New button_pressed! -> order is: (red, greeen, dial, e-stop)
        self.button_pressed = True
        self.last_button_state = joy_msg

        # IS button RED??
        if self.last_button_state.buttons[0] == 1:
            # Cancel current Navigation (if any)
            self.close_and_return()


    # ====================================================== #
    #                   WAIT FOR GREEN BUTTON                #
    # ====================================================== #
    def wait_for_green_button(self):
        green = False
        self.button_pressed = False
        while not green and not self.cancel_request:
            if self.button_pressed:
                self.button_pressed = False
                if self.last_button_state.buttons[1] == 1:
                    green = True
            rospy.sleep(0.5)
        #end-while


    # ====================================================== #
    #                     SAY VOICE                          #
    # ====================================================== #
    def say_voice(self, voice_msg, t_sec):
        self.add_task_srv_proxy(task_name="say", task_priority=11, task_permanence=False, task_impact="None", task_args=[voice_msg])
        rospy.sleep(t_sec)



# =============================================================
# ==========================  MAIN  ===========================
# =============================================================
if __name__ == '__main__':
    my_patrol = Patrol()

