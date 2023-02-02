#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
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


class RANSAC:
    def __init__(self, x_data, y_data, n):
        self.x_data = x_data
        self.y_data = y_data
        self.n = n
        self.d_min = 99999
        self.best_model = None

    def random_sampling(self):
        sample = []
        save_ran = []
        count = 0

        # get three points from data
        while True:
            ran = np.random.randint(len(self.x_data))

            if ran not in save_ran:
                sample.append((self.x_data[ran], self.y_data[ran]))
                save_ran.append(ran)
                count += 1

                if count == 3:
                    break

        return sample


    def make_model(self, sample):
        # calculate A, B, C value from three points by using matrix
        try:
            pt1 = sample[0]
            pt2 = sample[1]
            pt3 = sample[2]

            A = np.array([[pt2[0] - pt1[0], pt2[1] - pt1[1]], [pt3[0] - pt2[0], pt3[1] - pt2[1]]])
            B = np.array([[pt2[0]**2 - pt1[0]**2 + pt2[1]**2 - pt1[1]**2], [pt3[0]**2 - pt2[0]**2 + pt3[1]**2 - pt2[1]**2]])
            inv_A = inv(A)

            c_x, c_y = np.dot(inv_A, B) / 2
            c_x, c_y = c_x[0], c_y[0]
            r = np.sqrt((c_x - pt1[0])**2 + (c_y - pt1[1])**2)

            return True, (c_x, c_y, r)
        except np.linalg.LinAlgError as err:
            if 'Singular matrix' in str(err):
                return False, (0,0,0)
            else:
                raise

    def eval_model(self, model):
        d = 0
        c_x, c_y, r = model

        for i in range(len(self.x_data)):
            dis = np.sqrt((self.x_data[i]-c_x)**2 + (self.y_data[i]-c_y)**2)

            if dis >= r:
                d += dis - r
            else:
                d += r - dis

        return d

    def execute_ransac(self):
        # find best model
        for i in range(self.n):
            success, model = self.make_model(self.random_sampling())
            if success:
                d_temp = self.eval_model(model)

                if self.d_min > d_temp:
                    self.best_model = model
                    self.d_min = d_temp



class NavTest(object):
    # ---------------------------------------------------------------------
    #                                INIT
    # ---------------------------------------------------------------------
    def __init__(self):
        rospy.init_node("evaluate_navigation")
        
        # Read params
        self.verbose = rospy.get_param("~verbose",False)
        self.topic_buttons = rospy.get_param('~topic_buttons',"/giraff_node/buttons")
        
        # init vars
        self.button_pressed = False
        self.last_button_state = None

        self.nodes = []
        self.nav_results = []
        self.cancel_request = False

        self.current_robot_location = None
        self.record_robot_pose = False
        self.robot_pose_history = []
        self.nav_map = OccupancyGrid()
        self.last_update_time = rospy.Time.now()
        self.T = []
        self.T_inv = []
        self.robot_speed = 0.0
        self.global_costmap = OccupancyGrid()


        # Subscribers
        rospy.Subscriber(self.topic_buttons, Joy, self.buttons_msg_cb)                      # Green/Reg buttons
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amclPoseCallback)
        rospy.Subscriber("odom", Odometry, self.odomPoseCallback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.globalCostMap_cb)
        self.hasMap = False
        self.has_global_costmap = False

        #Publisher
        self.navMap_pub = rospy.Publisher('nav_map', OccupancyGrid, queue_size=1)

        # Service clients
        rospy.wait_for_service('/bt_manager/add_new_task')
        rospy.wait_for_service('/topology_graph/graph')
        rospy.wait_for_service('move_base/make_plan')
        self.add_task_srv_proxy = rospy.ServiceProxy('/bt_manager/add_new_task', addTask)   # Talk
        self.graph_srv_proxy = rospy.ServiceProxy('/topology_graph/graph', graph)           # Topo-Graph
        self.navAssist_srv_proxy = rospy.ServiceProxy('/navigation_assistant/add_point_of_interest', nav_assistant_point)       # NavAssistant-Add-POI
        self.make_plan_srv_proxy = rospy.ServiceProxy('move_base/make_plan', GetPlan)       # Get Plan


        # TEST (comment)
        #========================================
        test = PoseWithCovarianceStamped()
        test.pose.pose.position.x = -2.25
        test.pose.pose.position.y = 1.25
        self.pose_to_cell(test)

        idx = 878250

        # Get costmap as img arround cell idx
        test_img, top_left_idx = self.crop_costmap_as_img(idx)

        # Set CNP in the close vecinity
        Pimg = self.get_CNP_from_img(test_img)

        #Pcnp is given in pixels of the cropped-img
        # Get Coordinates in meters to add a CNP
        rospy.loginfo("New CNP at pixles (x=%d, y=%d)[px] of the Cropped Image", int(Pimg[0]), int(Pimg[1]) )

        idx_cnp = int(top_left_idx -int(Pimg[1])*self.nav_map.info.width + int(Pimg[0]))
        Pcnp = self.cell_to_pose(idx_cnp)
        rospy.loginfo("New CNP at cell=%d with pose (x=%.3f, y=%.3f)[m]", idx_cnp, Pcnp[0], Pcnp[1] )

        # Add CNP
        Pose_cnp = PoseStamped()
        Pose_cnp.pose.position.x = Pcnp[0]
        Pose_cnp.pose.position.y = Pcnp[1]
        Pose_cnp.pose.position.z = 0.0
        Pose_cnp.pose.orientation.w = 1-0
        res = self.navAssist_srv_proxy(action="add", type="CNP", pose=Pose_cnp)



        #========================================


        # START TALK
        if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Ready for Action." )
        self.say_voice("Hello. I am the Navigation Assistant. I am getting ready to perform a Navigation test ", 5.0)
        self.say_voice("Place on the map all the navigation goals that you want me to reach, and press the Green button to start the test. ", 5.0)

        # Action Server clients
        self.nav_client = actionlib.SimpleActionClient('nav_assistant', nav_assistantAction)
        self.nav_client.wait_for_server()


        # 0. Wait for User Confirmation
        #------------------------------
        if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Waiting GREEN button press." )
        self.wait_for_green_button()

        if self.cancel_request:
            self.close_and_return()
            return


        # 1. Get All Spaces/Nodes (from graph)
        #------------------------
        res = self.graph_srv_proxy(cmd="GetNodesbyType", params=["space"])
        self.nodes = res.result

        # Talk
        self.say_voice("I have detected a total of " + str(len(self.nodes)) + "spaces or rooms in this environment.", 4.0)
        self.say_voice("I will now proceed to test navigation to ensure that I can reach every one of them.", 3.0)
        n = self.nodes[0].split()  # n = [id, label, type, x, y, yaw]
        self.say_voice("When you are ready, please, take me to " + n[1] + " and press the Green Button. At any moment you can cancel this test by pressing the Red button.", 1.0)

        # 2. Wait for User Confirmation
        #------------------------------
        if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Waiting GREEN button press." )
        self.wait_for_green_button()

        if self.cancel_request:
            self.close_and_return()
            return

        # 4. Navigate (bidirectional) between all nodes
        #----------------------------------------------
        n1 = 0
        max_attemps = 3

        while n1 < len(self.nodes)-1:
            n1_data = self.nodes[n1].split()  # n = [id, label, type, x, y, yaw]
            n2 = n1 + 1

            while n2 < len(self.nodes):
                n2_data = self.nodes[n2].split()  # n = [id, label, type, x, y, yaw]

                # (GO) Navigate to n2
                #---------------------
                #self.save_path(n1_data, n2_data)
                nav_completed = False
                num_attemps = 1
                self.robot_pose_history = []
                while (not nav_completed) and (num_attemps <= max_attemps):
                    self.say_voice("Navigating to " + n2_data[1] + " . Attemp " + str(num_attemps), 2.0)
                    if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Navigating to " + n2_data[1] + " . Attemp " + str(num_attemps) )

                    nav_completed = self.navigate_to_node(n2_data, True)

                    if self.cancel_request:
                        self.close_and_return()
                        return

                    if nav_completed:
                        self.say_voice("Navigation comleted with success.", 3.0)
                        if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Navigation comleted with success." )
                    else:
                        # Request User help
                        if num_attemps < max_attemps:
                            self.say_voice("I am unable to reach my goal for some reason. Please help me a little bit and press the green button to resume the test.", 3.0)
                            if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Please help me a little bit and press the green button." )
                            self.wait_for_green_button()
                            num_attemps += 1
                        else:
                            self.say_voice("I am unable to reach my goal. Please take me to " + n2_data[1] + " and press the green button to resume the test.", 3.0)
                            if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Please take me to " + n2_data[1] + " and press the green button." )
                            self.wait_for_green_button()
                            num_attemps += 1

                    if self.cancel_request:
                        self.close_and_return()
                        return
                #end-while

                # Save result
                r = dict()
                r["from"] = n1_data
                r["to"] = n2_data
                r["path"] = self.robot_pose_history
                r["dist"] = self.computePathLength(self.robot_pose_history)
                r["time"] = (self.robot_pose_history[-1].header.stamp - self.robot_pose_history[0].header.stamp).to_sec()
                r["success"] = nav_completed
                r["attemps"] = num_attemps
                self.nav_results.append(r)
                if self.verbose:
                    self.print_result([self.nav_results[-1]])

                # (RETURN) Navigate to n1
                #------------------------
                #self.save_path(n2_data, n1_data)
                nav_completed = False
                num_attemps = 1
                self.robot_pose_history = []
                while (not nav_completed) and (num_attemps <= max_attemps):
                    self.say_voice("Navigating back to " + n1_data[1] + " . Attemp " + str(num_attemps), 2.0)
                    if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Navigating to " + n1_data[1] + " . Attemp " + str(num_attemps) )
                    nav_completed = self.navigate_to_node(n1_data, True)

                    if self.cancel_request:
                        self.close_and_return()
                        return

                    if nav_completed:
                        self.say_voice("Navigation comleted with success.", 3.0)
                    else:
                        # Request User help
                        if num_attemps < max_attemps:
                            self.say_voice("I am unable to reach my goal for some reason. Please help me a little bit and press the green button to resume the test.", 3.0)
                            if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Please help me a little bit and press the green button." )
                            self.wait_for_green_button()
                            num_attemps += 1
                        else:
                            self.say_voice("I am unable to reach my goal. Please take me to " + n1_data[1] + " and press the green button to resume the test.", 3.0)
                            if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Please take me to " + n1_data[1] + " and press the green button." )
                            self.wait_for_green_button()
                            num_attemps += 1

                    if self.cancel_request:
                        self.close_and_return()
                        return
                #end-while

                # Save result
                r2 = dict()
                r2["from"] = n2_data
                r2["to"] = n1_data
                r2["path"] = self.robot_pose_history
                r2["dist"] = self.computePathLength(self.robot_pose_history)
                r2["time"] = (self.robot_pose_history[-1].header.stamp - self.robot_pose_history[0].header.stamp).to_sec()
                r2["success"] = nav_completed
                r2["attemps"] = num_attemps
                self.nav_results.append(r2)
                if self.verbose:
                    self.print_result([self.nav_results[-1]])

                n2 += 1
            #end while n2
            n1 += 1            

            # MOVE to n1 (no recording)
            #--------------------------
            if n1 < len(self.nodes)-1:
                self.say_voice("Phase " + str(n1) +" Completed", 1.0)
                n1_data = self.nodes[n1].split()  # n = [id, label, type, x, y, yaw]
                nav_completed = False
                num_attemps = 1
                self.robot_pose_history = []
                while (not nav_completed) and (num_attemps <= max_attemps):
                    self.say_voice("To continue the Navigation Test I need to go to " + n1_data[1] + " . Attemp " + str(num_attemps), 2.0)
                    if self.verbose: rospy.loginfo( "[Evaluate_Navigation] To continue the Navigation Test I need to go to " + n1_data[1] + " . Attemp " + str(num_attemps) )
                    nav_completed = self.navigate_to_node(n1_data, False)

                    if self.cancel_request:
                        self.close_and_return()
                        return

                    if nav_completed:
                        self.say_voice("Navigation comleted with success.", 3.0)
                    else:
                        # Request User help
                        if num_attemps < max_attemps:
                            self.say_voice("I am unable to reach my goal for some reason. Please help me a little bit and press the green button to resume the test.", 3.0)
                            if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Please help me a little bit and press the green button." )
                            self.wait_for_green_button()
                            num_attemps += 1
                        else:
                            self.say_voice("I am unable to reach my goal. Please take me to " + n1_data[1] + " and press the green button to resume the test.", 3.0)
                            if self.verbose: rospy.loginfo( "[Evaluate_Navigation] Please take me to " + n1_data[1] + " and press the green button." )
                            self.wait_for_green_button()
                            num_attemps += 1

                    if self.cancel_request:
                        self.close_and_return()
                        return
                #end-while
            #end-if

        #end while n1


        # PRINT RESULTS
        self.print_result(self.nav_results)
        self.say_voice("The Navigation test has finished. Setting Critial Navigation Points Automatically", 3.0)


        # PROPOSE CRITICAL POINTS
        self.propose_CNP()



    # =============================================================
    # ================       PROPOSE CNP           ================
    # =============================================================
    def propose_CNP(self):
        # Navigation Cost (Costmaps are range 0-100)
        max_cost = max(self.nav_map.data)

        high_cost_points = []
        cnp_inventory = []
        idx = 0
        for cost in self.nav_map.data:
            if cost == max_cost:
                # Get pose in the map (x,y)[m]
                P = self.cell_to_pose(idx)

                # Check if valid point
                valid_point = True
                for j in high_cost_points:
                    if math.sqrt(pow(P[0]-j[0], 2) + pow(P[1]-j[1], 2)) <= 0.2: #distance in meters
                        valid_point = False

                if valid_point:
                    # Add valid point
                    high_cost_points.append(P)
                    rospy.loginfo("New Seed to CNP at pose (x=%.3f, y=%.3f)[m]", P[0], P[1] )

                    # Get costmap as img arround cell idx
                    img, top_left_idx = self.crop_costmap_as_img(idx)

                    # Set CNP in the close vecinity
                    Pimg = self.get_CNP_from_img(img)

                    if Pimg is not None:
                        #Pcnp is given in pixels of the cropped-img
                        # Get Coordinates in meters to add a CNP
                        rospy.loginfo("New CNP at pixles (x=%d, y=%d)[px] of the Cropped Image", int(Pimg[0]), int(Pimg[1]) )

                        idx_cnp = int(top_left_idx -int(Pimg[1])*self.nav_map.info.width + int(Pimg[0]))
                        Pcnp = self.cell_to_pose(idx_cnp)
                        rospy.loginfo("New CNP at cell=%d with pose (x=%.3f, y=%.3f)[m]", idx_cnp, Pcnp[0], Pcnp[1] )

                        # Check if cnp already set
                        valid_cnp = True
                        for h in cnp_inventory:
                            if math.sqrt(pow(Pcnp[0]-h[0], 2) + pow(Pcnp[1]-h[1], 2)) <= 0.2: #distance in meters
                                valid_cnp = False

                        if valid_cnp:
                            cnp_inventory.append(Pcnp)

                            # Add CNP
                            Pose_cnp = PoseStamped()
                            Pose_cnp.pose.position.x = Pcnp[0]
                            Pose_cnp.pose.position.y = Pcnp[1]
                            Pose_cnp.pose.position.z = 0.0
                            Pose_cnp.pose.orientation.w = 1-0
                            res = self.navAssist_srv_proxy(action="add", type="CNP", pose=Pose_cnp)

            # end if max-cost
            idx = idx +1


    # =============================================================
    # ================         CROP IMAGE          ================
    # =============================================================
    def crop_costmap_as_img(self, idx):

        while not self.has_global_costmap:
            rospy.sleep(0.5);

        # Transform Global Costmap to cv image
        width = self.global_costmap.info.width
        height = self.global_costmap.info.height
        resolution = self.global_costmap.info.resolution
        length = len(self.global_costmap.data)

        #creat a mat to load costmap
        #img = np.zeros([height,width])
        img = np.zeros((height,width,3), np.uint8)
        for i in range(1,height):
            for j in range(1,width):
                #img[i-1, j-1] = 255- int( float(self.global_costmap.data[(i-1)*width+j])/100.0*255.0 )
                if self.global_costmap.data[(i-1)*width+j] < 95:
                    img[height-i-1, j-1] = 0
                else:
                    img[height-i-1, j-1] = 255

        # Matrix ref-system is top-left (but idx is in the costmap ref that is bottom-left)
        cell_r = int(height - math.floor(idx / width))
        cell_c = int(idx % width)
        # Get pose in the map (x,y)[m]
        P = self.cell_to_pose(int(idx))
        rospy.loginfo("idx=%d corresponds to pose (x=%.3f, y=%.3f)[m]", int(idx), P[0], P[1] )

        # Display in CV
        img2 = copy.deepcopy(img)
        img2 = cv.circle(img2,( int(cell_c), int(cell_r)), 4, (255,0,255), -1)
        self.showImageAndWait(img2, "full costmap-image")

        # CROP around this point
        s = 20
        crop_img = img[int(cell_r-s):int(cell_r+s), int(cell_c-s):int(cell_c+s)]
        self.showImageAndWait(crop_img, "cropped image")

        # Return values
        top_left_idx = int(height-(cell_r-s))*self.nav_map.info.width + int(cell_c-s)
        return crop_img, top_left_idx



    # =============================================================
    # ================       get_CNP_from_img      ================
    # =============================================================
    def get_CNP_from_img(self, img):
        show_image = True

        # 1. Grayscale
        #--------------------
        gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

        # 2. Contours
        #--------------------
        im2, contours, hierarchy = cv.findContours(gray, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)


        # 3. Select the 2 longests Contours
        #-----------------------------------
        height, width, channels = img.shape
        longest_contours = []
        for c in contours:
            if len(longest_contours) == 0:
                # first contour
                longest_contours.append(c)
            elif len(longest_contours) == 1:
                # add contour in order of length
                if len(c) > len(longest_contours[0]):
                    longest_contours.append(c)
                else:
                    longest_contours.insert(0,c)
            else:
                # keep only the two longests contours
                if len(c) > len(longest_contours[0]):
                    # remove shortest contour (at [0])
                    longest_contours.pop(0)

                    # add contour in order of len
                    if len(c) > len(longest_contours[0]):
                        longest_contours.append(c)
                    else:
                        longest_contours.insert(0,c)

        # feedback
        if show_image:
            drawing = None
            drawing = copy.deepcopy(gray)
            fix_color = ( (200, 200, 100), (150, 250, 100) )
            # draw contours
            n = 0
            for c in longest_contours:
                for i in c:
                    drawing = cv.circle(drawing, tuple(i[0].astype(int)), 2, fix_color[n], -1)
                n = n+1

            self.showImageAndWait(drawing,'longests_contours')

        if len(longest_contours) != 2:
            return None


        # 4. Filter Contours
        #--------------------
        filtered_contours = [[],[]]
        n = 0
        for c in longest_contours:
            for i in c:
                if i[0][0]>2 and i[0][0]<width-2 and i[0][1]>2 and i[0][1]<width-2:
                    filtered_contours[n].append(i[0])
            n = n+1

        # feedback
        drawing = None
        if show_image:
            drawing = copy.deepcopy(img)
            fix_color = ( (200, 200, 100), (150, 250, 100) )
            # draw contours
            n = 0
            for c in filtered_contours:
                for i in c:
                    drawing = cv.circle(drawing, tuple(i.astype(int)), 2, fix_color[n], -1)
                n = n+1

            self.showImageAndWait(drawing,'filtered_contours')

        use_ransac_circles = False
        P = []
        if use_ransac_circles:
            # 5. get circle fitting with RANSAC
            #-----------------------------------
            centres = []
            radius = []
            for c in filtered_contours:
                cen_x, cen_y, rad = self.findCentre_ransac(c)
                centres.append( np.array([cen_x,cen_y, 1]) )
                radius.append(rad)

            # vector from centre to centre
            vcentres = np.abs(centres[0] - centres[1])
            # euclidean distance
            d_centres = np.sqrt(np.dot(vcentres[0:2].T, vcentres[0:2]))
            # min distance as the diff
            min_d = d_centres - radius[0] - radius[1]


            # 6. Set CRITICAL POINT (geometric relations)
            #--------------------------------------------
            d = radius[0] + min_d/2.0
            v = centres[1] - centres[0]
            P = centres[0][0:2] + (v[0:2]*d/d_centres)

            # feedback
            if show_image:
                im_result = copy.deepcopy(drawing)
                # Circles
                im_result = cv.circle(im_result,tuple(centres[0][0:2].astype(int)),int(radius[0]),(0,0,255),2)
                im_result = cv.circle(im_result,tuple(centres[1][0:2].astype(int)),int(radius[1]),(0,0,255),2)
                # Line centers
                im_result = cv.line(im_result, (int(centres[0][0]), int(centres[0][1])), ( int(centres[1][0]),int(centres[1][1]) ) ,(0,0,255),2)
                # RESULT
                im_result = cv.circle(im_result,( int(P[0]), int(P[1])), 4, (255,0,255), -1)
                self.showImageAndWait(im_result)

        else:
            # Use ineficient loop over all points!
            min_d_px = None
            a_px = []
            b_px = []
            for i in filtered_contours[0]:
                for j in filtered_contours[1]:
                    d_px = math.sqrt(pow(j[0]-i[0], 2) + pow(j[1]-i[1], 2))
                    if min_d_px == None:
                        min_d_px = d_px
                        a_px = i
                        b_px = j
                    elif d_px < min_d_px:
                        min_d_px = d_px
                        a_px = i
                        b_px = j

            P = a_px + (b_px-a_px)*0.5

            # feedback
            if show_image:
                im_result = copy.deepcopy(drawing)
                # Closest points
                im_result = cv.circle(im_result,tuple(a_px.astype(int)),2,(0,0,255),2)
                im_result = cv.circle(im_result,tuple(b_px.astype(int)),2,(0,0,255),2)

                # RESULT
                im_result = cv.circle(im_result,( int(P[0]), int(P[1])), 4, (255,0,255), -1)
                self.showImageAndWait(im_result)

        # P is in pixels and in the "cropped-img" ref-system
        return P



    def findCentre_ransac(self, c):
        # Approximates the contour c to a circle and get its centre and radius
        # make ransac class. [n]: how many times try sampling
        cx = []
        cy = []
        for i in c:
            cx.append(i[0])
            cy.append(i[1])

        ransac = RANSAC(cx, cy, 10)

        # execute ransac algorithm
        ransac.execute_ransac()

        # get best model from ransac
        a, b, r = ransac.best_model[0], ransac.best_model[1], ransac.best_model[2]
        return a, b, r


    # ====================================================== #
    #                  PRINT RESULT                          #
    # ====================================================== #
    def print_result(self, result_list):
        rospy.loginfo("")
        for r in result_list:
            rospy.loginfo("NavResult-> From(%s) To(%s) Dist(%.3f) Time(%.3f) Success(%d) Attemps(%d)",
                          r["from"][1], r["to"][1], r["dist"], r["time"], r["success"], r["attemps"] )



    # ====================================================== #
    #               SAVE PATH PLANNING                       #
    # ====================================================== #
    def save_path(self, n1_data, n2_data):
        # Save the PathPlan : Get Plan
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x = float(n1_data[3])
        start.pose.position.y = float(n1_data[4])
        start.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, float(n1_data[5]))
        start.pose.orientation = Quaternion(*quat)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(n2_data[3])
        goal.pose.position.y = float(n2_data[4])
        goal.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, float(n2_data[5]))
        goal.pose.orientation = Quaternion(*quat)

        p = self.make_plan_srv_proxy(start = start, goal = goal, tolerance = 0.0)
        self.navigation_plans.append( p.plan )


    def distanceBetweenPoses(self, pose1, pose2):
        """Compute the euclidian distance between 2 poses"""
        return math.sqrt(pow(pose2.position.x-pose1.position.x, 2) +
                         pow(pose2.position.y-pose1.position.y, 2))

    def computePathLength(self, poses):
        """Compute the length path from the list of geometry_msgs/PoseWithCovarianceStamped"""
        pathLength = 0.0
        # Iteration among along the poses in order to compute the length
        for index in range(1, len(poses)):
            pathLength += self.distanceBetweenPoses(poses[index-1].pose.pose, poses[index].pose.pose)
        return pathLength


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
        if self.verbose: rospy.loginfo("[Evaluate Navigation] Navigation requested. Waiting for navigation to complete...")
        while not self.nav_client.wait_for_result(rospy.Duration(2.0)):
            if self.cancel_request:
                self.close_and_return()
                break


        # Stop tracking
        self.record_robot_pose = False
        rospy.sleep(0.5)

        # Prints out the result of executing the action
        if self.nav_client.get_state() == 3:
            if self.verbose: rospy.loginfo("[Evaluate Navigation] Navigation succes.")
            return True
        else:
            if self.verbose: rospy.loginfo("[Evaluate Navigation] Navigation failed.")
            return False



    # ====================================================== #
    #                   BUTTON CALLBACK                      #
    # ====================================================== #
    def buttons_msg_cb(self, joy_msg):
        # New button_pressed! -> order is: (red, greeen, dial, e-stop)
        self.button_pressed = True
        self.last_button_state = joy_msg

        if self.last_button_state.buttons[0] == 1:
            # Cancel Navigation Test
            self.cancel_request = True


    # =============================================================
    # ================           GET THE MAP       ================
    # =============================================================
    def map_cb(self, map_msg):

        # Configure Navigation Costmap
        if not self.hasMap:
            self.nav_map.info = map_msg.info
            self.nav_map.data = [int(0)] * len(map_msg.data)

            dx = self.nav_map.info.origin.position.x
            dy = self.nav_map.info.origin.position.y
            self.T = np.array([ (1,0,dx), (0,1,dy), (0,0,1) ])

            # Transform inverse
            self.T_inv = np.linalg.inv(self.T)

        self.hasMap = True


    # =============================================================
    # ================    GET THE GLOBAL COSTMAP   ================
    # =============================================================
    def globalCostMap_cb(self, map_msg):

        # Keep a copy
        #if not self.has_global_costmap:
        self.global_costmap = map_msg
        self.has_global_costmap = True


    # ====================================================== #
    #                    ODOM CALLBACK                       #
    # ====================================================== #
    def odomPoseCallback(self, msg):
        self.robot_speed = msg.twist.twist.linear.x


    # ====================================================== #
    #                    AMCL CALLBACK                       #
    # ====================================================== #
    def amclPoseCallback(self, msg):
        # geometry_msgs/PoseWithCovarianceStamped (time, frame_id, pose)
        self.robot_pose = msg

        if self.record_robot_pose:
            self.robot_pose_history.append( msg )
            self.update_navigation_map( msg )

        #quaternion = ( data.pose.pose.orientation.x, data.pose.pose.orientation.y,
        #data.pose.pose.orientation.z, data.pose.pose.orientation.w )
        #amclth = tf.transformations.euler_from_quaternion(quaternion)[2]



    # =============================================================
    # ================       UPDATE NAV MAP        ================
    # =============================================================
    def update_navigation_map(self, robotPose):
        # robotPose = PoseWithCovarianceStamped

        # Pose to Cell idx. map is a 1D list
        cellIdx = self.pose_to_cell(robotPose)

        # Navigation Cost (Costmaps are range 0-100)
        min_cost = 10
        max_cost = 95
        max_speed = rospy.get_param("/move_base/DWAPlannerROS/max_vel_x")

        # Linear Cost
        #nav_value = int(round( (min_cost-max_cost/max_speed)*self.robot_speed + max_cost ))
        # Exponential Cost
        nav_value = int(round( (max_cost-min_cost)*math.exp(-3*self.robot_speed/max_speed) + min_cost ))

        # Update 4 cells around the robot (keep min)
        self.nav_map.data[cellIdx] = max(nav_value, self.nav_map.data[cellIdx])
        self.nav_map.data[cellIdx-1] = max(nav_value, self.nav_map.data[cellIdx-1])
        self.nav_map.data[cellIdx+1] = max(nav_value, self.nav_map.data[cellIdx+1])
        self.nav_map.data[cellIdx-self.nav_map.info.width] = max(nav_value, self.nav_map.data[cellIdx-self.nav_map.info.width])
        self.nav_map.data[cellIdx+self.nav_map.info.width] = max(nav_value, self.nav_map.data[cellIdx+self.nav_map.info.width])

        # publish map
        if rospy.Time.now() - self.last_update_time > rospy.Duration(2.0):
            self.navMap_pub.publish(self.nav_map)
            self.last_update_time = rospy.Time.now()


    # =============================================================
    # ================       POSE TO CELL          ================
    # =============================================================
    def pose_to_cell(self, p):

        # p = PoseWithCovarianceStamped()

        # 1. Get pose of the robot in the gridmap ref system (in meters)
        pm = np.matmul(self.T_inv, np.array([p.pose.pose.position.x, p.pose.pose.position.y, 1.0]))

        # pm is in meters and transformed, convert to pixels
        pm[0] = int(round(pm[0]/self.nav_map.info.resolution))
        pm[1] = int(round(pm[1]/self.nav_map.info.resolution))

        cell = pm[0] + pm[1]*self.nav_map.info.width

        #rospy.loginfo("Pose (%.3f, %.3f) is cell %d of %d", p.pose.pose.position.x, p.pose.pose.position.y, int(cell), self.nav_map.info.width*self.nav_map.info.height )

        return int(cell)

    # =============================================================
    # ================       CELL TO POSE          ================
    # =============================================================
    def cell_to_pose(self, cell):
        # Cell is an idx(1D), convert to pixels (2D)
        cell_r = math.floor(cell / self.nav_map.info.width)
        cell_c = cell % self.nav_map.info.width
        #rospy.loginfo("Cell %d => col=%d, row=%d", int(cell), int(cell_c), int(cell_r) )

        # Convert to meters
        pc = cell_c * self.nav_map.info.resolution
        pr = cell_r * self.nav_map.info.resolution

        # Apply ref system transformation (matrix to map)
        pose_meters = np.matmul(self.T ,np.array([pc,pr,1.0]))

        #rospy.loginfo("Cell %d => x=%.3f[m], y=%.3f[m]", int(cell), pose_meters[0], pose_meters[1] )
        return pose_meters

    # ====================================================== #
    #                    CLOSE & RETURN                      #
    # ====================================================== #
    def close_and_return(self):

        # STOP THE ROBOT
        self.nav_client.cancel_all_goals()

        self.say_voice("The red button has been pressed. Canceling Navigation Test.", 1.0)
        return


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


    def showImageAndWait(self, img, name_window='random'):
        cv.imshow(name_window, img)
        cv.waitKey(0)
        cv.destroyAllWindows()

# =============================================================
# ==========================  MAIN  ===========================
# =============================================================
if __name__ == '__main__':
    navtest = NavTest()

