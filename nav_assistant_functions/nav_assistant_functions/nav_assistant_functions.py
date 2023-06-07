#! /usr/bin/env python

import rclpy
from time import sleep
from rclpy.node import Node
from nav_assistant_msgs.srv import NavAssistantPOI, NavAssistantSetCNP
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData

import copy
import numpy as np
from numpy.linalg import inv
import matplotlib
import matplotlib.pyplot as plt
from math import cos, sin, radians, degrees, sqrt, atan2
import cv2 as cv
import sys

class InternalMap:
    info : MapMetaData
    data : np.ndarray


class nav_assist_functions(Node):
    # ---------------------------------------------------------------------
    #                                INIT
    # ---------------------------------------------------------------------
    def __init__(self):
        super().__init__('nav_assistant_functions')
        
        # Read params

        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value


        # Subscribers to MAP
        self.map_sub =  self.create_subscription( OccupancyGrid, "/map", self.map_cb, 10)
        self.currentMap = InternalMap()
        self.hasMap = False

        # subscribe to Global Costmap
        self.costmap_sub = self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.gcostmap_cb, 10)
        self.currentCostMap = InternalMap()
        self.has_global_costmap = False


        # Wait till map is available
        while not self.hasMap:
            self.get_logger().info( "Waiting to get the occupancy Map of the environment." )
            rclpy.spin_once(self, executor=None, timeout_sec=1)

        # Wait till CostMap is available
        while not self.has_global_costmap:
            self._logger.info( "Waiting to get the Global CostMap." )
            rclpy.spin_once(self, executor=None, timeout_sec=1)


        # Advertise services
        self.srv_poi = self.create_service(NavAssistantPOI, 'navigation_assistant/get_poi_related_poses', self.handle_new_poi)
        self.srv_cnp = self.create_service(NavAssistantSetCNP, 'navigation_assistant/get_cnp_pose_around', self.handle_new_cnp)
        self.get_logger().info( "Config done... LOOPING" )
        


    # =============================================================
    # ================           GET THE MAP       ================
    # =============================================================
    def map_cb(self, map_msg : OccupancyGrid):
        self._logger.info(f"Received map. Type {type(map_msg)}")
        self.hasMap = True

        self.currentMap.info = map_msg.info 
        # Reshape map to numpy
        #try:

        self.currentMap.data = np.array(map_msg.data).reshape( (self.currentMap.info.height, self.currentMap.info.width) )
        self.currentMap.data = np.flipud(self.currentMap.data)
        #except:
        #    self._logger.error( f"[NavAssitant-map_cb] Unexpected error when reshaping the MAP:{sys.exc_info()[0]}" )

        #print(type(self.currentMap.data))
        #print(self.currentMap.data.shape)
        #np.savetxt("/home/jgmonroy/mapa_paco_numpy.txt", self.currentMap.data, fmt='%d')

        #fig = plt.figure()
        #imgplot = plt.imshow(self.currentMap.data, cmap="Greys")
        #plt.show()



    # =============================================================
    # ================        GET THE COSTMAP      ================
    # =============================================================
    def gcostmap_cb(self, costmap_msg : OccupancyGrid):
        self.has_global_costmap = True

        self.currentCostMap.info = costmap_msg.info
        # Reshape map to numpy
        try:
            self.currentCostMap.data = np.array(costmap_msg.data).reshape( (self.currentCostMap.info.height, self.currentCostMap.info.width) )
            self.currentCostMap.data = np.flipud(self.currentCostMap.data)
        except:
            self._logger.error(f"[NavAssitant-gcostmap_cb] Unexpected error when reshaping the MAP:{sys.exc_info()[0]}")


    # =============================================================
    # ================    NEW POINT OF INTEREST (SRV CALL)  =======
    # =============================================================    
    def handle_new_poi(self,req, res):
        self._logger.info(" Setting SPs and INGs for given CNP/CP" )

        # Get Auxiliary points
        try:
            aux_points = self.get_auxiliary_nodes([req.pose.pose.position.x, req.pose.pose.position.y]);
        except:
            self._logger.error(f"[NavAssitant-get_auxiliary_nodes] Unexpected error:{sys.exc_info()[0]}")
            res.success = False
            return res

        # Prepare output
        if aux_points is not None:
            res.success = True
            try:
                # aux_points [ [[sp1x,sp1y],[sp2x,sp2y],[ing1x,ing1y],[ing2x,ing2y]], r, [line] ]
                # SP points
                res.sp = []
                mypose = PoseStamped()
                mypose.header.stamp = rclpy.Time.now()
                mypose.header.frame_id = "map"
                mypose.pose.position.x = aux_points[0][0][0]
                mypose.pose.position.y = aux_points[0][0][1]
                mypose.pose.position.z = 0.0
                mypose.pose.orientation =  Quaternion()
                res.sp.append(mypose)

                mypose2 = PoseStamped()
                mypose2.header.stamp = rclpy.Time.now()
                mypose2.header.frame_id = "map"
                mypose2.pose.position.x = aux_points[0][1][0]
                mypose2.pose.position.y = aux_points[0][1][1]
                mypose2.pose.position.z = 0.0
                mypose2.pose.orientation =  Quaternion()
                res.sp.append(mypose2)


                # ING points
                res.ing = []
                mypose3 = PoseStamped()
                mypose3.header.stamp = rclpy.Time.now()
                mypose3.header.frame_id = "map"
                mypose3.pose.position.x = aux_points[0][2][0]
                mypose3.pose.position.y = aux_points[0][2][1]
                mypose3.pose.position.z = 0.0
                mypose3.pose.orientation =  Quaternion()
                res.ing.append(mypose3)

                mypose4 = PoseStamped()
                mypose4.header.stamp = rclpy.Time.now()
                mypose4.header.frame_id = "map"
                mypose4.pose.position.x = aux_points[0][3][0]
                mypose4.pose.position.y = aux_points[0][3][1]
                mypose4.pose.position.z = 0.0
                mypose4.pose.orientation =  Quaternion()
                res.ing.append(mypose4)
            except:
                self._logger.error(f"[NavAssitant-handle_new_poi] Unexpected error:{sys.exc_info()[0]}")
                res.success = False
                return res
        else:
            res.success = False

        # Done
        return res


    # =============================================================
    # ============    GET CNP POSE around Point (SRV call)  =======
    # =============================================================
    def handle_new_cnp(self, req, res):
        self._logger.info(" Setting pose of CNP/CP around given pose." )

        # Init values
        resolution = self.currentCostMap.info.resolution                            #[m/cell]
        height = self.currentCostMap.info.height                                    #[cell]
        width = self.currentCostMap.info.width                                      #[cell]
        # Work with the reference system as in Matrix (top-left) = (0,0)
        Dx = self.currentCostMap.info.origin.position.x                             #[m]
        Dy = height*resolution - abs(self.currentCostMap.info.origin.position.y)    #[m]

        try:
            # Get global costmap as img arround given location
            T = np.array([ (1,0,0,Dx), (0,-1,0,Dy), (0,0,-1,0), (0,0,0,1) ])
            seed_px = self.meters_to_pixels(T, [req.pose.pose.position.x, req.pose.pose.position.y], resolution)
            img = self.crop_costmap_as_img( seed_px[1], seed_px[0] )

            # get optimal CNP location
            Pimg = self.get_CNP_from_img(img)

            if Pimg is not None:
                # Pimg is the location in pixels [in the cropped-img system]

                #if self.verbose: self._logger.info(" New CNP at pixles (x=%d, y=%d)[px] of the Cropped Image", int(Pimg[0]), int(Pimg[1]) )
                height, width, channels = img.shape
                Ppx_x = seed_px[0] - int(height/2) + Pimg[0]
                Ppx_y = seed_px[1] - int(width/2) + Pimg[1]

                # Get Coordinates in meters in the "map" ref system
                Pcnp = self.pixels_to_meters(T, [Ppx_x,Ppx_y], resolution)
                self._logger.info(" New CNP at pose (x=%.3f, y=%.3f)[m]",  Pcnp[0], Pcnp[1] )

                # Done
                mypose = PoseStamped()
                mypose.header.stamp = rclpy.Time.now()
                mypose.header.frame_id = "map"
                mypose.pose.position.x = Pcnp[0]
                mypose.pose.position.y = Pcnp[1]
                mypose.pose.position.z = 0.0
                mypose.pose.orientation =  Quaternion()
                res.pose = mypose
                res.success = True
                return res
            else:
                self._logger.info("Could not set CNP at pose (x=%.3f, y=%.3f)[m]",  Pcnp[0], Pcnp[1] )

                # CNP already exist or we are unable to set it properly
                res.success = False
                return res
        except:
            self._logger.error( f"[NavAssitant-handle_new_cnp] Unexpected error:{sys.exc_info()[0]}")
            res.success = False
            return res

    # =============================================================
    # ================         CROP IMAGE          ================
    # =============================================================
    def crop_costmap_as_img(self, cell_r, cell_c):
        show_image = False

        while not self.has_global_costmap:
            rclpy.sleep(0.5);

        # Transform Global Costmap to cv image
        width = self.currentCostMap.info.width
        height = self.currentCostMap.info.height
        resolution = self.currentCostMap.info.resolution

        # Create an Image to load the costmap (binary)
        img = np.zeros((height,width,3), np.uint8)
        for i in range(height):
            for j in range(width):
                if self.currentCostMap.data[i,j] < 95:      #127
                    img[i,j] = 0
                else:
                    img[i,j] = 255


        # Display in CV
        if show_image:
            img2 = copy.deepcopy(img)
            img2 = cv.circle(img2,( int(cell_c), int(cell_r)), 4, (255,0,255), -1)
            self.showImageAndWait(img2, "full costmap-image")

        # CROP around this point
        s = 30      #px
        crop_img = img[int(cell_r-s):int(cell_r+s), int(cell_c-s):int(cell_c+s)]
        #self.showImageAndWait(crop_img, "cropped image")

        # Return values
        return crop_img



    # =============================================================
    # ================       get_CNP_from_img      ================
    # =============================================================
    def get_CNP_from_img(self, img):
        show_image = False

        # 1. Grayscale
        #--------------------
        gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

        # 2. Contours (list)
        #--------------------
        im2, contours_array, hierarchy = cv.findContours(gray, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

        # 3. Filter Contours (removing points in the borders of the img)
        #--------------------
        filtered_contours = []
        height, width, channels = img.shape
        min_contour_length = 5
        for c in contours_array:        # "c" is a np.ndarray
            contour = []
            for i in c:
                if i[0][0]>2 and i[0][0]<width-2 and i[0][1]>2 and i[0][1]<width-2:
                    contour.append(i[0])
            # end of c
            if len(contour) > min_contour_length:
                filtered_contours.append(contour)

        # feedback
        drawing = None
        if show_image:
            drawing = copy.deepcopy(img)
            fix_color = ( (200, 200, 100), (150, 250, 100) )
            # draw contours
            n = 0
            for c in filtered_contours:
                for i in c:
                    drawing = cv.circle(drawing, tuple(i.astype(int)), 1, fix_color[n], -1)
                n = 1 - n

            self.showImageAndWait(drawing,'filtered_contours')



        # 4. Select the 2 best Contours
        #-------------------------------
        selected_contours = []
        min_d = 0.0
        Pcenter = []
        for c in filtered_contours:
            if len(selected_contours) == 0:
                # first contour
                selected_contours.append(c)
            elif len(selected_contours) == 1:
                # add contour and get min_dist
                selected_contours.append(c)
                min_d, Pcenter = self.min_dist_two_contours(selected_contours[0], selected_contours[1])
            else:
                # keep only the two contours with min distance between them
                min_d0, Pcenter0 = self.min_dist_two_contours(selected_contours[0], c)
                min_d1, Pcenter1 = self.min_dist_two_contours(selected_contours[1], c)

                if min_d0 < min_d1:
                    if min_d0 < min_d:
                        # keep only [0] and [c]
                        selected_contours.pop(1)
                        selected_contours.append(c)
                        Pcenter = Pcenter0
                else:
                    if min_d1 < min_d:
                        # keep only [1] and [c]
                        selected_contours.pop(0)
                        selected_contours.append(c)
                        Pcenter = Pcenter1

        # feedback
        if show_image:
            drawing = None
            drawing = copy.deepcopy(gray)
            fix_color = ( (200, 200, 100), (150, 250, 100) )
            # draw contours
            n = 0
            for c in selected_contours:
                for i in c:
                    drawing = cv.circle(drawing, tuple(i.astype(int)), 2, fix_color[n], -1)
                n = n+1
            self.showImageAndWait(drawing,'longests_contours')


        # ASSERT we have 2 contours!
        if len(selected_contours) != 2:
            if self.verbose: self._logger.error(" Error detecting contours in costmap")
            return None
        else:
            return Pcenter


    # =============================================================
    # ================       MIN DIST CONTORURS    ================
    # =============================================================
    def min_dist_two_contours(self, c1, c2):
        # Use ineficient loop over all points!
        min_d_px = None
        a_px = []       # Point in C1
        b_px = []       # Point in C2

        for i in c1:
            for j in c2:
                d_px = sqrt(pow(j[0]-i[0], 2) + pow(j[1]-i[1], 2))
                if min_d_px == None:
                    min_d_px = d_px
                    a_px = i
                    b_px = j
                elif d_px < min_d_px:
                    min_d_px = d_px
                    a_px = i
                    b_px = j

        # Point in the middle of the min distance line.
        P = a_px + (b_px-a_px)*0.5
        return min_d_px, P


    # =============================================================
    # ================       SHOW IMG & WAIT       ================
    # =============================================================
    def showImageAndWait(self, img, name_window='random'):
        cv.imshow(name_window, img)
        cv.waitKey(0)
        cv.destroyAllWindows()


    # =============================================================
    # ================       METERS TO PIXEL       ================
    # =============================================================
    def meters_to_pixels(self, T, p, s):
        T_inv = np.linalg.inv(T)
        pm = np.matmul(T_inv,np.array([p[0],p[1],0,1]))
        # pm is in meters and transformed, convert to pixels
        pm[0] = int(round(pm[0]/s))
        pm[1] = int(round(pm[1]/s))
        return pm

    # =============================================================
    # ================       PIXEL TO METERS       ================
    # =============================================================
    def pixels_to_meters(self, T, p, s):
        # p is in pixels, convert to meters and transform
        lp = copy.deepcopy(p)
        lp[0] *= s
        lp[1] *= s
        ppx = np.matmul(T,np.array([lp[0],lp[1],0,1]))
        return ppx

    # =============================================================
    # ================    POINT WITH MIN COST      ================
    # =============================================================
    def get_min_cost(self, px_min, px_max, py_min, py_max):
        # Set x and y limits according to costmap
        if px_min < 0:
            px_min = 0
        if px_max > self.currentCostMap.info.width:
            px_max = self.currentCostMap.info.width
        if py_min < 0:
            py_min = 0
        if py_max > self.currentCostMap.info.height:
            py_max = self.currentCostMap.info.height

        # get point of min cost
        # more efficient first with heigh and then width because map information is in row-major order
        min_cost = 255
        found_px = px_min
        found_py = py_min
        for j in range(py_min, py_max):
            for i in range(px_min, px_max):
                if self.currentCostMap.data[j][i] < min_cost and self.currentMap.data[j][i] == 0:
                    min_cost = self.currentCostMap.data[j][i]
                    found_px = i
                    found_py = j
            #end-for
        #end-for

        return [found_px, found_py]



    def get_min_cost2(self, px_ideal, py_ideal, slope):
        WINDOW_H_WIDTH = 4      # +/- px
        WINDOW_H_HEIGHT = 5     # +/- px
        # slope corresponds to the slop of the two points of the door.

        # get point of min cost
        # more efficient first with heigh and then width because map information is in row-major order
        min_cost = 255
        found_px = px_ideal
        found_py = py_ideal
        for j in range(-WINDOW_H_HEIGHT, WINDOW_H_HEIGHT):
            for i in range(-WINDOW_H_WIDTH, WINDOW_H_WIDTH):
                nx = int(round(px_ideal + i*cos(slope) - j*sin(slope)))
                ny = int(round(py_ideal + i*sin(slope) + j*cos(slope)))
                if nx > self.currentCostMap.info.width or nx < 0 or ny > self.currentCostMap.info.height or ny < 0:
                    continue

                if self.currentCostMap.data[ny][nx] < min_cost and self.currentMap.data[ny][nx] == 0:
                    min_cost = self.currentCostMap.data[ny][nx]
                    found_px = nx
                    found_py = ny
            #end-for
        #end-for

        return [found_px, found_py]

    # =============================================================
    # ================    GET AUXILIARY POINTS     ================
    # =============================================================
    # 'map' is a string with the map filename, 'point' and 'origin' are in meters, 'scale' is in m/px
    # output: 2x passage points (m) + 2x perpendicullar points (m) + radius of the circunference (m) + line coefficients for the passage ()
    def get_auxiliary_nodes(self, point):
        # prepare output
        nodes = []

        # Init values
        scale = self.currentMap.info.resolution            #[m/cell]
        height = self.currentMap.info.height               #[cell]
        width = self.currentMap.info.width                 #[cell]

        # Work with the reference system as in Matrix (top-left) = (0,0)
        Dx = self.currentMap.info.origin.position.x                     #[m]
        Dy = height*scale - abs(self.currentMap.info.origin.position.y) #[m]
        img = self.currentMap.data

        """
        # read map, transform to grayscale and binarize it -- map in pgm format
        img_g = cv.imread(map,cv.IMREAD_GRAYSCALE)
        _,img = cv.threshold(img_g,5,255,cv.THRESH_BINARY)
        # cv.imshow('Image_BW',img)
        # cv.waitKey(0)

        # width and height in meters
        height,_ = img.shape[:2]
        ly = height*scale
        Dx = origin[0] # < 0
        Dy = ly - abs(origin[1])
        """

        # define the transformation to set px-> and m->px (different ref systems)
        #T = np.array([ (1,0,0,Dx), (0,1,0,Dy), (0,0,1,0), (0,0,0,1) ])
        T = np.array([ (1,0,0,Dx), (0,-1,0,Dy), (0,0,-1,0), (0,0,0,1) ])
        npoint = self.meters_to_pixels(T,point,scale)
        if self.verbose: print("Point in pixels: " + str(npoint[0:2]))

        # define some values
        TH = 30.0              # [deg] angle th to consider valid points
        ANGLE_STEP = 1.0
        SKIP_ANGLE = 90.0

        # radius
        # -- in meters
        INI_RADIUS_M = 0.1
        RADIUS_STEP_M = 0.01 #0.025
        END_RADIUS_M = 2.0
        if self.verbose: print("Radius to find SP [m] from " + str(INI_RADIUS_M) + " to " + str(END_RADIUS_M) + " by " + str(RADIUS_STEP_M) )

        # -- in pixels
        INI_RADIUS_PX = INI_RADIUS_M/scale
        RADIUS_STEP_PX = RADIUS_STEP_M/scale
        END_RADIUS_PX = END_RADIUS_M/scale
        N_RADIUS = (END_RADIUS_PX-INI_RADIUS_PX)/RADIUS_STEP_PX
        if self.verbose: print("Radius to find SP [px] from " + str(INI_RADIUS_PX) + " to " + str(END_RADIUS_PX) + " by " + str(RADIUS_STEP_PX))

        # robot
        # -- in meters
        DIAMETER_M = 2*rclpy.get_param("/move_base/global_costmap/robot_radius")
        INFLATION = 1.2
        if self.verbose: print("Robot radious[m]=%.3f, Inflation=%.3f" %(DIAMETER_M, INFLATION))

        # distance
        # -- in meters
        INI_DIST_M = 0.1
        DIST_STEP_M = 0.1
        END_DIST_M = INFLATION*DIAMETER_M
        if self.verbose: print("Distance to set ING [m] from " + str(INI_DIST_M) + " to " + str(END_DIST_M) + " by " + str(DIST_STEP_M) )

        # -- in pixels
        INI_DIST_PX = INI_DIST_M/scale
        DIST_STEP_PX = DIST_STEP_M/scale
        END_DIST_PX = END_DIST_M/scale
        N_DIST = (END_DIST_PX-INI_DIST_PX)/DIST_STEP_PX
        if self.verbose: print("Distance to set ING [px] from " + str(INI_DIST_PX) + " to " + str(END_DIST_PX) + " by " + str(DIST_STEP_PX))

        #-----------
        # main loop
        #-----------
        pcx, pcy = 0, 0
        found = []
        for r in np.linspace( INI_RADIUS_PX, END_RADIUS_PX, N_RADIUS ):
            if self.verbose: print("\nLooking SP with Radius = %.3f[m] = %.3f[px] " %(r*scale, r))

            if len(found) >= 1:
                found = [found[0]]    #remove sencond candidate

            # START increasing the angle
            ang = 1
            while ang < 360.0:
                cx = int(round(npoint[0] + r*cos(radians(ang))))
                cy = int(round(npoint[1] + r*sin(radians(ang))))

                if pcx == cx and pcy == cy:
                    ang += ANGLE_STEP
                    continue

                pcx = copy.deepcopy(cx)
                pcy = copy.deepcopy(cy)

                # Ensure not out of bounds
                if cx > width or cy > height:
                    if self.verbose: print("DATA OUT OF IMAGE.. IGNORING")
                    ang += ANGLE_STEP
                    continue

                # Check Cell Occupancy: range [0,100].  Unknown is -1.
                if img[cy][cx] != 100:
                    # Free cell
                    ang += ANGLE_STEP
                    continue
                else:
                    # Occupied cell
                    v = [cx,cy,ang]
                    found.append(v)     # append the coordinates and angle
                    #if self.verbose: print("Found point at: %d,%d,%.3f" %(cx,cy,ang))

                    #ang += SKIP_ANGLE   # skip X degrees
                    ang += ANGLE_STEP


                if len(found) == 2:
                    # we have found a double match
                    # compute angle
                    u = found[0]
                    v = found[1]
                    an = abs(v[2]-u[2])

                    #print("Angle1=%.3f, Angle2=%.3f,|Diff|=%.3f, Check:%.3f"%( u[2],v[2],an,abs(an-180)))
                    if abs(an-180) <= TH:
                        # we have found the SP points (Segment Points)
                        line = np.cross( [u[0],u[1],1], [v[0],v[1],1] )

                        # convert points back to meters wrt the reference system
                        nu = self.pixels_to_meters(T,u,scale)
                        nodes.append(nu[0:2].tolist())

                        nv = self.pixels_to_meters(T,v,scale)
                        nodes.append(nv[0:2].tolist())
                        if self.verbose: print("\nSP found at: %3f,%3f" %(nu[0],nu[1]))
                        if self.verbose: print("SP found at: %3f,%3f\n" %(nv[0],nv[1]))


                        use_costmap = True
                        if use_costmap:
                            # Search the point with min costmap weight in a submap arround the ideal goal
                            # dv: director vector of the prependicular line
                            #if float(line[0]) < 0.01:
                            #    dv = [1.0, 1000]
                            #else:
                            #    dv = [1.0, float(line[1])/float(line[0])]
                            dv = [line[0],line[1]]

                            # Normalization
                            dv /= np.linalg.norm(dv)
                            # Slope of line
                            slope = atan2(-line[0],line[1])

                            # ING1
                            px_ideal = int(round(npoint[0] + END_DIST_PX*dv[0]))     #px
                            py_ideal = int(round(npoint[1] + END_DIST_PX*dv[1]))     #px

                            # Look for min cost in a square (sub-area) around the ideal point
                            if False:
                                W_SIZE_px = 8
                                px_min = int(round(px_ideal - W_SIZE_px))
                                px_max = int(round(px_ideal + W_SIZE_px))
                                py_min = int(round(py_ideal - W_SIZE_px))
                                py_max = int(round(py_ideal + W_SIZE_px))
                                ing = self.get_min_cost(px_min, px_max, py_min, py_max)
                            else:
                                ing = self.get_min_cost2(px_ideal,py_ideal,slope)

                            np1 = self.pixels_to_meters(T,ing,scale)
                            nodes.append(np1[0:2].tolist())
                            if self.verbose: print("\nING found at: %.3f,%.3f [m]" %(np1[0],np1[1]))

                            # ING2
                            px_ideal = int(round(npoint[0] - END_DIST_PX*dv[0]))     #px
                            py_ideal = int(round(npoint[1] - END_DIST_PX*dv[1]))     #px

                            # Look for min cost in a square (sub-area) around the ideal point
                            if False:
                                px_min = int(round(px_ideal - W_SIZE_px))
                                px_max = int(round(px_ideal + W_SIZE_px))
                                py_min = int(round(py_ideal - W_SIZE_px))
                                py_max = int(round(py_ideal + W_SIZE_px))
                                ing = self.get_min_cost(px_min, px_max, py_min, py_max)
                            else:
                                ing = self.get_min_cost2(px_ideal,py_ideal,slope)

                            np2 = self.pixels_to_meters(T,ing,scale)
                            nodes.append(np2[0:2].tolist())
                            if self.verbose: print("\nING found at: %.3f,%.3f [m]" %(np2[0],np2[1]))

                        else:
                            #-------------------------------------
                            # ING - Intermediate Navigation Goals
                            #-------------------------------------
                            # two new points perpendicular to SP and not occupied
                            # -- line passing through both SP points: ax + by + c = 0, with (a,b,c)=line
                            # -- perpendicular line passing through "passage" p0 = (px,py): (b/a)*x - y - (b/a)*px+py = 0
                            # -- director vector of perpendicular line: u = (1,b/a)
                            dv = [1.0, float(line[1])/float(line[0])]
                            dv /= np.linalg.norm(dv)

                            # points on that line at a distance "d" from p0: p1 = p0 + d*u and p2 = p0 - d*u
                            # iterate over "d" until a free point is found
                            found1, found2 = False, False

                            distance_range_normal = np.linspace( INI_DIST_PX, END_DIST_PX, N_DIST )
                            distance_range = distance_range_normal[::-1]
                            for d in distance_range:
                                #if self.verbose: print("Setting ING at distance %.3f[m] = %.3f[px]" %(d*scale, d))
                                if not found1:
                                    px1 = int(round(npoint[0] + d*dv[0]))
                                    py1 = int(round(npoint[1] + d*dv[1]))

                                    # Ensure not out of bounds
                                    if px1 > width or py1 > height:
                                        if self.verbose: print("ING point OUT OF IMAGE.. IGNORING")
                                    else:
                                        if img[py1][px1] == 0:
                                            # Free cell
                                            np1 = self.pixels_to_meters(T,[px1,py1],scale)
                                            nodes.append(np1[0:2].tolist())
                                            found1 = True
                                            if self.verbose: print("\nING found at: %.3f,%.3f [m] with d=%.3f" %(np1[0],np1[1], d))
                                            #print (dv)
                                            #print (line)

                                if not found2:
                                    px2 = int(round(npoint[0] - d*dv[0]))
                                    py2 = int(round(npoint[1] - d*dv[1]))

                                    if img[py2][px2] == 0:
                                        # Free cell
                                        np2 = self.pixels_to_meters(T,[px2,py2],scale)
                                        nodes.append(np2[0:2].tolist())
                                        found2 = True
                                        if self.verbose: print("ING found at: %3f,%3f[m] with d=%.3f\n" %(np2[0],np2[1], d))
                                        #print (dv)
                                        #print (line)

                                if found1 and found2:
                                    break

                            else:
                                print('Error [get_auxiliary_passage_nodes]: ING points could not be found!')
                                return None

                        # end-if use_costmap

                        # SUCCESS!!
                        return [nodes, r*scale, line.tolist()] # nodes and radius in meters, line is adimensional

                    else:
                        # Second point not valid!
                        found = [found[0]]    #remove sencond candidate

                    # end-if abs(an-180) <= TH:

                # end-if found(2)

            # end-while ang<360

        # end-for each circle-radious
        else:
            self._logger.error('[get_auxiliary_passage_nodes]: SP points not found!')
            return None

# =============================================================
# ==========================  MAIN  ===========================
# =============================================================
def main(args=None):
    rclpy.init(args=args)
    navfun = nav_assist_functions()
    rclpy.spin(navfun)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navfun.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
