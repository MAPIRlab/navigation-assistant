#include <ros/ros.h>
#include "topology_graph/ahgraph.h"
#include "topology_graph/graph.h"
#include <json.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/algorithm/string.hpp>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_listener.h>

#include <string.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

using namespace std;
using json = nlohmann::json;

class CGraphWrapper
{
public:
    CGraphWrapper();
    ~CGraphWrapper();
    void publishGraphAsMarkers();
    double marker_lifespam;

protected:
    ros::NodeHandle n;
    bool verbose;

    // The graph!
    CAHGraph my_graph;

    // Robot localization
    ros::Subscriber localization_sub;
    void localizationCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    geometry_msgs::PoseWithCovarianceStamped current_robot_pose;

    // Make:plan service client
    ros::ServiceClient mb_srv_client;    

    // Service offered
    ros::ServiceServer service;
    bool srvCB(topology_graph::graph::Request &req, topology_graph::graph::Response &res);

    // Path tools
    //bool check_if_direct_path(double start_x, double start_y, double end_x, double end_y, double &nav_dist);
    double get_nav_distance_two_poses(geometry_msgs::Pose pose_origin, geometry_msgs::Pose pose_goal, std::vector<std::string> avoiding_node_types);
    bool segment_intersect_node(geometry_msgs::Pose segment_ini, geometry_msgs::Pose segment_end, std::vector<std::string> sp_list);
    bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
        float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);

    // RViz stuff
    void DrawNode(size_t id, string node_label, string node_type, double x, double y, double yaw);
    void DrawArc(double from_x, double from_y, double to_x, double to_y);
    void DrawSegment_intersection(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2);

    int marker_id;
    ros::Publisher marker_pub, path_pub;
    visualization_msgs::MarkerArray graphMarkerList;
    
    // debug
    std::vector<nav_msgs::Path> free_paths;
    int num_path;
};
