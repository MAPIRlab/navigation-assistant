#include <rclcpp/rclcpp.hpp>
#include "topology_graph/ahgraph.h"
#include "topology_graph/srv/graph.hpp"
#include <topology_graph/json/json.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <boost/algorithm/string.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <tf2_ros/transform_listener.h>

#include <string.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using json = nlohmann::json;

class CGraphWrapper : public rclcpp::Node
{
public:
    CGraphWrapper();
    ~CGraphWrapper();
    void publishGraphAsMarkers();
    double marker_lifespam;

protected:
    bool verbose;

    // The graph!
    CAHGraph my_graph;

    // Robot localization
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub;
    void localizationCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    geometry_msgs::msg::PoseWithCovarianceStamped current_robot_pose;

    // Make:plan service client
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr mb_srv_client;    

    // Service offered
    rclcpp::Service<topology_graph::srv::Graph>::SharedPtr service;
    bool srvCB(topology_graph::srv::Graph::Request::SharedPtr req, topology_graph::srv::Graph::Response::SharedPtr res);

    // Path tools
    //bool check_if_direct_path(double start_x, double start_y, double end_x, double end_y, double &nav_dist);
    double get_nav_distance_two_poses(geometry_msgs::msg::Pose pose_origin, geometry_msgs::msg::Pose pose_goal, std::vector<std::string> avoiding_node_types);
    bool segment_intersect_node(geometry_msgs::msg::Pose segment_ini, geometry_msgs::msg::Pose segment_end, std::vector<std::string> sp_list);
    bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
        float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);

    // RViz stuff
    void DrawNode(size_t id, string node_label, string node_type, double x, double y, double yaw);
    void DrawArc(double from_x, double from_y, double to_x, double to_y);
    void DrawSegment_intersection(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point q1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point q2);

    int marker_id;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    visualization_msgs::msg::MarkerArray graphMarkerList;
    
    // debug
    std::vector<nav_msgs::msg::Path> free_paths;
    int num_path;
};
