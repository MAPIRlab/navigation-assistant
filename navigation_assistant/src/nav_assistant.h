#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>      // std::ofstream
#include <math.h>       /* atan2 */
#include <json.hpp>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <navigation_assistant/nav_assistantAction.h>
#include <navigation_assistant/nav_assistant_point.h>
#include <navigation_assistant/nav_assistant_poi.h>
#include <navigation_assistant/nav_assistant_set_CNP.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <angles/angles.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// move_base (AS)
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base/move_base.h>
#include <nav_msgs/GetPlan.h>

// topology_graph
#include <topology_graph/graph.h>
#include <navigation_assistant/make_plan.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client;
using namespace std;
using json = nlohmann::json;

class CNavAssistant
{
protected:
    ros::NodeHandle nh_;
    std::string action_name_;

    // Offer an Action Server (Navigation)
    actionlib::SimpleActionServer<navigation_assistant::nav_assistantAction> as_;
    navigation_assistant::nav_assistantResult result_;
    tf::TransformListener tf_listener;

    // Offer a Service (Add navigation goals)
    ros::ServiceServer service;
    bool srvCB(navigation_assistant::nav_assistant_point::Request &req, navigation_assistant::nav_assistant_point::Response &res);
    bool addNode(std::string node_label, std::string node_type, double pose_x, double pose_y, double pose_yaw);
    bool deleteNode(std::string node_type, double pose_x, double pose_y, double pose_yaw);

public:
    CNavAssistant(std::string name);
    ~CNavAssistant(void);
    void executeAS(const navigation_assistant::nav_assistantGoalConstPtr &goal);

private:
    //subscriber to Robot localization
    ros::Subscriber localization_sub_;
    void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    geometry_msgs::PoseWithCovarianceStamped current_robot_pose;

    // Make:plan service client (to estimate paths)
    ros::ServiceClient mb_srv_client;
    ros::ServiceClient graph_srv_client;
    ros::ServiceClient nav_assist_functions_client, nav_assist_functions_client2;
    ros::ServiceServer makePlanServer;

    bool makePlan(navigation_assistant::make_plan::Request& request, navigation_assistant::make_plan::Response& response);

    // cmd_vel Publisher
    ros::Publisher cmd_vel_publisher, ready_publisher;

    //MoveBase Action Client
    mb_client mb_action_client;
    void mb_done_cb(const actionlib::SimpleClientGoalState& state,
                  const move_base_msgs::MoveBaseActionResultConstPtr &result);


    // functions
    void close_and_return();
    void turn_towards_path(geometry_msgs::PoseStamped pose_goal);
    void move_base_nav_and_wait(geometry_msgs::PoseStamped pose_goal);
    bool move_base_cancel_and_wait(double wait_time_sec);
    void get_graph_data_from_json(json json_msg);
    void regenerate_arcs();

    //Parameters
    bool verbose;                   //true/false
    bool init_from_param, load_passages_as_CP, force_CP_as_additional_ANP;
    std::string topology_parameter, init_from_file, save_to_file;
    std::string topology_str;
    int counter;
};
