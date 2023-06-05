#include <rclcpp/rclcpp.hpp>
#include <fstream>      // std::ofstream
#include <math.h>       /* atan2 */

// actionlib
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_assistant_msgs/action/nav_assistant.hpp>
#include <nav_assistant_msgs/srv/nav_assistant_point.hpp>
#include <nav_assistant_msgs/srv/nav_assistant_poi.hpp>
#include <nav_assistant_msgs/srv/nav_assistant_set_cnp.hpp>

// msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <angles/angles.h>

// move_base (AS)
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>

// topology_graph
#include "topology_graph/srv/graph.hpp"
#include <topology_graph/json/json.hpp>
#include <nav_assistant_msgs/srv/make_plan.hpp>

using namespace std;
using json = nlohmann::json;
namespace NAS=nav_assistant_msgs::srv;
namespace NAS_ac=nav_assistant_msgs::action;
typedef rclcpp_action::ServerGoalHandle<NAS_ac::NavAssistant> GoalHandleNavigate_Server;

typedef nav2_msgs::action::NavigateToPose NavToPose;
typedef rclcpp_action::ClientGoalHandle<NavToPose> NavToPoseClientGoalHandle;

typedef nav2_msgs::action::ComputePathToPose GetPlan;


class CNavAssistant : public rclcpp::Node
{
protected:
    std::string action_name_;

    // Offer an Action Server (Navigation)
    rclcpp_action::Server<NAS_ac::NavAssistant>::SharedPtr as_;

    // Offer a Service (Add navigation goals)
    rclcpp::Service<NAS::NavAssistantPoint>::SharedPtr service;
    bool srvCB(NAS::NavAssistantPoint::Request::SharedPtr req, NAS::NavAssistantPoint::Response::SharedPtr res);
    bool addNode(std::string node_label, std::string node_type, double pose_x, double pose_y, double pose_yaw);
    bool deleteNode(std::string node_type, double pose_x, double pose_y, double pose_yaw);

public:
    CNavAssistant(std::string name);
    ~CNavAssistant(void);
    void Init();
    std::shared_ptr<const GoalHandleNavigate_Server> m_activeServerGoalHandle;
    std::shared_ptr<NavToPoseClientGoalHandle> m_activeClientGoalHandle;

    void execute();
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NAS_ac::NavAssistant::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandleNavigate_Server> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNavigate_Server> goal_handle);

private:

    nav_msgs::msg::Path m_CurrentPlan;
    void getPlanCB(const rclcpp_action::ClientGoalHandle<GetPlan>::WrappedResult& w_result);

    bool goalCancelled=false;
    bool checkIfCancelled() 
    {
        if(goalCancelled)
        {
            RCLCPP_WARN(get_logger(), "Goal has been cancelled by the client");
            goalCancelled = false;
            return true;
        }
        return false;
    }

    void cancelCurrentGoal()
    {
        goalCancelled = true;
        m_activeServerGoalHandle = {nullptr};
    }

    //subscriber to Robot localization
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
    void localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    geometry_msgs::msg::PoseWithCovarianceStamped current_robot_pose;

    // Make:plan service client (to estimate paths)
    rclcpp::Client<topology_graph::srv::Graph>::SharedPtr graph_srv_client;
    rclcpp::Client<NAS::NavAssistantPOI>::SharedPtr nav_assist_functions_client_POI; 
    rclcpp::Client<NAS::NavAssistantSetCNP>::SharedPtr nav_assist_functions_client_CNP;
    
    rclcpp_action::Client<GetPlan>::SharedPtr getPlanClient;
    rclcpp::Service<NAS::MakePlan>::SharedPtr makePlanServer;

    bool makePlan(NAS::MakePlan::Request::SharedPtr request, NAS::MakePlan::Response::SharedPtr response);

    // cmd_vel Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher;

    //MoveBase Action Client
    rclcpp_action::Client<NavToPose>::SharedPtr mb_action_client;


    // functions
    void close_and_return();
    void turn_towards_path(geometry_msgs::msg::PoseStamped pose_goal);
    void move_base_nav_and_wait(geometry_msgs::msg::PoseStamped pose_goal);
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
