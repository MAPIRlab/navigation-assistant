#include "nav_assistant.h"
using json = nlohmann::json;
using namespace std::placeholders;

//-----------------------------------------------------------
//                    Subscriptions
//----------------------------------------------------------

// Robot location update
void CNavAssistant::localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // keep the most recent robot pose = position + orientation
    geometry_msgs::msg::PoseStamped msgPose;
    msgPose.header = msg->header;
    msgPose.pose = msg->pose.pose;
    current_robot_pose = transformPoseToFrame(msgPose, "map");
}

geometry_msgs::msg::PoseStamped CNavAssistant::transformPoseToFrame(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame)
{
    static tf2_ros::Buffer buffer(get_clock());
    static tf2_ros::TransformListener listener(buffer);
    using namespace std::chrono_literals;
    geometry_msgs::msg::PoseStamped result;
    try
    {
        result = buffer.transform(pose, target_frame, std::chrono::duration_cast<std::chrono::nanoseconds>(0.5s));
    }
    catch (const std::exception& e)
    {
        result = pose;
        RCLCPP_ERROR(get_logger(), "Error transforming pose: %s", e.what());
    }
    return result;
}

//-----------------------------------------------------------
//                    Action Server Initialization
//----------------------------------------------------------
CNavAssistant::CNavAssistant(std::string name) : Node("Nav_assistant_Server"), m_currentGoal(get_logger())
{
    /*! IMPORTANT
     * DEFINE all Action Servers to be used or published before executing the callback to avoid problems
     * */

    main_actionServer = rclcpp_action::create_server<NAS_ac::NavAssistant>(this, name, std::bind(&CNavAssistant::handle_goal, this, _1, _2),
                                                                           std::bind(&CNavAssistant::handle_cancel, this, _1),
                                                                           std::bind(&CNavAssistant::handle_accepted, this, _1));

    // Read config parameters
    verbose = declare_parameter<bool>("verbose", false);
    init_from_param = declare_parameter<bool>("init_from_param", false);
    topology_parameter = declare_parameter<std::string>("topological_json_parameter", "/topological_map");
    load_passages_as_CP = declare_parameter<bool>("load_passages_as_CP", false);
    force_CP_as_additional_ANP = declare_parameter<bool>("force_CP_as_additional_ANP", false);
    init_from_file = declare_parameter<std::string>("init_from_file", "");
    save_to_file = declare_parameter<std::string>("save_to_file", "");
    std::string localization_topic = declare_parameter<std::string>("localization_topic", "amcl_pose");

    makePlanServer = create_service<NAS::MakePlan>("navigation_assistant/make_plan", std::bind(&CNavAssistant::makePlan_async, this, _1, _2));

    // Service clients
    mb_action_client = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    getPlanClient = rclcpp_action::create_client<GetPlan>(this, "compute_path_to_pose");  // only accounts for global_costmap but works at all times
    graph_srv_client = create_client<topology_graph::srv::Graph>("topology_graph/graph"); // Graph service client
    nav_assist_functions_client_POI = create_client<NAS::NavAssistantPOI>("navigation_assistant/get_poi_related_poses");
    nav_assist_functions_client_CNP = create_client<NAS::NavAssistantSetCNP>("navigation_assistant/get_cnp_pose_around");

    // Subscribers
    localization_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(localization_topic, 1,
                                                                                           std::bind(&CNavAssistant::localizationCallback, this, _1));
    // Publishers
    cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    ready_publisher = create_publisher<std_msgs::msg::Bool>("nav_assistant/ready", 1);
    path_publisher = create_publisher<nav_msgs::msg::Path>("nav_assistant/Path", 1);

    // log some info
    {
        std::string _namespace = get_namespace();
        // handle the empty namespace
        if (_namespace == "/")
            _namespace = "";

        std::string as_name = (_namespace + "/" + name);
        RCLCPP_INFO(get_logger(), "Advertising navigation ActionServer: %s", as_name.c_str());
        RCLCPP_INFO(get_logger(), "Advertising \"make plan\" Service: %s", makePlanServer->get_service_name());
        RCLCPP_INFO(get_logger(), "Listening to localization topic: %s", localization_sub_->get_topic_name());
    }

    {
        using namespace std::chrono_literals;
        // WAITs
        while (rclcpp::ok() && !mb_action_client->wait_for_action_server(10s))
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with MoveBase server! waiting...");
        while (rclcpp::ok() && !getPlanClient->wait_for_action_server(10s))
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with MoveBase make_plan srv! waiting...");
        while (rclcpp::ok() && !graph_srv_client->wait_for_service(10s))
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with the GRAPH srv! waiting...");
        while (rclcpp::ok() && !nav_assist_functions_client_POI->wait_for_service(10s))
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with the navigation_assistant functions srv! waiting...");
        while (rclcpp::ok() && !nav_assist_functions_client_CNP->wait_for_service(10s))
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with the navigation_assistant functions srv! waiting...");
    }
}

void CNavAssistant::Init()
{
    // This initialization has to be moved outside of the constructor because calling services requires the shared_from_this() pointer to be
    // avaliable, which can only happen after the constructor has exited

    if (init_from_file != "")
    {
        // The topology has been previously saved to file. Reload it.
        RCLCPP_INFO(get_logger(), "[NavAssistant] Loading Graph from file [%s]", init_from_file.c_str());
        // File contains a boost::graphviz description of the nodes and arcs
        auto request = std::make_shared<topology_graph::srv::Graph::Request>();
        request->cmd = "LoadGraph"; // params: [file_path]
        request->params.clear();
        request->params.push_back(init_from_file);

        auto future = graph_srv_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) != rclcpp::FutureReturnCode::SUCCESS)
            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to Load Graph from file. Skipping.");
    }

    if (init_from_param)
    {
        // The topology is provided externally (MoveCare project)
        RCLCPP_INFO(get_logger(), "[NavAssistant] Loading Graph from parameter [%s]", topology_parameter.c_str());
        // Get global parameter with the JSON description of the topology
        bool topo_found = false;
        while (rclcpp::ok() && !topo_found)
        {
            if (has_parameter(topology_parameter))
            {
                get_parameter<std::string>(topology_parameter, topology_str);
                json json_msg = json::parse(topology_str);

                // Process JSON
                get_graph_data_from_json(json_msg);

                // Done
                topo_found = true;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "[topology_graph] Waiting for topological_map global parameter (see TaskCoordinator node).");
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(2s));
            }
        }
    }

    // Advertise service to add new points
    addpointService =
        create_service<NAS::NavAssistantPoint>("/navigation_assistant/add_point_of_interest", std::bind(&CNavAssistant::addPointCB, this, _1, _2));

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    ready_publisher->publish(ready_msg);

    // init internal variables
    counter = 0;
    RCLCPP_INFO(get_logger(), "[NavAssistant] Node ready for action!");
}

// handle the request asynchronously. See https://github.com/ros2/rclcpp/pull/1709#discussion_r1276535852
void CNavAssistant::makePlan_async(const std::shared_ptr<rmw_request_id_t> header, const NAS::MakePlan::Request::SharedPtr request)
{
    rclcpp::Time sendTime = now();
    GetPlan::Goal mb_request;

    mb_request.use_start = true;
    mb_request.start = request->start;
    mb_request.goal = request->goal;

    auto callback = [header, sendTime, this](const rclcpp_action::ClientGoalHandle<GetPlan>::WrappedResult& w_result)
    {
        nav_msgs::msg::Path& m_CurrentPlan = w_result.result->path;
        RCLCPP_INFO(get_logger(), "Path calculated");
        RCLCPP_INFO(get_logger(), "GOT REPLY IN %fs", (now() - sendTime).seconds());
        NAS::MakePlan::Response response;

        if (m_CurrentPlan.poses.empty())
        {
            if (verbose)
                RCLCPP_WARN(get_logger(), "[NavAssistant-turn_towards_path] Unable to get plan");
            response.valid_path = false;
            return;
        }
        path_publisher->publish(m_CurrentPlan);
        response.valid_path = true;
        response.plan = m_CurrentPlan;
        makePlanServer->send_response(*header, response);
    };

    auto goal_options = rclcpp_action::Client<GetPlan>::SendGoalOptions();
    goal_options.result_callback = std::move(callback);
    getPlanClient->async_send_goal(mb_request, goal_options);
}

// cannot be called from within a callback, because it requires spinning
bool CNavAssistant::makePlan_sync(NAS::MakePlan::Request& request, NAS::MakePlan::Response& response)
{
    GetPlan::Goal mb_request;

    mb_request.use_start = true;
    mb_request.start = request.start;
    mb_request.goal = request.goal;

    // send the "make plan" goal to nav2 and wait until the response comes back
    std::optional<nav_msgs::msg::Path> m_CurrentPlan = std::nullopt;

    auto callback = [&m_CurrentPlan, this](const rclcpp_action::ClientGoalHandle<GetPlan>::WrappedResult& w_result)
    {
        m_CurrentPlan = w_result.result->path;
        RCLCPP_INFO(get_logger(), "Path calculated");
    };
    auto goal_options = rclcpp_action::Client<GetPlan>::SendGoalOptions();
    goal_options.result_callback = callback;

    auto future = getPlanClient->async_send_goal(mb_request, goal_options);
    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);

    // Check if valid goal with move_base srv
    if (result != rclcpp::FutureReturnCode::SUCCESS)
    {
        // SRV is not available!! Report Error
        RCLCPP_ERROR(get_logger(), "[NavAssistant-turn_towards_path] Unable to call MAKE_PLAN service from MoveBase");
        response.valid_path = false;
        return false;
    }

    // wait until path is received. The spin-until-future-complete line only blocks until the request is accepted!
    rclcpp::Rate wait_rate(200);
    while (!m_CurrentPlan.has_value())
    {
        wait_rate.sleep();
        rclcpp::spin_some(shared_from_this());
    }

    if (m_CurrentPlan.value().poses.empty())
    {
        if (verbose)
            RCLCPP_WARN(get_logger(), "[NavAssistant-turn_towards_path] Unable to get plan");
        response.valid_path = false;
        return true;
    }
    path_publisher->publish(m_CurrentPlan.value());
    response.valid_path = true;
    response.plan = m_CurrentPlan.value();
    return true;
}

// ----------------------------------
// Turn in place before Navigation  -
// ----------------------------------
void CNavAssistant::turn_towards_path(geometry_msgs::msg::PoseStamped pose_goal)
{
    // Use move_base service "make_plan" to set initial orientation
    NAS::MakePlan::Request getPlanReq;
    NAS::MakePlan::Response response;

    getPlanReq.start = current_robot_pose;
    // Set Goal pose
    getPlanReq.goal = pose_goal;

    bool result = makePlan_sync(getPlanReq, response);

    // Check if valid goal with move_base srv
    if (!result)
    {
        // SRV is not available!! Report Error
        RCLCPP_ERROR(get_logger(), "[NavAssistant-turn_towards_path] Unable to call MAKE_PLAN service from MoveBase");
        return;
    }
    else if (response.plan.poses.empty())
    {
        if (verbose)
            RCLCPP_WARN(get_logger(), "[NavAssistant-turn_towards_path] Unable to get plan");
        return;
    }
    else
    {
        try
        {
            // get initial orientation of path from vector<pose>
            double Ax, Ay, d = 0.0;
            geometry_msgs::msg::Pose p;
            p = response.plan.poses[response.plan.poses.size() - 1].pose; // init at goal
            for (size_t h = 0; h < response.plan.poses.size(); h++)
            {
                if (h == 0)
                {
                    Ax = getPlanReq.start.pose.position.x - response.plan.poses[h].pose.position.x;
                    Ay = getPlanReq.start.pose.position.y - response.plan.poses[h].pose.position.y;
                }
                else
                {
                    Ax = response.plan.poses[h - 1].pose.position.x - response.plan.poses[h].pose.position.x;
                    Ay = response.plan.poses[h - 1].pose.position.y - response.plan.poses[h].pose.position.y;
                }
                d += sqrt(pow(Ax, 2) + pow(Ay, 2));

                // end condition [m]
                if (d >= 0.3)
                {
                    p = response.plan.poses[h].pose;
                    break;
                }
            }

            // The path does not cotains orientation, only positions, so we need to calculate the orientation between robot and target in the path
            Ax = p.position.x - current_robot_pose.pose.position.x;
            Ay = p.position.y - current_robot_pose.pose.position.y;
            double target_yaw_map = atan2(Ay, Ax);

            // Set goal in MoveBase
            geometry_msgs::msg::PoseStamped local_goal;
            local_goal.header = current_robot_pose.header;
            local_goal.pose.position = current_robot_pose.pose.position;
            tf2::Quaternion q;
            q.setRPY(0, 0, angles::normalize_angle(target_yaw_map));
            local_goal.pose.orientation = tf2::toMsg(q);
            if (verbose)
                RCLCPP_INFO(get_logger(), "Requesting Initial Turn");

            // Call MoveBase (turn in place)
            move_base_nav_and_wait(local_goal);

            return;
        }
        catch (exception e)
        {
            RCLCPP_ERROR(get_logger(), "[taskCoordinator] Error in turn_towards_path: [%s]", e.what());
            return;
        }
    }
}

// -------------------------------------------------
// Cancel current navigation and wait confirmation -
// -------------------------------------------------
bool CNavAssistant::move_base_cancel_and_wait(double wait_time_sec)
{
    try
    {
        rclcpp::Time start_time = now();

        // Request goal cancelation (asincronous)
        auto future = mb_action_client->async_cancel_all_goals();

        auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);

        if (result == rclcpp::FutureReturnCode::SUCCESS)
            return true;
        else
        {
            RCLCPP_ERROR(get_logger(), "Goal could not be canceled");
            return false;
        }
    }
    catch (exception e)
    {
        RCLCPP_ERROR(get_logger(), "[taskCoordinator-CancelActionAndWait] Exception: [%s]", e.what());
        return false;
    }
}

// ----------------------------------------
// Trigger Navigation and wait completion -
// ----------------------------------------
void CNavAssistant::move_base_nav_and_wait(geometry_msgs::msg::PoseStamped pose_goal)
{
    auto currentServerGoal = m_currentGoal.ServerGoalHandle;
    // request MoveBase to reach a goal
    NavToPose::Goal mb_goal;
    mb_goal.pose = pose_goal;

    if (verbose)
        RCLCPP_INFO(get_logger(), "Requesting MoveBase Navigation to [%.2f, %.2f]", mb_goal.pose.pose.position.x, mb_goal.pose.pose.position.y);

    auto goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    goal_options.result_callback = std::bind(&NavigationGoal::result_cb, &m_currentGoal, _1);
    goal_options.goal_response_callback = std::bind(&NavigationGoal::response_cb, &m_currentGoal, _1);

    m_currentGoal.complete = false;

    auto future = mb_action_client->async_send_goal(mb_goal, goal_options);
    rclcpp::spin_until_future_complete(shared_from_this(), future);

    rclcpp::Rate rate(200);
    while (rclcpp::ok() && !m_currentGoal.complete && !m_currentGoal.goalCancelled)
    {
        // if there was explicit cancellation or just a new goal overwriting the current one
        if (m_currentGoal.goalCancelled || m_currentGoal.ServerGoalHandle != currentServerGoal)
        {
            m_currentGoal.goalCancelled = true;
            mb_action_client->async_cancel_goal(m_currentGoal.ClientGoalHandle);
        }
        rclcpp::spin_some(shared_from_this());
        rate.sleep();
    }
    m_currentGoal.ClientGoalHandle = {nullptr};
    m_currentGoal.complete = false;
}

//=================================================================
// Action Server Callback -> New Goal for the NavAssistant (START!)
//=================================================================
rclcpp_action::GoalResponse CNavAssistant::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NAS_ac::NavAssistant::Goal> goal)
{
    if (verbose)
        RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CNavAssistant::handle_cancel(const std::shared_ptr<GoalHandleNavigate_Server> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    cancelCurrentGoal();
    mb_action_client->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CNavAssistant::handle_accepted(const std::shared_ptr<GoalHandleNavigate_Server> goal_handle)
{
    if (verbose)
        RCLCPP_INFO(this->get_logger(), "Accepted goal");
    if (m_currentGoal.ClientGoalHandle.get() != nullptr)
        cancelCurrentGoal();
    m_currentGoal.ServerGoalHandle = goal_handle;
}

void CNavAssistant::execute()
{
    // This ActionServer is a wrapper for move_base aiming at a more robust navvigation and error handling
    try
    {
        auto goal = m_currentGoal.ServerGoalHandle->get_goal();
        if (verbose)
            RCLCPP_INFO(get_logger(), "Starting Navigation Assistant to reach: [%.2f, %.2f]", goal->pose.pose.position.x, goal->pose.pose.position.y);

        // Get path (robot->goal) from topology-graph node
        std::string node_start, node_end;
        std::vector<std::string> path;

        // 1. Get starting ING node
        auto graphRequest = std::make_shared<topology_graph::srv::Graph::Request>();
        graphRequest->cmd = "GetClosestNode"; // params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
        graphRequest->params.push_back(std::to_string(current_robot_pose.pose.position.x));
        graphRequest->params.push_back(std::to_string(current_robot_pose.pose.position.y));
        graphRequest->params.push_back(std::to_string(0.0));
        graphRequest->params.push_back("ING");

        auto future = graph_srv_client->async_send_request(graphRequest);
        auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();

        if (response->success)
        {
            node_start = response->result[0]; // result: [id label type x y yaw]
            if (verbose)
                RCLCPP_INFO(get_logger(), "[NavAssistant]: Starting Node is %s - %s", node_start.c_str(), response->result[1].c_str());

            // 2. Get ending ING node
            graphRequest->cmd = "GetClosestNode"; // params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
            graphRequest->params.clear();
            response->result.clear();
            graphRequest->params.push_back(std::to_string(goal->pose.pose.position.x));
            graphRequest->params.push_back(std::to_string(goal->pose.pose.position.y));
            graphRequest->params.push_back(std::to_string(0.0));
            graphRequest->params.push_back("ING");
            future = graph_srv_client->async_send_request(graphRequest);
            result = rclcpp::spin_until_future_complete(shared_from_this(), future);
            response = future.get();
            if (response->success)
            {
                node_end = response->result[0]; // result: [id label type x y yaw]
                if (verbose)
                    RCLCPP_INFO(get_logger(), "[NavAssistant]: End Node is %s -%s", node_end.c_str(), response->result[1].c_str());

                // Navigation to Intermediate Navigation Goals (ING) if any
                if (node_start != node_end)
                {
                    // Find Path
                    // params: [nodeID_start, nodeID_end]
                    // result: Success/Failure. On success the list of Nodes ["id1 label1 type1 x1 y1 yaw", ..., ""idN labelN typeN xN yN yawN"]
                    graphRequest->cmd = "FindPath";
                    graphRequest->params.clear();
                    response->result.clear();
                    graphRequest->params.push_back(node_start);
                    graphRequest->params.push_back(node_end);
                    future = graph_srv_client->async_send_request(graphRequest);
                    result = rclcpp::spin_until_future_complete(shared_from_this(), future);
                    response = future.get();
                    if (response->success)
                    {
                        path = response->result;
                        if (verbose)
                        {
                            for (auto n : path)
                                RCLCPP_INFO(get_logger(), "[NavAssistant]: %s", n.c_str());
                        }

                        try
                        {
                            //===============================================================
                            // The path is composed of ING nodes.
                            // ING nodes corresponding to a CNP/CP have the same label
                            // We only consider here those ING that cross a Critial Point
                            // Therefore, remove any orphan ING (single label)
                            // Orphan nodes are prone to happen at the start and end points
                            //===============================================================
                            // START node
                            std::vector<string> node0, node1;
                            boost::split(node0, path[0], boost::is_any_of(" "));
                            boost::split(node1, path[1], boost::is_any_of(" "));
                            if (node0[1] != node1[1]) // Orphan node. Remove it
                                path.erase(path.begin());
                            // END node
                            boost::split(node0, path[path.size() - 1], boost::is_any_of(" "));
                            boost::split(node1, path[path.size() - 2], boost::is_any_of(" "));
                            if (node0[1] != node1[1]) // Orphan node. Remove it
                                path.erase(path.end());

                            // Step by Step NAVIGATION
                            for (size_t i = 0; i < path.size(); i++)
                            {
                                // Node data = "id label type x y yaw"
                                std::vector<string> current_node_data, next_node_data;
                                boost::split(current_node_data, path[i], boost::is_any_of(" "));

                                // Set i_goal
                                geometry_msgs::msg::PoseStamped i_goal;
                                i_goal.header.frame_id = "map";
                                i_goal.header.stamp = rclcpp::Time(0);
                                i_goal.pose.position.x = atof(current_node_data[3].c_str());
                                i_goal.pose.position.y = atof(current_node_data[4].c_str());
                                i_goal.pose.position.z = 0.0;
                                // Set correct orientation (towards next node)
                                double target_yaw_map;
                                if (i < (path.size() - 1))
                                {
                                    // Head towards next node
                                    boost::split(next_node_data, path[i + 1], boost::is_any_of(" "));
                                    double Ax = atof(next_node_data[3].c_str()) - atof(current_node_data[3].c_str());
                                    double Ay = atof(next_node_data[4].c_str()) - atof(current_node_data[4].c_str());
                                    target_yaw_map = atan2(Ay, Ax);
                                }
                                else
                                {
                                    // Head towards goal!
                                    double Ax = goal->pose.pose.position.x - atof(current_node_data[3].c_str());
                                    double Ay = goal->pose.pose.position.y - atof(current_node_data[4].c_str());
                                    target_yaw_map = atan2(Ay, Ax);
                                }
                                // Set orientation
                                tf2::Quaternion q;
                                q.setRPY(0, 0, angles::normalize_angle(target_yaw_map));
                                i_goal.pose.orientation = tf2::toMsg(q);

                                // A. Initial Turn
                                if (m_currentGoal.checkIfCancelled(shared_from_this()))
                                    return;
                                if (goal->turn_before_nav)
                                    turn_towards_path(i_goal);

                                // B. Navigate to i_goal using MOVE_BASE!
                                if (m_currentGoal.checkIfCancelled(shared_from_this()))
                                    return;
                                move_base_nav_and_wait(i_goal);
                            }
                        }
                        catch (exception e)
                        {
                            RCLCPP_ERROR(get_logger(), "[NavAssistant] Exception while navigating through path: %s\n", e.what());
                            close_and_return();
                            return;
                        }
                        catch (...)
                        {
                            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unknown Exception while navigating through path");
                            close_and_return();
                            return;
                        }
                    }
                    else
                    {
                        if (verbose)
                            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to get Path. Trying direct navigation.");
                    }
                } // end if-start!=end
            }
            else if (verbose)
                RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to get Closest ING Node (end) pose=[%.3f, %.3f]. Trying direct navigation.",
                            goal->pose.pose.position.x, goal->pose.pose.position.y);
        }
        else if (verbose)
            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to get Closest ING Node (start) pose=[%.3f, %.3f]. Trying direct navigation.",
                        current_robot_pose.pose.position.x, current_robot_pose.pose.position.y);

        // -------------------------------
        // NAVIGATION TO TARGET GOAL (MB)
        // -------------------------------

        if (m_currentGoal.checkIfCancelled(shared_from_this()))
            return;

        // 1. Initial Turn
        if (goal->turn_before_nav)
            turn_towards_path(goal->pose);

        rclcpp::spin_some(shared_from_this());

        if (m_currentGoal.checkIfCancelled(shared_from_this()))
            return;
        // 2. Navigate to goal
        move_base_nav_and_wait(goal->pose);

        // reset the cancelled flag if the cancellation happened after the second goal was sent
        if (!m_currentGoal.checkIfCancelled(shared_from_this()))
        {
            auto result = std::make_shared<NAS_ac::NavAssistant::Result>();
            m_currentGoal.ServerGoalHandle->succeed(result);
        }
    }

    catch (exception e)
    {
        RCLCPP_ERROR(get_logger(), "[NavAssistant] Exception while executting AS: %s\n", e.what());
        close_and_return();
        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "[NavAssistant] Unknown Exception in AS");
        close_and_return();
        return;
    }
}

// ----------------
// Stop the robot -
// ----------------
void CNavAssistant::close_and_return()
{
    // Cancell MoveBase Actions (in any) and Wait!
    move_base_cancel_and_wait(5.0);

    // Stop the robot (manually)
    geometry_msgs::msg::Twist stop;
    stop.linear.x = stop.linear.y = stop.linear.z = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_publisher->publish(stop);

    if (verbose)
        RCLCPP_INFO(get_logger(), "[NavAssistant] Closing Action...");
}

CNavAssistant::~CNavAssistant()
{
}

//===================================================================================
//================================== MAIN ===========================================
//===================================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<CNavAssistant> nav_assistant = std::make_shared<CNavAssistant>("nav_assistant");
    nav_assistant->Init();
    RCLCPP_INFO(nav_assistant->get_logger(), "[NavAssistant] Action server is ready for action!...");

    rclcpp::Rate rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(nav_assistant);
        nav_assistant->HandleGraphRequests();

        if (nav_assistant->m_currentGoal.ServerGoalHandle.get() != nullptr)
        {
            auto currentServerGoalHandle = nav_assistant->m_currentGoal.ServerGoalHandle;
            nav_assistant->execute();

            // dont reset the variable if a new goal was received before the last one was completed
            if (currentServerGoalHandle == nav_assistant->m_currentGoal.ServerGoalHandle)
                nav_assistant->m_currentGoal.ServerGoalHandle = {nullptr};
        }
        rate.sleep();
    }

    return 0;
}
