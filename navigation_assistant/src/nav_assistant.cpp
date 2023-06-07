#include "nav_assistant.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/algorithm/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using json = nlohmann::json;
using namespace std::placeholders;

//-----------------------------------------------------------
//                    Subscriptions
//----------------------------------------------------------

// Robot location update
void CNavAssistant::localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    //keep the most recent robot pose = position + orientation
    geometry_msgs::msg::PoseStamped msgPose;
    msgPose.header = msg->header;
    msgPose.pose = msg->pose.pose;
    current_robot_pose = transformPoseToFrame(msgPose, "map");
}

double getYaw(const geometry_msgs::msg::Pose& pose)
{
    tf2::Quaternion quat;
    tf2::fromMsg(pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::msg::PoseStamped CNavAssistant::transformPoseToFrame(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame)
{
    static tf2_ros::Buffer buffer(get_clock());
    static tf2_ros::TransformListener listener(buffer);
    using namespace std::chrono_literals;
    geometry_msgs::msg::PoseStamped result;
    try
    {
        result = buffer.transform(pose, target_frame, std::chrono::duration_cast<std::chrono::nanoseconds>(0.5s) );
    }
    catch(const std::exception& e)
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
    
    as_ = rclcpp_action::create_server<NAS_ac::NavAssistant>(this, name, 
        std::bind(&CNavAssistant::handle_goal, this, _1, _2), 
        std::bind(&CNavAssistant::handle_cancel, this, _1),
        std::bind(&CNavAssistant::handle_accepted, this, _1)
    ); 

    // Read config parameters
    verbose = declare_parameter<bool>("verbose",  false);
    init_from_param = declare_parameter<bool>("init_from_param",  false);
    topology_parameter = declare_parameter<std::string>("topological_json_parameter","/topological_map");
    load_passages_as_CP = declare_parameter<bool>("load_passages_as_CP",  false);
    force_CP_as_additional_ANP = declare_parameter<bool>("force_CP_as_additional_ANP",  false);
    init_from_file = declare_parameter<std::string>("init_from_file","");
    save_to_file = declare_parameter<std::string>("save_to_file","");


    makePlanServer = create_service<NAS::MakePlan>("navigation_assistant/make_plan", std::bind(&CNavAssistant::makePlan, this, _1, _2));


    // Service clients
    mb_action_client = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    getPlanClient = rclcpp_action::create_client<GetPlan>(this, "compute_path_to_pose");  // only accounts for global_costmap but works at all times
    graph_srv_client = create_client<topology_graph::srv::Graph>("topology_graph/graph");    // Graph service client
    nav_assist_functions_client_POI = create_client<NAS::NavAssistantPOI>("navigation_assistant/get_poi_related_poses");
    nav_assist_functions_client_CNP = create_client<NAS::NavAssistantSetCNP>("navigation_assistant/get_cnp_pose_around");

    // Subscribers
    localization_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&CNavAssistant::localizationCallback,this, _1) );

    // Publishers
    cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    ready_publisher = create_publisher<std_msgs::msg::Bool>("/nav_assistant/ready",1);
    path_publisher = create_publisher<nav_msgs::msg::Path>("/nav_assistant/Path",1);

    {
        using namespace std::chrono_literals;
        // WAITs
        while (rclcpp::ok() && !mb_action_client->wait_for_action_server(10s))
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with MoveBase server! waiting...");
        while (rclcpp::ok() && !getPlanClient->wait_for_action_server(10s) )
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with MoveBase make_plan srv! waiting...");
        while (rclcpp::ok() && !graph_srv_client->wait_for_service(10s) )
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with the GRAPH srv! waiting...");
        while (rclcpp::ok() && !nav_assist_functions_client_POI->wait_for_service(10s) )
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with the navigation_assistant functions srv! waiting...");
        while (rclcpp::ok() && !nav_assist_functions_client_CNP->wait_for_service(10s) )
            RCLCPP_ERROR(get_logger(), "[NavAssistant] Unable to contact with the navigation_assistant functions srv! waiting...");

    }
}

void CNavAssistant::Init()
{
    //This initialization has to be moved outside of the constructor because calling services requires the shared_from_this() pointer to be avaliable, 
    //which can only happen after the constructor has exited
    
    if (init_from_file != "")
    {
        // The topology has been previously saved to file. Reload it.
        RCLCPP_INFO(get_logger(), "[NavAssistant] Loading Graph from file [%s]", init_from_file.c_str());
        //File contains a boost::graphviz description of the nodes and arcs
        auto request = std::make_shared<topology_graph::srv::Graph::Request>();
        request->cmd = "LoadGraph";    // params: [file_path]
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
        //Get global parameter with the JSON description of the topology
        bool topo_found = false;
        while (rclcpp::ok() && !topo_found)
        {
            if (has_parameter(topology_parameter))
            {
                get_parameter<std::string>(topology_parameter, topology_str);
                json json_msg = json::parse( topology_str );

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
    service = create_service<NAS::NavAssistantPoint>("/navigation_assistant/add_point_of_interest", std::bind(&CNavAssistant::srvCB, this, _1, _2) );

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    ready_publisher->publish(ready_msg);

    // init internal variables
    counter = 0;
    RCLCPP_INFO(get_logger(), "[NavAssistant] Node ready for action!");
}

void CNavAssistant::getPlanCB(const rclcpp_action::ClientGoalHandle<GetPlan>::WrappedResult& w_result)
{
    m_CurrentPlan = w_result.result->path;
    RCLCPP_INFO(get_logger(), "Path calculated");
}

bool CNavAssistant::makePlan(NAS::MakePlan::Request::SharedPtr request, NAS::MakePlan::Response::SharedPtr response)
{
    GetPlan::Goal mb_request;

    mb_request.use_start = true;
    mb_request.start = request->start;
    mb_request.goal = request->goal;

    auto goal_options = rclcpp_action::Client<GetPlan>::SendGoalOptions();
    goal_options.result_callback = std::bind(&CNavAssistant::getPlanCB, this, _1);
    
    m_CurrentPlan.header.stamp.sec=-1;

    auto future = getPlanClient->async_send_goal(mb_request, goal_options); 
    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);

    //Check if valid goal with move_base srv
    if( result != rclcpp::FutureReturnCode::SUCCESS )
    {
        // SRV is not available!! Report Error
        RCLCPP_ERROR(get_logger(), "[NavAssistant-turn_towards_path] Unable to call MAKE_PLAN service from MoveBase");
        response->valid_path = false;
        return false;
    }
    
    rclcpp::Rate wait_rate(2);
    while(m_CurrentPlan.header.stamp.sec == -1)
    {
        //wait until path is received. The spinning line only blocks until the request is accepted!
        wait_rate.sleep();
        rclcpp::spin_some(shared_from_this());
    }
    
    if (m_CurrentPlan.poses.empty() )
    {
        if (verbose) RCLCPP_WARN(get_logger(), "[NavAssistant-turn_towards_path] Unable to get plan");
        response->valid_path = false;
        return true;
    }
    path_publisher->publish(m_CurrentPlan);
    response->valid_path = true;
    response->plan = m_CurrentPlan;
    return true;
}


// ----------------------------------
// Load Navigation Graph from JSON   -
// ----------------------------------
void CNavAssistant::get_graph_data_from_json(json json_msg)
{
    /*
     * The JSON floorplan looks like this:
    {
        "spaces":[
            {"category":"outside","label":"OUTSIDE1","location":{"y":-1.45,"x":-3.775}},
            {"category":"kitchen","label":"KITCHEN1","location":{"y":-2.4771541875,"x":2.172979}}
        ],

        "poses":[
            {"space_label":"OUTSIDE1","location":{"y":3.569545,"x":-0.689465875}},
            {"space_label":"KITCHEN1","location":{"y":7.5355625,"x":-0.0051883124999961}}
            {"space_label":"KITCHEN1","location":{"y":3.569545,"x":-0.689465875}},
        ],

        "passages":[
            {"location":{"y":-0.976899125,"x":1.232270875},"label":"OPEN1","connecting":["OUTSIDE1","KITCHEN1"]},
            {"location":{"y":-1.4562701875,"x":2.0522945625},"label":"OPEN2","connecting":["HALLWAY2","KITCHEN1"]}
        ],

        "userid":"2c938084683d9f87016a92216a6f005e"
    }
    */



    //1. Do we have "poses" (old/new version)
    bool have_poses = false;
    try
    {
        if (json_msg.find("poses") != json_msg.end())
            have_poses = true;
    }
    catch (exception e)
    {
        RCLCPP_ERROR(get_logger(), "[topology_graph] Error getting Poses. Error is: [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        return;
    }


    //2. Load Spaces (aka Rooms)
    try
    {
        if (!have_poses)
        {
            // Single vertex per space
            for (json::iterator it = json_msg["spaces"].begin(); it != json_msg["spaces"].end(); ++it)
            {
                auto content = it.value();
                // 1. Add a new node in the graph
                if (content["label"] != "OUTSIDE1")
                    addNode(content["label"], "space", content["location"]["x"], content["location"]["y"], 0.0);
            }
        }
        else
        {
            // Multiple poses per room
            for (json::iterator it = json_msg["poses"].begin(); it != json_msg["poses"].end(); ++it)
            {
                auto content = it.value();
                // 1. Add a new node in the graph
                if (content["space_label"] != "OUTSIDE1")
                    addNode(content["space_label"], "space", content["location"]["x"], content["location"]["y"], 0.0);
            }
        }
    }
    catch (exception e)
    {
        RCLCPP_ERROR(get_logger(), "[NavAssistant] Error loading graph data. Spaces list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        return;
    }


    //3. Load passages (aka doors)
    try
    {
        for (json::iterator it = json_msg["passages"].begin(); it != json_msg["passages"].end(); ++it)
        {
            auto content = it.value();
            //0. Chech that it does not connect with OUTSIDE1
            auto spaces_connected = content["connecting"];
            if(spaces_connected.at(0) != "OUTSIDE1" && spaces_connected.at(1) != "OUTSIDE1")
            {
                // 1. Add a new node in the graph
                if (load_passages_as_CP)
                    addNode(content["label"], "CP", content["location"]["x"], content["location"]["y"], 0.0);
                else
                    addNode(content["label"], "passage", content["location"]["x"], content["location"]["y"], 0.0);
            }
        }
    }
    catch (exception e)
    {
        RCLCPP_ERROR(get_logger(), "[NavAssistant] Error loading graph data. Passages list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        return;
    }


    // 4. Check Objects
    try
    {
        char buffer [50];
        for (json::iterator it = json_msg["objects"].begin(); it != json_msg["objects"].end(); ++it)
        {
            auto content = it.value();
            if (content["label"] == "docking_station" || content["type"]=="dockingstation")
            {
                // 1. Add a new node in the graph
                addNode(content["label"], "docking", content["location"]["x"], content["location"]["y"], 0.0);
            }
        }
    }
    catch (exception e)
    {
        RCLCPP_ERROR(get_logger(), "[taskCoordinator] Error loading Env data. Passages list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        return;
    }

    // Finally, regenerate Arcs                
    using namespace std::chrono_literals;
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(1s));

    regenerate_arcs();
}



// Function that creates a new node in the GRAPH
/*
 * THERE ARE 4 MAIN TYPES OF NODES
 * space   -> To define rooms, navigation goals, etc by setting a label
 * passage -> To define doors and separation between spaces (interesting for topological localization)
 * CNP     -> Critical Navigation Point. Defines an area difficult for robot navigation and set two Intermediate Navigation Goals (ING) (i.e narrow doors)
 * CP      -> Critial Passage. Is a combination of passage (topo-localization) and CNP, also generating the 2 ING
 * docking -> For Docking Approach
 */
bool CNavAssistant::addNode(std::string node_label, std::string node_type, double pose_x, double pose_y, double pose_yaw)
{
    // node as a pose
    geometry_msgs::msg::PoseStamped node_pose;
    node_pose.header.frame_id = "map";
    node_pose.header.stamp = rclcpp::Time(0);
    node_pose.pose.position.x = pose_x;
    node_pose.pose.position.y = pose_y;
    node_pose.pose.position.z = 0.0;
    node_pose.pose.orientation = tf2::toMsg( tf2::Quaternion(tf2::Vector3(0,0,1), pose_yaw) );

    if (node_type == "CNP" || node_type == "CP" || node_type == "passage")
    {
        if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant] Estimating optimal pose for CNP around [%.3f, %.3f]", pose_x, pose_y);
        // Estimate best location for the CP/CNP/passage around given pose
        auto request = std::make_shared<NAS::NavAssistantSetCNP::Request>();
        request->pose = node_pose;
        auto future = nav_assist_functions_client_CNP->async_send_request(request);
        rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();
        //Update pose
        if (response->success)
        {
            node_pose = response->pose;
            if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant] Setting CNP at [%.3f, %.3f]", node_pose.pose.position.x, node_pose.pose.position.y);
        }
        else
            if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant] Error setting CNP around [%.3f, %.3f], Using exact point.", pose_x, pose_y);
    }


    // 1. Add a new node in the graph
    auto graphRequest = std::make_shared<topology_graph::srv::Graph::Request>();
    graphRequest->cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
    graphRequest->params.push_back(node_label);
    graphRequest->params.push_back(node_type);
    graphRequest->params.push_back(std::to_string(node_pose.pose.position.x));
    graphRequest->params.push_back(std::to_string(node_pose.pose.position.y));
    graphRequest->params.push_back(std::to_string(pose_yaw));
        
    {
        auto future = graph_srv_client->async_send_request(graphRequest);
        rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();
        
        if (!response->success)
        {
            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create new Node");
            return false;
        }    
    }

    // According to the type of node, additional nodes may be necessary
    if (node_type == "passage")
    {
        // Estimate and Add the two Segment Points (SP) that define this passage
        auto request = std::make_shared<NAS::NavAssistantPOI::Request>();
        request->pose = node_pose;

        auto future = nav_assist_functions_client_POI->async_send_request(request);
        rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();

        if (response->success)
        {
            for (auto p: response->sp)
            {
                graphRequest->cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
                graphRequest->params.clear();
                graphRequest->params.push_back(node_label);   //use the same label as the original POI
                graphRequest->params.push_back("SP");
                graphRequest->params.push_back(std::to_string(p.pose.position.x));
                graphRequest->params.push_back(std::to_string(p.pose.position.y));

                graphRequest->params.push_back(std::to_string(getYaw(p.pose)));
                
                auto future = graph_srv_client->async_send_request(graphRequest);
                rclcpp::spin_until_future_complete(shared_from_this(), future);
                auto response = future.get();

                if (!response->success)
                {
                    RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create new Node");
                    return false;
                }
            }
        }
    }

    // Critical Navigation Point (CNP) , Critial Passage (CP)
    else if ( (node_type == "CNP") || (node_type == "CP") )
    {
        // Estimate and Add the two Segment Points (SP) and the two Intermediate Navigation Goals (ING)
        auto poiRequest = std::make_shared<NAS::NavAssistantPOI::Request>();
        poiRequest->pose = node_pose;

        auto future = nav_assist_functions_client_POI->async_send_request(poiRequest);
        rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();

        if (response->success)
        {
            //2.1 Add "SP" nodes (Segment Points)
            for (auto p: response->sp)
            {
                graphRequest->cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
                graphRequest->params.clear();
                graphRequest->params.push_back(node_label);   //use the same label as the original POI
                graphRequest->params.push_back("SP");
                graphRequest->params.push_back(std::to_string(p.pose.position.x));
                graphRequest->params.push_back(std::to_string(p.pose.position.y));

                graphRequest->params.push_back(std::to_string(getYaw(p.pose)));
                
                auto future = graph_srv_client->async_send_request(graphRequest);
                rclcpp::spin_until_future_complete(shared_from_this(), future);
                auto response = future.get();

                if (!response->success)
                {
                    RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create new Node");
                    return false;
                }
            }

            //2.2 Add a "ING" nodes (intermediate navigation goals)
            std::vector<std::string> ing_id;
            for (auto p: response->ing)
            {
                graphRequest->cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
                graphRequest->params.clear();
                graphRequest->params.push_back(node_label);   //use the same label as the original POI
                graphRequest->params.push_back("SP");
                graphRequest->params.push_back(std::to_string(p.pose.position.x));
                graphRequest->params.push_back(std::to_string(p.pose.position.y));
                

                graphRequest->params.push_back(std::to_string(getYaw(p.pose)));
                
                auto future = graph_srv_client->async_send_request(graphRequest);
                rclcpp::spin_until_future_complete(shared_from_this(), future);
                auto response = future.get();

                if (!response->success)
                {
                    RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create new Node");
                    return false;
                }

                //keep node_id
                std::string current_node_id = response->result[0];
                ing_id.push_back(current_node_id);
            }            
        }
    }

    // Done
    return true;
}


void CNavAssistant::regenerate_arcs()
{
    // Adding or Removing nodes affect the overall Arc distribution

    //1. Remove all Arcs in the Graph
    {
        if (verbose) 
            RCLCPP_INFO(get_logger(), "[NavAssistant] Regenerating Arcs in Graph");
        auto clearRequest = std::make_shared<topology_graph::srv::Graph::Request>();
        clearRequest->cmd = "DeleteAllArcs";    // params: [node_id]
        clearRequest->params.push_back("-1");     // All nodes = -1
        auto future = graph_srv_client->async_send_request(clearRequest);
        rclcpp::spin_until_future_complete(shared_from_this(), future);
    }

    /* Regenerate Arcs
     * Only nodes of type: ING, and optionally CP and CNP are connected in the graph
     * connections avoid Passages and Segment Points
     */

    std::vector<string> nodes_to_connect;
    nodes_to_connect.clear();
    {
        auto regenerateRequest=std::make_shared<topology_graph::srv::Graph::Request>();
        // ING nodes
        regenerateRequest->cmd = "GetNodesbyType";    // params: node_type
        regenerateRequest->params.clear();
        regenerateRequest->params.push_back("ING");
    
        auto future = graph_srv_client->async_send_request(regenerateRequest);
        auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();
    
        if (!response->success)
        {
            RCLCPP_WARN(get_logger(), "[NavAssistant-regenerateArcs]: Unable to get ING Nodes from Graph");
            return;
        }
        else
            nodes_to_connect.insert(nodes_to_connect.end(), response->result.begin(), response->result.end());
    }

    if (force_CP_as_additional_ANP)
    {
        {
            auto regenerateRequest=std::make_shared<topology_graph::srv::Graph::Request>();
            // CP nodes
            regenerateRequest->cmd = "GetNodesbyType";    // params: node_type
            regenerateRequest->params.clear();
            regenerateRequest->params.push_back("CP");
            
            auto future = graph_srv_client->async_send_request(regenerateRequest);
            auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
            auto response = future.get();
            if (!response->success)
            {
                RCLCPP_WARN(get_logger(), "[NavAssistant-regenerateArcs]: Unable to get CP Nodes from Graph");
                return;
            }
            else
                nodes_to_connect.insert(nodes_to_connect.end(), response->result.begin(), response->result.end());
        }

        {
            auto regenerateRequest=std::make_shared<topology_graph::srv::Graph::Request>();
            // CNP nodes
            regenerateRequest->cmd = "GetNodesbyType";    // params: node_type
            regenerateRequest->params.clear();
            regenerateRequest->params.push_back("CNP");
            
            auto future = graph_srv_client->async_send_request(regenerateRequest);
            auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
            auto response = future.get();
            if (!response->success)
            {
                RCLCPP_WARN(get_logger(), "[NavAssistant-regenerateArcs]: Unable to get ING Nodes from Graph");
                return;
            }
            else
                nodes_to_connect.insert(nodes_to_connect.end(), response->result.begin(), response->result.end());
        }
    }


    // for each node in the list
    for (auto it = nodes_to_connect.begin(); it != nodes_to_connect.end(); ++it)
    {
        // Node data = "id label type x y yaw"
        std::vector<std::string> node_data;
        node_data.clear();
        boost::split(node_data, *it, boost::is_any_of(" "));

        // for each other node in the list
        for (auto it2 = it+1; it2 != nodes_to_connect.end(); ++it2)
        {
            // Node data = "id label type x y yaw"
            std::vector<std::string> node_data2;
            node_data2.clear();
            boost::split(node_data2, *it2, boost::is_any_of(" "));

            // Same label (points of the same CNP)
            if (node_data[1] == node_data2[1])
            {
                if (!force_CP_as_additional_ANP)
                {
                    //Create Arc beetwing the two INGs
                    auto addArcRequest=std::make_shared<topology_graph::srv::Graph::Request>();
                    addArcRequest->cmd = "AddArc";    // params: [idfrom, idto, label, type, bidirectional]
                    addArcRequest->params.clear();
                    addArcRequest->params.push_back(node_data[0]);
                    addArcRequest->params.push_back(node_data2[0]);
                    addArcRequest->params.push_back("arc");
                    addArcRequest->params.push_back("arc");
                    addArcRequest->params.push_back("true");       // bidirectional arc
                    
                    auto future = graph_srv_client->async_send_request(addArcRequest);
                    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
                    auto response = future.get();

                    if (!response->success)
                        RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create Arc");
                }
                // Create arc if different types
                else if (node_data[2] != node_data2[2])
                {
                    //Create Arc
                    auto addArcRequest=std::make_shared<topology_graph::srv::Graph::Request>();
                    addArcRequest->cmd = "AddArc";    // params: [idfrom, idto, label, type, bidirectional]
                    addArcRequest->params.clear();
                    addArcRequest->params.push_back(node_data[0]);
                    addArcRequest->params.push_back(node_data2[0]);
                    addArcRequest->params.push_back("arc");
                    addArcRequest->params.push_back("arc");
                    addArcRequest->params.push_back("true");       // bidirectional arc
                    
                    auto future = graph_srv_client->async_send_request(addArcRequest);
                    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
                    auto response = future.get();

                    if (!response->success)
                        RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create Arc");
                }
            }

            // Different Labels
            else
            {
                if (node_data[2]=="ING" && node_data2[2]=="ING")
                {
                    // Create arc if different label and type ING
                    // Check if there is a path not crossing a segment defined by a CNP or CP
                    auto distanceRequest=std::make_shared<topology_graph::srv::Graph::Request>();
                    distanceRequest->cmd = "GetNavDistTwoPoses";    // params: p1(x,y,yaw), p2(x,y,yaw), [avoid_node_types]
                    distanceRequest->params.clear();
                    // p1
                    distanceRequest->params.push_back(node_data[3].c_str());
                    distanceRequest->params.push_back(node_data[4].c_str());
                    distanceRequest->params.push_back(node_data[5].c_str());
                    // p2
                    distanceRequest->params.push_back(node_data2[3].c_str());
                    distanceRequest->params.push_back(node_data2[4].c_str());
                    distanceRequest->params.push_back(node_data2[5].c_str());
                    // avoid intersecting nodes of type
                    distanceRequest->params.push_back("CNP");
                    distanceRequest->params.push_back("CP");

                    auto future = graph_srv_client->async_send_request(distanceRequest);
                    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
                    auto response = future.get();

                    if (verbose) 
                        RCLCPP_WARN(get_logger(), "[NavAssistant]: NavDistance between Node[%s] <-> [%s] is %s (succes=%d)",node_data[0].c_str(), node_data2[0].c_str(), response->result[0].c_str(), response->success);

                    if (response->success)
                    {
                        //system("read -p 'Press Enter to continue...' var");
                        if (verbose) RCLCPP_WARN(get_logger(), "[NavAssistant]: Adding new Arc between ING Nodes [%s] <-> [%s]", node_data[0].c_str(), node_data2[0].c_str());
                        //Create Arc
                        auto addArcRequest=std::make_shared<topology_graph::srv::Graph::Request>();
                        addArcRequest->cmd = "AddArc";    // params: [idfrom, idto, label, type, bidirectional]
                        addArcRequest->params.clear();
                        addArcRequest->params.push_back(node_data[0]);
                        addArcRequest->params.push_back(node_data2[0]);
                        addArcRequest->params.push_back("arc");
                        addArcRequest->params.push_back("arc");
                        addArcRequest->params.push_back("true");       // bidirectional arc
                        auto future = graph_srv_client->async_send_request(distanceRequest);
                        auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
                        auto response = future.get();

                        if (!response->success)
                            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create Arc");
                    }
                }
            }
        }//end.for each other ING
    }//end-for each ING
}




bool CNavAssistant::deleteNode(std::string node_type, double pose_x, double pose_y, double pose_yaw)
{
    if (verbose) 
        RCLCPP_INFO(get_logger(), "[NavAssistant] Deleting Node of type [%s], close to [%.3f, %.3f]", node_type.c_str(), pose_x, pose_y );
    //1. Get all Nodes of type: node_type
    auto getNodesRequest=std::make_shared<topology_graph::srv::Graph::Request>();
    getNodesRequest->cmd = "GetNodesbyType";    // params: [node_type]
    getNodesRequest->params.push_back(node_type);
    
    auto future = graph_srv_client->async_send_request(getNodesRequest);
    auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
    auto response = future.get();

    if (response->success)
    {
        double min_dist;
        std::string node_label_to_delete = "";

        //1. Get closest node (euclidean dist)
        for (std::string n : response->result)
        {
            // Node data = "id label type x y yaw"
            std::vector<std::string> node_data;
            node_data.clear();
            boost::split(node_data, n, boost::is_any_of(" "));

            //get Euclidean distance between clicked point and nodes (keep closest one)
            double d = std::sqrt( std::pow(pose_x - std::atof(node_data[3].c_str()),2) + std::pow(pose_y - std::atof(node_data[4].c_str()),2) );
            if (node_label_to_delete == "")
            {
                node_label_to_delete = node_data[1];
                min_dist = d;
            }
            else if (d < min_dist)
            {
                node_label_to_delete = node_data[1];
                min_dist = d;
            }
        }

        //2. Delete all the nodes matching this label (nodes of all types!)
        if (verbose) 
            RCLCPP_INFO(get_logger(), "[NavAssistant] Deleting Nodes with label %s", node_label_to_delete.c_str() );
        auto matchNodesRequest=std::make_shared<topology_graph::srv::Graph::Request>();
        matchNodesRequest->cmd = "GetNodesbyLabel";    // params: [node_label]
        matchNodesRequest->params.push_back( node_label_to_delete );
        
        auto future = graph_srv_client->async_send_request(matchNodesRequest);
        auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto matchResponse = future.get();

        if (matchResponse->success)
        {
            auto deleteRequest=std::make_shared<topology_graph::srv::Graph::Request>();
            // for each node with the matching label (delete it)
            for (std::string n : matchResponse->result)
            {
                // Node data = "id label type x y yaw"
                std::vector<std::string> node_data;
                node_data.clear();
                boost::split(node_data, n, boost::is_any_of(" "));

                deleteRequest->cmd = "DeleteNodeById";    // params: node_id
                deleteRequest->params.clear();
                deleteRequest->params.push_back( node_data[0] );
                auto future = graph_srv_client->async_send_request(matchNodesRequest);
            }
        }
    }

    // Done
    return true;
}


// ----------------------------------
// Navigation Assistant srv handler -
// ----------------------------------
bool CNavAssistant::srvCB(NAS::NavAssistantPoint::Request::SharedPtr req, NAS::NavAssistantPoint::Response::SharedPtr res)
{
    pointRequestQueue.push_back(req);
    return true;
}

void CNavAssistant::HandleGraphRequests()
{
    while(!pointRequestQueue.empty())
    {
        auto req = pointRequestQueue.front();
        pointRequestQueue.pop_front();

        // This SRV implements functions to Add or Delete node nodes in the graph (topology)
        // Get Yaw from Pose
        double yaw_angle = getYaw(req->pose.pose);

        // Add / Delete
        if (req->action == "add")
        {
            // Set a new label
            std::string label = req->type + "_" + std::to_string(counter);
            counter ++;
            if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant] Request to Add a Node," );

            // Execute action and return result
            if ( addNode(label, req->type, req->pose.pose.position.x, req->pose.pose.position.y, yaw_angle) )
            {
                if (req->type == "CP" || req->type == "CNP" || req->type == "passage")
                    regenerate_arcs();
            }

        }
        else if (req->action == "delete")
        {
            if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant] Request to Delete a Node," );
            if ( deleteNode(req->type, req->pose.pose.position.x, req->pose.pose.position.y, yaw_angle) )
            {
                if (req->type == "CP" || req->type == "CNP" || req->type == "passage")
                    regenerate_arcs();
            }
        }
        else if (req->action == "save")
        {
            // Keep copy of current Graph
            if (save_to_file != "")
            {
                // Save to file
                RCLCPP_INFO(get_logger(), "[NavAssistant] Saving Graph to file [%s]", save_to_file.c_str());
                //File contains a boost::graphviz description of the nodes and arcs
                auto saveRequest=std::make_shared<topology_graph::srv::Graph::Request>();
                saveRequest->cmd = "SaveGraph";    // params: [file_path]
                saveRequest->params.clear();
                saveRequest->params.push_back(save_to_file);
                    
                auto future = graph_srv_client->async_send_request(saveRequest);
                auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
                auto response = future.get();

                if (!response->success)
                    RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to Save Graph from file. Skipping.");
            }
        }
    }
}

// ----------------------------------
// Turn in place before Navigation  -
// ----------------------------------
void CNavAssistant::turn_towards_path(geometry_msgs::msg::PoseStamped pose_goal)
{
    // Use move_base service "make_plan" to set initial orientation
    auto getPlanReq=std::make_shared<NAS::MakePlan::Request>();
    auto response=std::make_shared<NAS::MakePlan::Response>();

    getPlanReq->start = current_robot_pose;
    // Set Goal pose
    getPlanReq->goal = pose_goal;
    
    // 1. Set starting pose (robot location)
    bool result = makePlan(getPlanReq, response);

    //Check if valid goal with move_base srv
    if(!result)
    {
        // SRV is not available!! Report Error
        RCLCPP_ERROR(get_logger(), "[NavAssistant-turn_towards_path] Unable to call MAKE_PLAN service from MoveBase");
        return;
    }
    else if ( response->plan.poses.empty() )
    {
        if (verbose) RCLCPP_WARN(get_logger(), "[NavAssistant-turn_towards_path] Unable to get plan");
        return;
    }
    else
    {
        try
        {
            // get initial orientation of path from vector<pose>
            double Ax, Ay, d = 0.0;
            geometry_msgs::msg::Pose p;
            p = response->plan.poses[response->plan.poses.size()-1].pose;     //init at goal
            for (size_t h=0; h<response->plan.poses.size(); h++)
            {
                if (h==0)
                {
                    Ax = getPlanReq->start.pose.position.x - response->plan.poses[h].pose.position.x;
                    Ay = getPlanReq->start.pose.position.y - response->plan.poses[h].pose.position.y;
                }
                else
                {
                    Ax = response->plan.poses[h-1].pose.position.x - response->plan.poses[h].pose.position.x;
                    Ay = response->plan.poses[h-1].pose.position.y - response->plan.poses[h].pose.position.y;
                }
                d += sqrt( pow(Ax,2) + pow(Ay,2) );

                //end condition [m]
                if (d >= 0.3)
                {
                    p = response->plan.poses[h].pose;
                    break;
                }
            }

            // The path does not cotains orientation, only positions, so we need to calculate the orientation between robot and target in the path
            Ax = p.position.x - current_robot_pose.pose.position.x;
            Ay = p.position.y - current_robot_pose.pose.position.y;
            double target_yaw_map = atan2(Ay,Ax);

            // Set goal in MoveBase
            geometry_msgs::msg::PoseStamped local_goal;
            local_goal.header = current_robot_pose.header;
            local_goal.pose.position = current_robot_pose.pose.position;
            tf2::Quaternion q;
            q.setRPY(0, 0, angles::normalize_angle(target_yaw_map));
            local_goal.pose.orientation = tf2::toMsg(q);
            if (verbose) 
                RCLCPP_INFO(get_logger(), "Requesting Initial Turn" );

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
        
        if(result == rclcpp::FutureReturnCode::SUCCESS)
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
        RCLCPP_INFO(get_logger(), "Requesting MoveBase Navigation to [%.2f, %.2f]", mb_goal.pose.pose.position.x, mb_goal.pose.pose.position.y );
    
    auto goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    goal_options.result_callback = std::bind(&NavigationGoal::result_cb, &m_currentGoal, _1);
    goal_options.goal_response_callback = std::bind(&NavigationGoal::response_cb, &m_currentGoal, _1);
    
    
    m_currentGoal.complete = false;
    
    auto future = mb_action_client->async_send_goal(mb_goal, goal_options);
    rclcpp::spin_until_future_complete(shared_from_this(), future);

    rclcpp::Rate rate(10);
    while(rclcpp::ok() && !m_currentGoal.complete && !m_currentGoal.goalCancelled)
    {
        //if there was explicit cancellation or just a new goal overwriting the current one
        if(m_currentGoal.goalCancelled || m_currentGoal.ServerGoalHandle != currentServerGoal)
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
rclcpp_action::GoalResponse CNavAssistant::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NAS_ac::NavAssistant::Goal> goal)
{
    if(verbose)
        RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CNavAssistant::handle_cancel( const std::shared_ptr<GoalHandleNavigate_Server> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    cancelCurrentGoal();
    mb_action_client->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CNavAssistant::handle_accepted(const std::shared_ptr<GoalHandleNavigate_Server> goal_handle)
{
    if(verbose)
        RCLCPP_INFO(this->get_logger(), "Accepted goal");
    if(m_currentGoal.ClientGoalHandle.get()!=nullptr)
        cancelCurrentGoal();
    m_currentGoal.ServerGoalHandle=goal_handle;
}

void CNavAssistant::execute()
{
    // This ActionServer is a wrapper for move_base aiming at a more robust navvigation and error handling
    try
    {
        auto goal = m_currentGoal.ServerGoalHandle->get_goal();
        if (verbose) 
            RCLCPP_INFO(get_logger(), "Starting Navigation Assistant to reach: [%.2f, %.2f]", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        
        // Get path (robot->goal) from topology-graph node
        std::string node_start, node_end;
        std::vector<std::string> path;

        //1. Get starting ING node
        auto graphRequest=std::make_shared<topology_graph::srv::Graph::Request>();
        graphRequest->cmd = "GetClosestNode";    // params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
        graphRequest->params.push_back(std::to_string(current_robot_pose.pose.position.x));
        graphRequest->params.push_back(std::to_string(current_robot_pose.pose.position.y));
        graphRequest->params.push_back(std::to_string(0.0));
        graphRequest->params.push_back("ING");
        
        auto future = graph_srv_client->async_send_request(graphRequest);
        auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
        auto response = future.get();

        if (response->success)
        {
            node_start = response->result[0];                // result: [id label type x y yaw]
            if (verbose) 
                RCLCPP_INFO(get_logger(), "[NavAssistant]: Starting Node is %s - %s", node_start.c_str(), response->result[1].c_str() );


            // 2. Get ending ING node
            graphRequest->cmd = "GetClosestNode";    // params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
            graphRequest->params.clear();
            response->result.clear();
            graphRequest->params.push_back(std::to_string(goal->target_pose.pose.position.x));
            graphRequest->params.push_back(std::to_string(goal->target_pose.pose.position.y));
            graphRequest->params.push_back(std::to_string(0.0));
            graphRequest->params.push_back("ING");
            future = graph_srv_client->async_send_request(graphRequest);
            result = rclcpp::spin_until_future_complete(shared_from_this(), future);
            response = future.get();
            if (response->success)
            {
                node_end = response->result[0];                // result: [id label type x y yaw]
                if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant]: End Node is %s -%s", node_end.c_str(), response->result[1].c_str() );

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
                            if (node0[1] != node1[1])   //Orphan node. Remove it
                                path.erase(path.begin());
                            // END node
                            boost::split(node0, path[path.size()-1], boost::is_any_of(" "));
                            boost::split(node1, path[path.size()-2], boost::is_any_of(" "));
                            if (node0[1] != node1[1])   //Orphan node. Remove it
                                path.erase(path.end());


                            // Step by Step NAVIGATION
                            for (size_t i=0; i<path.size(); i++)
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
                                if (i < (path.size() - 1) )
                                {
                                    // Head towards next node
                                    boost::split(next_node_data, path[i+1], boost::is_any_of(" "));
                                    double Ax = atof(next_node_data[3].c_str()) - atof(current_node_data[3].c_str());
                                    double Ay = atof(next_node_data[4].c_str()) - atof(current_node_data[4].c_str());
                                    target_yaw_map = atan2(Ay,Ax);
                                }
                                else
                                {
                                    //Head towards goal!
                                    double Ax = goal->target_pose.pose.position.x - atof(current_node_data[3].c_str());
                                    double Ay = goal->target_pose.pose.position.y - atof(current_node_data[4].c_str());
                                    target_yaw_map = atan2(Ay,Ax);
                                }
                                // Set orientation
                                tf2::Quaternion q;
                                q.setRPY(0, 0, angles::normalize_angle(target_yaw_map));
                                i_goal.pose.orientation = tf2::toMsg(q);


                                //A. Initial Turn
                                if(m_currentGoal.checkIfCancelled(shared_from_this()))
                                    return;
                                if (goal->turn_before_nav)
                                    turn_towards_path(i_goal);


                                //B. Navigate to i_goal using MOVE_BASE!
                                if(m_currentGoal.checkIfCancelled(shared_from_this()))
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
                        if(verbose)
                            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to get Path. Trying direct navigation.");
                    }
                } //end if-start!=end
            }
            else if (verbose)
                RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to get Closest ING Node (end) pose=[%.3f, %.3f]. Trying direct navigation.", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        }
        else if (verbose)
            RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to get Closest ING Node (start) pose=[%.3f, %.3f]. Trying direct navigation.", current_robot_pose.pose.position.x, current_robot_pose.pose.position.y );



        // -------------------------------
        // NAVIGATION TO TARGET GOAL (MB)
        // -------------------------------
        
        if(m_currentGoal.checkIfCancelled(shared_from_this()))
            return;
        
        //1. Initial Turn
        if (goal->turn_before_nav)
            turn_towards_path(goal->target_pose);

        rclcpp::spin_some(shared_from_this());

        if(m_currentGoal.checkIfCancelled(shared_from_this()))
            return;
        //2. Navigate to goal
        move_base_nav_and_wait(goal->target_pose);

        //reset the cancelled flag if the cancellation happened after the second goal was sent
        m_currentGoal.checkIfCancelled(shared_from_this());
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

    if (verbose) RCLCPP_INFO(get_logger(), "[NavAssistant] Closing Action...");
}

CNavAssistant::~CNavAssistant(){}

//===================================================================================
//================================== MAIN ===========================================
//===================================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    std::shared_ptr<CNavAssistant> nav_assistant = std::make_shared<CNavAssistant>("nav_assistant");
    nav_assistant->Init();
    RCLCPP_INFO(nav_assistant->get_logger(), "[NavAssistant] Action server is ready for action!...");

    rclcpp::Rate rate(20);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(nav_assistant);
        nav_assistant->HandleGraphRequests();

        if(nav_assistant->m_currentGoal.ServerGoalHandle.get() != nullptr)
        {
            auto currentServerGoalHandle = nav_assistant->m_currentGoal.ServerGoalHandle; 
            nav_assistant->execute();
            
            //dont reset the variable if a new goal was received before the last one was completed
            if(currentServerGoalHandle == nav_assistant->m_currentGoal.ServerGoalHandle)
                nav_assistant->m_currentGoal.ServerGoalHandle = {nullptr};
        }
        rate.sleep();
    }

    return 0;
}
