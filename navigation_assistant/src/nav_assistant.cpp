#include "nav_assistant.h"
#include <iostream>

using json = nlohmann::json;

//-----------------------------------------------------------
//                    Subscriptions
//----------------------------------------------------------

// Robot location update
void CNavAssistant::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    //keep the most recent robot pose = position + orientation
    current_robot_pose = *msg;
}



//-----------------------------------------------------------
//                    Action Server Initialization
//----------------------------------------------------------
CNavAssistant::CNavAssistant(std::string name) :
    as_(nh_, name, boost::bind(&CNavAssistant::executeAS, this, _1),false),
    mb_action_client("move_base",true)
{
    /*! IMPORTANT
     * DEFINE all Action Servers to be used or published before executing the callback to avoid problems
     * */

    // Read config parameters
    ros::NodeHandle pn("~");
    pn.param<bool>("verbose", verbose, false);
    pn.param<bool>("init_from_param", init_from_param, false);
    pn.param<std::string>("topological_json_parameter",topology_parameter,"/topological_map");
    pn.param<bool>("load_passages_as_CP", load_passages_as_CP, false);
    pn.param<bool>("force_CP_as_additional_ANP", force_CP_as_additional_ANP, false);
    pn.param<std::string>("init_from_file",init_from_file,"~/topological_map");
    pn.param<std::string>("save_to_file",save_to_file,"");


    makePlanServer = pn.advertiseService("make_plan", &CNavAssistant::makePlan, this);


    // Service clients
    mb_srv_client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");  // only accounts for global_costmap but works at all times
    graph_srv_client = nh_.serviceClient<topology_graph::graph>("topology_graph/graph");    // Graph service client
    nav_assist_functions_client = nh_.serviceClient<navigation_assistant::nav_assistant_poi>("navigation_assistant/get_poi_related_poses");
    nav_assist_functions_client2 = nh_.serviceClient<navigation_assistant::nav_assistant_set_CNP>("navigation_assistant/get_cnp_pose_around");

    // Subscribers
    localization_sub_ = nh_.subscribe("/amcl_pose",100,&CNavAssistant::localizationCallback,this);

    // Publishers
    cmd_vel_publisher = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ready_publisher = nh_.advertise<std_msgs::Bool>("/nav_assistant/ready",1);


    // WAITs
    while (!mb_action_client.waitForServer(ros::Duration(30)))
        ROS_ERROR("[NavAssistant] Unable to contact with MoveBase serer! waiting...");
    while (!mb_srv_client.waitForExistence(ros::Duration(30)) )
        ROS_ERROR("[NavAssistant] Unable to contact with MoveBase make_plan srv! waiting...");
    while (!graph_srv_client.waitForExistence(ros::Duration(30)) )
        ROS_ERROR("[NavAssistant] Unable to contact with the GRAPH srv! waiting...");
    while (!nav_assist_functions_client.waitForExistence(ros::Duration(30)) )
        ROS_ERROR("[NavAssistant] Unable to contact with the navigation_assistant functions srv! waiting...");
    while (!nav_assist_functions_client2.waitForExistence(ros::Duration(30)) )
        ROS_ERROR("[NavAssistant] Unable to contact with the navigation_assistant functions srv! waiting...");


    // All srv and actions are ready to be used.
    // Initialize Navigation Graph
    if (init_from_file != "")
    {
        // The topology has been previously saved to file. Reload it.
        ROS_INFO("[NavAssistant] Loading Graph from file [%s]", init_from_file.c_str());
        //File contains a boost::graphviz description of the nodes and arcs
        topology_graph::graph srv_call2;
        srv_call2.request.cmd = "LoadGraph";    // params: [file_path]
        srv_call2.request.params.clear();
        srv_call2.request.params.push_back(init_from_file);
        graph_srv_client.call(srv_call2);
        if (!srv_call2.response.success)
            ROS_WARN("[NavAssistant]: Unable to Load Graph from file. Skipping.");
    }

    if (init_from_param)
    {
        // The topology is provided externally (MoveCare project)
        ROS_INFO("[NavAssistant] Loading Graph from parameter [%s]", topology_parameter.c_str());
        //Get global parameter with the JSON description of the topology
        bool topo_found = false;
        while (ros::ok() && !topo_found)
        {
            if (nh_.hasParam(topology_parameter))
            {
                nh_.param<std::string>(topology_parameter, topology_str, "");
                json json_msg = json::parse( topology_str );

                // Process JSON
                get_graph_data_from_json(json_msg);

                // Done
                topo_found = true;
            }
            else
            {
                ROS_WARN("[topology_graph] Waiting for topological_map global parameter (see TaskCoordinator node).");
                ros::Duration(2.0).sleep();
            }
        }
    }
    // Disable the init_from_param to avoid reloading the topology in case or node-relaunch
    pn.setParam("init_from_param", false);

    // Action server has been initialized and the executeAS linked!. Start offering the server over ROS
    as_.start();

    // Advertise service to add new points
    service = nh_.advertiseService("/navigation_assistant/add_point_of_interest", &CNavAssistant::srvCB, this);

    std_msgs::Bool ready_msg;
    ready_msg.data = true;
    ready_publisher.publish(ready_msg);

    // init internal variables
    counter = 0;
    ROS_INFO("[NavAssistant] Node ready for action!");
}

bool CNavAssistant::makePlan(navigation_assistant::make_plan::Request& request, navigation_assistant::make_plan::Response& response)
{
    nav_msgs::GetPlan mb_srv;

    // 1. Set starting pose (robot location)
    mb_srv.request.start.header.frame_id = "map";
    mb_srv.request.start.header.stamp = ros::Time::now();
    mb_srv.request.start.pose = current_robot_pose.pose.pose;
    // Set Goal pose

    mb_srv.request.goal.header.frame_id = "map";
    mb_srv.request.goal.header.stamp = ros::Time::now();
    mb_srv.request.goal.pose.position.x = request.x;
    mb_srv.request.goal.pose.position.y = request.y;
    mb_srv.request.goal.pose.position.z = request.z;

    //Check if valid goal with move_base srv
    if( !mb_srv_client.call(mb_srv) )
    {
        // SRV is not available!! Report Error
        ROS_ERROR("[NavAssistant-turn_towards_path] Unable to call MAKE_PLAN service from MoveBase");
        response.validPath = false;
        return false;
    }
    else if ( mb_srv.response.plan.poses.empty() )
    {
        if (verbose) ROS_WARN("[NavAssistant-turn_towards_path] Unable to get plan");
        response.validPath = false;
        return true;
    }

    response.validPath = true;
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
        ROS_ERROR("[topology_graph] Error getting Poses. Error is: [%s]", e.what());
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
        ROS_ERROR("[NavAssistant] Error loading graph data. Spaces list malformed [%s]", e.what());
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
        ROS_ERROR("[NavAssistant] Error loading graph data. Passages list malformed [%s]", e.what());
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
        ROS_ERROR("[taskCoordinator] Error loading Env data. Passages list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        return;
    }

    // Finally, regenerate Arcs
    ros::Duration(1.0).sleep();
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
    geometry_msgs::PoseStamped node_pose;
    node_pose.header.frame_id = "map";
    node_pose.header.stamp = ros::Time(0);
    node_pose.pose.position.x = pose_x;
    node_pose.pose.position.y = pose_y;
    node_pose.pose.position.z = 0.0;
    node_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_yaw);

    if (node_type == "CNP" || node_type == "CP" || node_type == "passage")
    {
        if (verbose) ROS_INFO("[NavAssistant] Estimating optimal pose for CNP around [%.3f, %.3f]", pose_x, pose_y);
        // Estimate best location for the CP/CNP/passage around given pose
        navigation_assistant::nav_assistant_set_CNP srv_call;
        srv_call.request.pose = node_pose;
        nav_assist_functions_client2.call(srv_call);

        //Update pose
        if (srv_call.response.success)
        {
            node_pose = srv_call.response.pose;
            if (verbose) ROS_INFO("[NavAssistant] Setting CNP at [%.3f, %.3f]", node_pose.pose.position.x, node_pose.pose.position.y);
        }
        else
            if (verbose) ROS_INFO("[NavAssistant] Error setting CNP around [%.3f, %.3f], Using exact point.", pose_x, pose_y);
    }


    // 1. Add a new node in the graph
    topology_graph::graph srv_call;
    srv_call.request.cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
    srv_call.request.params.push_back(node_label);
    srv_call.request.params.push_back(node_type);
    srv_call.request.params.push_back(std::to_string(node_pose.pose.position.x));
    srv_call.request.params.push_back(std::to_string(node_pose.pose.position.y));
    srv_call.request.params.push_back(std::to_string(pose_yaw));
    graph_srv_client.call(srv_call);
    if (!srv_call.response.success)
    {
        ROS_WARN("[NavAssistant]: Unable to create new Node");
        return false;
    }    

    // According to the type of node, additional nodes may be necessary
    if (node_type == "passage")
    {
        // Estimate and Add the two Segment Points (SP) that define this passage
        navigation_assistant::nav_assistant_poi poi_srv_call;
        poi_srv_call.request.pose = node_pose;
        nav_assist_functions_client.call(poi_srv_call);

        if (poi_srv_call.response.success)
        {
            for (auto p: poi_srv_call.response.sp)
            {
                srv_call.request.cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
                srv_call.request.params.clear();
                srv_call.request.params.push_back(node_label);   //use the same label as the original POI
                srv_call.request.params.push_back("SP");
                srv_call.request.params.push_back(std::to_string(p.pose.position.x));
                srv_call.request.params.push_back(std::to_string(p.pose.position.y));
                tf::Pose tfpose;
                tf::poseMsgToTF(p.pose, tfpose);
                double yaw_angle = tf::getYaw(tfpose.getRotation());
                srv_call.request.params.push_back(std::to_string(yaw_angle));
                graph_srv_client.call(srv_call);
                if (!srv_call.response.success)
                {
                    ROS_WARN("[NavAssistant]: Unable to create new Node");
                    return false;
                }
            }
        }
    }

    // Critical Navigation Point (CNP) , Critial Passage (CP)
    else if ( (node_type == "CNP") || (node_type == "CP") )
    {
        // Estimate and Add the two Segment Points (SP) and the two Intermediate Navigation Goals (ING)
        navigation_assistant::nav_assistant_poi poi_srv_call;
        poi_srv_call.request.pose = node_pose;
        nav_assist_functions_client.call(poi_srv_call);

        if (poi_srv_call.response.success)
        {
            //2.1 Add "SP" nodes (Segment Points)
            for (auto p: poi_srv_call.response.sp)
            {
                srv_call.request.cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
                srv_call.request.params.clear();
                srv_call.request.params.push_back(node_label);   //use the same label as the original POI
                srv_call.request.params.push_back("SP");
                srv_call.request.params.push_back(std::to_string(p.pose.position.x));
                srv_call.request.params.push_back(std::to_string(p.pose.position.y));
                tf::Pose tfpose;
                tf::poseMsgToTF(p.pose, tfpose);
                double yaw_angle = tf::getYaw(tfpose.getRotation());
                srv_call.request.params.push_back(std::to_string(yaw_angle));
                graph_srv_client.call(srv_call);
                if (!srv_call.response.success)
                {
                    ROS_WARN("[NavAssistant]: Unable to create new Node");
                    return false;
                }
            }

            //2.2 Add a "ING" nodes (intermediate navigation goals)
            std::vector<std::string> ing_id;
            for (auto p: poi_srv_call.response.ing)
            {
                srv_call.request.cmd = "AddNode";    // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
                srv_call.request.params.clear();
                srv_call.request.params.push_back(node_label);   //use the same label as the original POI
                srv_call.request.params.push_back("ING");
                srv_call.request.params.push_back(std::to_string(p.pose.position.x));
                srv_call.request.params.push_back(std::to_string(p.pose.position.y));
                tf::Pose tfpose;
                tf::poseMsgToTF(p.pose, tfpose);
                double yaw_angle = tf::getYaw(tfpose.getRotation());
                srv_call.request.params.push_back(std::to_string(yaw_angle));
                graph_srv_client.call(srv_call);
                if (!srv_call.response.success)
                {
                    ROS_WARN("[NavAssistant]: Unable to create new Node");
                    return false;
                }

                //keep node_id
                std::string current_node_id = srv_call.response.result[0];
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
    if (verbose) ROS_INFO("[NavAssistant] Regenerating Arcs in Graph");
    topology_graph::graph srv_call;
    srv_call.request.cmd = "DeleteAllArcs";    // params: [node_id]
    srv_call.request.params.push_back("-1");     // All nodes = -1
    graph_srv_client.call(srv_call);

    /* Regenerate Arcs
     * Only nodes of type: ING, and optionally CP and CNP are connected in the graph
     * connections avoid Passages and Segment Points
     */
    std::vector<string> nodes_to_connect;
    nodes_to_connect.clear();
    topology_graph::graph srv_call2;
    // ING nodes
    srv_call2.request.cmd = "GetNodesbyType";    // params: node_type
    srv_call2.request.params.clear();
    srv_call2.request.params.push_back("ING");
    graph_srv_client.call(srv_call2);
    if (!srv_call2.response.success)
    {
        ROS_WARN("[NavAssistant-regenerateArcs]: Unable to get ING Nodes from Graph");
        return;
    }
    else
        nodes_to_connect.insert(nodes_to_connect.end(), srv_call2.response.result.begin(), srv_call2.response.result.end());

    if (force_CP_as_additional_ANP)
    {
        // CP nodes
        srv_call2.request.cmd = "GetNodesbyType";    // params: node_type
        srv_call2.request.params.clear();
        srv_call2.request.params.push_back("CP");
        graph_srv_client.call(srv_call2);
        if (!srv_call2.response.success)
        {
            ROS_WARN("[NavAssistant-regenerateArcs]: Unable to get CP Nodes from Graph");
            return;
        }
        else
            nodes_to_connect.insert(nodes_to_connect.end(), srv_call2.response.result.begin(), srv_call2.response.result.end());


        // CNP nodes
        srv_call2.request.cmd = "GetNodesbyType";    // params: node_type
        srv_call2.request.params.clear();
        srv_call2.request.params.push_back("CNP");
        graph_srv_client.call(srv_call2);
        if (!srv_call2.response.success)
        {
            ROS_WARN("[NavAssistant-regenerateArcs]: Unable to get ING Nodes from Graph");
            return;
        }
        else
            nodes_to_connect.insert(nodes_to_connect.end(), srv_call2.response.result.begin(), srv_call2.response.result.end());
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
                    topology_graph::graph srv_call3;
                    srv_call3.request.cmd = "AddArc";    // params: [idfrom, idto, label, type, bidirectional]
                    srv_call3.request.params.clear();
                    srv_call3.request.params.push_back(node_data[0]);
                    srv_call3.request.params.push_back(node_data2[0]);
                    srv_call3.request.params.push_back("arc");
                    srv_call3.request.params.push_back("arc");
                    srv_call3.request.params.push_back("true");       // bidirectional arc
                    graph_srv_client.call(srv_call3);
                    if (!srv_call3.response.success)
                        ROS_WARN("[NavAssistant]: Unable to create Arc");
                }
                // Create arc if different types
                else if (node_data[2] != node_data2[2])
                {
                    //Create Arc
                    topology_graph::graph srv_call3;
                    srv_call3.request.cmd = "AddArc";    // params: [idfrom, idto, label, type, bidirectional]
                    srv_call3.request.params.clear();
                    srv_call3.request.params.push_back(node_data[0]);
                    srv_call3.request.params.push_back(node_data2[0]);
                    srv_call3.request.params.push_back("arc");
                    srv_call3.request.params.push_back("arc");
                    srv_call3.request.params.push_back("true");       // bidirectional arc
                    graph_srv_client.call(srv_call3);
                    if (!srv_call3.response.success)
                        ROS_WARN("[NavAssistant]: Unable to create Arc");
                }
            }

            // Different Labels
            else
            {
                if (node_data[2]=="ING" && node_data2[2]=="ING")
                {
                    // Create arc if different label and type ING
                    // Check if there is a path not crossing a segment defined by a CNP or CP
                    topology_graph::graph srv_call_dist;
                    srv_call_dist.request.cmd = "GetNavDistTwoPoses";    // params: p1(x,y,yaw), p2(x,y,yaw), [avoid_node_types]
                    srv_call_dist.request.params.clear();
                    // p1
                    srv_call_dist.request.params.push_back(node_data[3].c_str());
                    srv_call_dist.request.params.push_back(node_data[4].c_str());
                    srv_call_dist.request.params.push_back(node_data[5].c_str());
                    // p2
                    srv_call_dist.request.params.push_back(node_data2[3].c_str());
                    srv_call_dist.request.params.push_back(node_data2[4].c_str());
                    srv_call_dist.request.params.push_back(node_data2[5].c_str());
                    // avoid intersecting nodes of type
                    srv_call_dist.request.params.push_back("CNP");
                    srv_call_dist.request.params.push_back("CP");
                    graph_srv_client.call(srv_call_dist);

                    if (verbose) ROS_WARN("[NavAssistant]: NavDistance between Node[%s] <-> [%s] is %s (succes=%d)",node_data[0].c_str(), node_data2[0].c_str(), srv_call_dist.response.result[0].c_str(), srv_call_dist.response.success);

                    if (srv_call_dist.response.success)
                    {
                        //system("read -p 'Press Enter to continue...' var");
                        if (verbose) ROS_WARN("[NavAssistant]: Adding new Arc between ING Nodes [%s] <-> [%s]", node_data[0].c_str(), node_data2[0].c_str());
                        //Create Arc
                        topology_graph::graph srv_call2;
                        srv_call2.request.cmd = "AddArc";    // params: [idfrom, idto, label, type, bidirectional]
                        srv_call2.request.params.clear();
                        srv_call2.request.params.push_back(node_data[0]);
                        srv_call2.request.params.push_back(node_data2[0]);
                        srv_call2.request.params.push_back("arc");
                        srv_call2.request.params.push_back("arc");
                        srv_call2.request.params.push_back("true");       // bidirectional arc
                        graph_srv_client.call(srv_call2);
                        if (!srv_call2.response.success)
                            ROS_WARN("[NavAssistant]: Unable to create Arc");
                    }
                }
            }
        }//end.for each other ING
    }//end-for each ING
}




bool CNavAssistant::deleteNode(std::string node_type, double pose_x, double pose_y, double pose_yaw)
{
    if (verbose) ROS_INFO("[NavAssistant] Deleting Node of type [%s], close to [%.3f, %.3f]", node_type.c_str(), pose_x, pose_y );
    //1. Get all Nodes of type: node_type
    topology_graph::graph srv_call;
    srv_call.request.cmd = "GetNodesbyType";    // params: [node_type]
    srv_call.request.params.push_back(node_type);
    graph_srv_client.call(srv_call);

    if (srv_call.response.success)
    {
        double min_dist;
        std::string node_label_to_delete = "";

        //1. Get closest node (euclidean dist)
        for (std::string n : srv_call.response.result)
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
        if (verbose) ROS_INFO("[NavAssistant] Deleting Nodes with label %s", node_label_to_delete.c_str() );
        topology_graph::graph srv_call2;
        srv_call2.request.cmd = "GetNodesbyLabel";    // params: [node_label]
        srv_call2.request.params.push_back( node_label_to_delete );
        graph_srv_client.call(srv_call2);

        if (srv_call2.response.success)
        {
            topology_graph::graph srv_call3;
            // for each node with the matching label (delete it)
            for (std::string n : srv_call2.response.result)
            {
                // Node data = "id label type x y yaw"
                std::vector<std::string> node_data;
                node_data.clear();
                boost::split(node_data, n, boost::is_any_of(" "));

                srv_call3.request.cmd = "DeleteNodeById";    // params: node_id
                srv_call3.request.params.clear();
                srv_call3.request.params.push_back( node_data[0] );
                graph_srv_client.call(srv_call3);
            }
        }
    }

    // Done
    return true;
}


// ----------------------------------
// Navigation Assistant srv handler -
// ----------------------------------
bool CNavAssistant::srvCB(navigation_assistant::nav_assistant_point::Request &req, navigation_assistant::nav_assistant_point::Response &res)
{
    // This SRV implements functions to Add or Delete node nodes in the graph (topology)
    // Get Yaw from Pose
    tf::Pose tfpose;
    tf::poseMsgToTF(req.pose.pose, tfpose);
    double yaw_angle = tf::getYaw(tfpose.getRotation());

    // Add / Delete
    if (req.action == "add")
    {
        // Set a new label
        std::string label = req.type + "_" + std::to_string(counter);
        counter ++;

        // Execute action and return result
        if ( addNode(label, req.type, req.pose.pose.position.x, req.pose.pose.position.y, yaw_angle) )
        {
            if (req.type == "CP" || req.type == "CNP" || req.type == "passage")
                regenerate_arcs();
            return true;
        }
        else
            return false;

    }
    else if (req.action == "delete")
    {
        if (verbose) ROS_INFO("[NavAssistant] Request to Delete a Node," );
        if ( deleteNode(req.type, req.pose.pose.position.x, req.pose.pose.position.y, yaw_angle) )
        {
            if (req.type == "CP" || req.type == "CNP" || req.type == "passage")
                regenerate_arcs();
            return true;
        }
        else
            return false;
    }
    else if (req.action == "save")
    {
        // Keep copy of current Graph
        if (save_to_file != "")
        {
            // Save to file
            ROS_INFO("[NavAssistant] Saving Graph to file [%s]", save_to_file.c_str());
            //File contains a boost::graphviz description of the nodes and arcs
            topology_graph::graph srv_call2;
            srv_call2.request.cmd = "SaveGraph";    // params: [file_path]
            srv_call2.request.params.clear();
            srv_call2.request.params.push_back(save_to_file);
            graph_srv_client.call(srv_call2);
            if (!srv_call2.response.success)
                ROS_WARN("[NavAssistant]: Unable to Save Graph from file. Skipping.");
            return true;
        }
        else
            return false;
    }
    return false;
}



// ----------------------------------
// Turn in place before Navigation  -
// ----------------------------------
void CNavAssistant::turn_towards_path(geometry_msgs::PoseStamped pose_goal)
{
    // Use move_base service "make_plan" to set initial orientation
    nav_msgs::GetPlan mb_srv;

    // 1. Set starting pose (robot location)
    mb_srv.request.start.header.frame_id = "map";
    mb_srv.request.start.header.stamp = ros::Time::now();
    mb_srv.request.start.pose = current_robot_pose.pose.pose;
    // Set Goal pose
    mb_srv.request.goal = pose_goal;

    //Check if valid goal with move_base srv
    if( !mb_srv_client.call(mb_srv) )
    {
        // SRV is not available!! Report Error
        ROS_ERROR("[NavAssistant-turn_towards_path] Unable to call MAKE_PLAN service from MoveBase");
        return;
    }
    else if ( mb_srv.response.plan.poses.empty() )
    {
        if (verbose) ROS_WARN("[NavAssistant-turn_towards_path] Unable to get plan");
        return;
    }
    else
    {
        try
        {
            // get initial orientation of path from vector<pose>
            double Ax, Ay, d = 0.0;
            geometry_msgs::Pose p;
            p = mb_srv.response.plan.poses[mb_srv.response.plan.poses.size()-1].pose;     //init at goal
            for (size_t h=0; h<mb_srv.response.plan.poses.size(); h++)
            {
                if (h==0)
                {
                    Ax = mb_srv.request.start.pose.position.x - mb_srv.response.plan.poses[h].pose.position.x;
                    Ay = mb_srv.request.start.pose.position.y - mb_srv.response.plan.poses[h].pose.position.y;
                }
                else
                {
                    Ax = mb_srv.response.plan.poses[h-1].pose.position.x - mb_srv.response.plan.poses[h].pose.position.x;
                    Ay = mb_srv.response.plan.poses[h-1].pose.position.y - mb_srv.response.plan.poses[h].pose.position.y;
                }
                d += sqrt( pow(Ax,2) + pow(Ay,2) );

                //end condition [m]
                if (d >= 0.3)
                {
                    p = mb_srv.response.plan.poses[h].pose;
                    break;
                }
            }

            // The path does not cotains orientation, only positions, so we need to calculate the orientation between robot and target in the path
            Ax = p.position.x - current_robot_pose.pose.pose.position.x;
            Ay = p.position.y - current_robot_pose.pose.pose.position.y;
            double target_yaw_map = atan2(Ay,Ax);

            // Set goal in MoveBase
            geometry_msgs::PoseStamped local_goal;
            local_goal.header = current_robot_pose.header;
            local_goal.pose.position = current_robot_pose.pose.pose.position;
            tf::Quaternion q;
            q.setRPY(0, 0, angles::normalize_angle(target_yaw_map));
            tf::quaternionTFToMsg(q, local_goal.pose.orientation);
            if (verbose) ROS_INFO("[NavAssistant] Requesting Initial Turn" );

            // Call MoveBase (turn in place)
            move_base_nav_and_wait(local_goal);

            return;
        }
        catch (exception e)
        {
            ROS_ERROR("[taskCoordinator] Error in turn_towards_path: [%s]", e.what());
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
        if (mb_action_client.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ros::Time start_time = ros::Time::now();

            // Request goal cancelation (asincronous)
            mb_action_client.cancelAllGoals();

            //Wait for action server to get state PREEMPTED or timeout
            while (mb_action_client.getState() == actionlib::SimpleClientGoalState::ACTIVE)
            {
                if (verbose) ROS_INFO("[NavAssistant-CancelActionAndWait] Waiting for ActionServer to Cancel the goal");

                //check if timeout
                if( (ros::Time::now() - start_time).toSec() > wait_time_sec )
                {
                    if (verbose) ROS_INFO("[NavAssistant-CancelActionAndWait] Timeout Reached...");
                    return false;
                }

                //Sleep a while
                ros::Duration(0.2).sleep();
            }

            // Action server is now in state PREEMPTED/SUCCESS/ABORTED but not Active!
            return true;
        }
        else
        {
            if (verbose) ROS_INFO("[NavAssistant-CancelActionAndWait] AS is not ACTIVE... skipping request");
            return true;
        }
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator-CancelActionAndWait] Exception: [%s]", e.what());
        return false;
    }
}



// ----------------------------------------
// Trigger Navigation and wait completion -
// ----------------------------------------
void CNavAssistant::move_base_nav_and_wait(geometry_msgs::PoseStamped pose_goal)
{
    // request MoveBase to reach a goal
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = pose_goal;
    mb_goal.target_pose.header.frame_id="map";
    if (verbose) ROS_INFO("[NavAssistant] Requesting MoveBase Navigation to [%.2f, %.2f]", mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y );
    mb_action_client.sendGoal(mb_goal);

    // Execute Navigation without blocking!
    while ( !mb_action_client.waitForResult(ros::Duration(2.0)) )
    {
        // Check that preempt has not been requested by the client (that is, canceled)
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_WARN("[NavAssistant-MoveBaseNav]: has been Preempted");
            mb_action_client.cancelAllGoals();
            return;
        }
    }

    if (verbose) ROS_INFO("[NavAssistant] MoveBase navigation has finished in state [%s]", mb_action_client.getState().toString().c_str() );
}



//=================================================================
// Action Server Callback -> New Goal for the NavAssistant (START!)
//=================================================================
void CNavAssistant::executeAS(const navigation_assistant::nav_assistantGoalConstPtr &goal)
{
    // This ActionServer is a wrapper for move_base aiming at a more robust navvigation and error handling
    try
    {
        if (verbose) ROS_INFO("[NavAssistant] Starting Navigation Assistant to reach: [%.2f, %.2f]", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        result_.trace = "";

        // Get path (robot->goal) from topology-graph node
        std::string node_start, node_end;
        std::vector<std::string> path;

        //1. Get starting ING node
        topology_graph::graph srv_call;
        srv_call.request.cmd = "GetClosestNode";    // params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
        srv_call.request.params.push_back(std::to_string(current_robot_pose.pose.pose.position.x));
        srv_call.request.params.push_back(std::to_string(current_robot_pose.pose.pose.position.y));
        srv_call.request.params.push_back(std::to_string(0.0));
        srv_call.request.params.push_back("ING");
        graph_srv_client.call(srv_call);
        if (srv_call.response.success)
        {
            node_start = srv_call.response.result[0];                // result: [id label type x y yaw]
            if (verbose) ROS_INFO("[NavAssistant]: Starting Node is %s - %s", node_start.c_str(), srv_call.response.result[1].c_str() );


            // 2. Get ending ING node
            srv_call.request.cmd = "GetClosestNode";    // params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
            srv_call.request.params.clear();
            srv_call.response.result.clear();
            srv_call.request.params.push_back(std::to_string(goal->target_pose.pose.position.x));
            srv_call.request.params.push_back(std::to_string(goal->target_pose.pose.position.y));
            srv_call.request.params.push_back(std::to_string(0.0));
            srv_call.request.params.push_back("ING");
            graph_srv_client.call(srv_call);
            if (srv_call.response.success)
            {
                node_end = srv_call.response.result[0];                // result: [id label type x y yaw]
                if (verbose) ROS_INFO("[NavAssistant]: End Node is %s -%s", node_end.c_str(), srv_call.response.result[1].c_str() );

                // Navigation to Intermediate Navigation Goals (ING) if any
                if (node_start != node_end)
                {
                    // Find Path
                    // params: [nodeID_start, nodeID_end]
                    // result: Success/Failure. On success the list of Nodes ["id1 label1 type1 x1 y1 yaw", ..., ""idN labelN typeN xN yN yawN"]
                    srv_call.request.cmd = "FindPath";
                    srv_call.request.params.clear();
                    srv_call.response.result.clear();
                    srv_call.request.params.push_back(node_start);
                    srv_call.request.params.push_back(node_end);
                    graph_srv_client.call(srv_call);
                    if (srv_call.response.success)
                    {
                        path = srv_call.response.result;
                        if (verbose)
                        {
                            for (auto n : path)
                                ROS_INFO("[NavAssistant]: %s", n.c_str());
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
                                geometry_msgs::PoseStamped i_goal;
                                i_goal.header.frame_id = "map";
                                i_goal.header.stamp = ros::Time(0);
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
                                tf::Quaternion q;
                                q.setRPY(0, 0, angles::normalize_angle(target_yaw_map));
                                tf::quaternionTFToMsg(q, i_goal.pose.orientation);


                                //A. Initial Turn
                                if (goal->turn_before_nav)
                                    turn_towards_path(i_goal);

                                if (as_.isPreemptRequested() || !ros::ok())
                                {
                                    ROS_WARN("[NavAssistant]: has been Preempted");
                                    // set the action state to preempted
                                    result_.trace += " - Preemted";
                                    as_.setPreempted(result_);
                                    close_and_return();
                                    return;
                                }

                                //B. Navigate to i_goal using MOVE_BASE!
                                move_base_nav_and_wait(i_goal);

                                if (as_.isPreemptRequested() || !ros::ok())
                                {
                                    ROS_WARN("[NavAssistant]: has been Preempted");
                                    // set the action state to preempted
                                    result_.trace += " - Preemted";
                                    as_.setPreempted(result_);
                                    close_and_return();
                                    return;
                                }
                            }
                        }
                        catch (exception e)
                        {
                            ROS_ERROR("[NavAssistant] Exception while navigating through path: %s\n", e.what());
                            result_.trace += " - Exception";
                            as_.setAborted(result_);
                            close_and_return();
                            return;
                        }
                        catch (...)
                        {
                            ROS_ERROR("[NavAssistant] Unknown Exception while navigating through path");
                            result_.trace += " - Exception";
                            as_.setAborted(result_);
                            close_and_return();
                            return;
                        }
                    }
                    else
                    {
                        if(verbose)
                            ROS_WARN("[NavAssistant]: Unable to get Path. Trying direct navigation.");
                        result_.trace += " - Unable to get Path";
                    }
                } //end if-start!=end
            }
            else if (verbose)
                ROS_WARN("[NavAssistant]: Unable to get Closest ING Node (end) pose=[%.3f, %.3f]. Trying direct navigation.", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        }
        else if (verbose)
            ROS_WARN("[NavAssistant]: Unable to get Closest ING Node (start) pose=[%.3f, %.3f]. Trying direct navigation.", current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y );



        // -------------------------------
        // NAVIGATION TO TARGET GOAL (MB)
        // -------------------------------
        //1. Initial Turn
        if (goal->turn_before_nav)
            turn_towards_path(goal->target_pose);

        ros::Duration(0.2).sleep();
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_WARN("[NavAssistant]: has been Preempted");
            // set the action state to preempted
            result_.trace += " - Preemted";
            as_.setPreempted(result_);
            close_and_return();
            return;
        }

        //2. Navigate to goal
        move_base_nav_and_wait(goal->target_pose);

        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_WARN("[NavAssistant]: has been Preempted");
            // set the action state to preempted
            result_.trace += " - Preemted";
            as_.setPreempted(result_);
            close_and_return();
            return;
        }

        // 3. Check Result
        if (mb_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            result_.trace += " - MoveBase Succeeded";
            as_.setSucceeded(result_);
            close_and_return();
            return;
        }
        else
        {
            result_.trace += " - MoveBase Failed";
            as_.setAborted(result_);
            close_and_return();
            return;
        }
    }

    catch (exception e)
    {
        ROS_ERROR("[NavAssistant] Exception while executting AS: %s\n", e.what());
        result_.trace += " - Exception";
        as_.setAborted(result_);
        close_and_return();
        return;
    }
    catch (...)
    {
        ROS_ERROR("[NavAssistant] Unknown Exception in AS");
        result_.trace += " - Exception";
        as_.setAborted(result_);
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
    geometry_msgs::Twist stop;
    stop.linear.x = stop.linear.y = stop.linear.z = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_publisher.publish(stop);

    if (verbose) ROS_INFO("[NavAssistant] Closing Action...");
    ros::Duration(0.1).sleep();


}

CNavAssistant::~CNavAssistant(){}

//===================================================================================
//================================== MAIN ===========================================
//===================================================================================
int main(int argc, char** argv)
{
    ros::init(argc,argv,"navigation_asssistant_node");

    CNavAssistant nav_assistant("nav_assistant");
    ROS_INFO("[NavAssistant] Action server is ready for action!...");

    ros::spin();

    return 0;
}

