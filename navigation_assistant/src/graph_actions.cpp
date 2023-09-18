#include "nav_assistant.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/algorithm/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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

	// 1. Do we have "poses" (old/new version)
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

	// 2. Load Spaces (aka Rooms)
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

	// 3. Load passages (aka doors)
	try
	{
		for (json::iterator it = json_msg["passages"].begin(); it != json_msg["passages"].end(); ++it)
		{
			auto content = it.value();
			// 0. Chech that it does not connect with OUTSIDE1
			auto spaces_connected = content["connecting"];
			if (spaces_connected.at(0) != "OUTSIDE1" && spaces_connected.at(1) != "OUTSIDE1")
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
		char buffer[50];
		for (json::iterator it = json_msg["objects"].begin(); it != json_msg["objects"].end(); ++it)
		{
			auto content = it.value();
			if (content["label"] == "docking_station" || content["type"] == "dockingstation")
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
	node_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), pose_yaw));

	if (node_type == "CNP" || node_type == "CP" || node_type == "passage")
	{
		if (verbose)
			RCLCPP_INFO(get_logger(), "[NavAssistant] Estimating optimal pose for CNP around [%.3f, %.3f]", pose_x, pose_y);
		// Estimate best location for the CP/CNP/passage around given pose
		auto request = std::make_shared<NAS::NavAssistantSetCNP::Request>();
		request->pose = node_pose;
		auto future = nav_assist_functions_client_CNP->async_send_request(request);
		rclcpp::spin_until_future_complete(shared_from_this(), future);
		auto response = future.get();
		// Update pose
		if (response->success)
		{
			node_pose = response->pose;
			if (verbose)
				RCLCPP_INFO(get_logger(), "[NavAssistant] Setting CNP at [%.3f, %.3f]", node_pose.pose.position.x, node_pose.pose.position.y);
		}
		else if (verbose)
			RCLCPP_INFO(get_logger(), "[NavAssistant] Error setting CNP around [%.3f, %.3f], Using exact point.", pose_x, pose_y);
	}

	// 1. Add a new node in the graph
	auto graphRequest = std::make_shared<topology_graph::srv::Graph::Request>();
	graphRequest->cmd = "AddNode"; // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
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
			for (auto p : response->sp)
			{
				graphRequest->cmd = "AddNode"; // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
				graphRequest->params.clear();
				graphRequest->params.push_back(node_label); // use the same label as the original POI
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
	else if ((node_type == "CNP") || (node_type == "CP"))
	{
		// Estimate and Add the two Segment Points (SP) and the two Intermediate Navigation Goals (ING)
		auto poiRequest = std::make_shared<NAS::NavAssistantPOI::Request>();
		poiRequest->pose = node_pose;

		auto future = nav_assist_functions_client_POI->async_send_request(poiRequest);
		rclcpp::spin_until_future_complete(shared_from_this(), future);
		auto response = future.get();

		if (response->success)
		{
			// 2.1 Add "SP" nodes (Segment Points)
			for (auto p : response->sp)
			{
				graphRequest->cmd = "AddNode"; // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
				graphRequest->params.clear();
				graphRequest->params.push_back(node_label); // use the same label as the original POI
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

			// 2.2 Add a "ING" nodes (intermediate navigation goals)
			std::vector<std::string> ing_id;
			for (auto p : response->ing)
			{
				graphRequest->cmd = "AddNode"; // params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
				graphRequest->params.clear();
				graphRequest->params.push_back(node_label); // use the same label as the original POI
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

				// keep node_id
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

	// 1. Remove all Arcs in the Graph
	{
		if (verbose)
			RCLCPP_INFO(get_logger(), "[NavAssistant] Regenerating Arcs in Graph");
		auto clearRequest = std::make_shared<topology_graph::srv::Graph::Request>();
		clearRequest->cmd = "DeleteAllArcs";  // params: [node_id]
		clearRequest->params.push_back("-1"); // All nodes = -1
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
		auto regenerateRequest = std::make_shared<topology_graph::srv::Graph::Request>();
		// ING nodes
		regenerateRequest->cmd = "GetNodesbyType"; // params: node_type
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
			auto regenerateRequest = std::make_shared<topology_graph::srv::Graph::Request>();
			// CP nodes
			regenerateRequest->cmd = "GetNodesbyType"; // params: node_type
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
			auto regenerateRequest = std::make_shared<topology_graph::srv::Graph::Request>();
			// CNP nodes
			regenerateRequest->cmd = "GetNodesbyType"; // params: node_type
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
		for (auto it2 = it + 1; it2 != nodes_to_connect.end(); ++it2)
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
					// Create Arc beetwing the two INGs
					auto addArcRequest = std::make_shared<topology_graph::srv::Graph::Request>();
					addArcRequest->cmd = "AddArc"; // params: [idfrom, idto, label, type, bidirectional]
					addArcRequest->params.clear();
					addArcRequest->params.push_back(node_data[0]);
					addArcRequest->params.push_back(node_data2[0]);
					addArcRequest->params.push_back("arc");
					addArcRequest->params.push_back("arc");
					addArcRequest->params.push_back("true"); // bidirectional arc

					auto future = graph_srv_client->async_send_request(addArcRequest);
					auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
					auto response = future.get();

					if (!response->success)
						RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create Arc");
				}
				// Create arc if different types
				else if (node_data[2] != node_data2[2])
				{
					// Create Arc
					auto addArcRequest = std::make_shared<topology_graph::srv::Graph::Request>();
					addArcRequest->cmd = "AddArc"; // params: [idfrom, idto, label, type, bidirectional]
					addArcRequest->params.clear();
					addArcRequest->params.push_back(node_data[0]);
					addArcRequest->params.push_back(node_data2[0]);
					addArcRequest->params.push_back("arc");
					addArcRequest->params.push_back("arc");
					addArcRequest->params.push_back("true"); // bidirectional arc

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
				if (node_data[2] == "ING" && node_data2[2] == "ING")
				{
					// Create arc if different label and type ING
					// Check if there is a path not crossing a segment defined by a CNP or CP
					auto distanceRequest = std::make_shared<topology_graph::srv::Graph::Request>();
					distanceRequest->cmd = "GetNavDistTwoPoses"; // params: p1(x,y,yaw), p2(x,y,yaw), [avoid_node_types]
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
						RCLCPP_WARN(get_logger(), "[NavAssistant]: NavDistance between Node[%s] <-> [%s] is %s (succes=%d)", node_data[0].c_str(), node_data2[0].c_str(), response->result[0].c_str(), response->success);

					if (response->success)
					{
						// system("read -p 'Press Enter to continue...' var");
						if (verbose)
							RCLCPP_WARN(get_logger(), "[NavAssistant]: Adding new Arc between ING Nodes [%s] <-> [%s]", node_data[0].c_str(), node_data2[0].c_str());
						// Create Arc
						auto addArcRequest = std::make_shared<topology_graph::srv::Graph::Request>();
						addArcRequest->cmd = "AddArc"; // params: [idfrom, idto, label, type, bidirectional]
						addArcRequest->params.clear();
						addArcRequest->params.push_back(node_data[0]);
						addArcRequest->params.push_back(node_data2[0]);
						addArcRequest->params.push_back("arc");
						addArcRequest->params.push_back("arc");
						addArcRequest->params.push_back("true"); // bidirectional arc
						auto future = graph_srv_client->async_send_request(distanceRequest);
						auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
						auto response = future.get();

						if (!response->success)
							RCLCPP_WARN(get_logger(), "[NavAssistant]: Unable to create Arc");
					}
				}
			}
		} // end.for each other ING
	}     // end-for each ING
}

bool CNavAssistant::deleteNode(std::string node_type, double pose_x, double pose_y, double pose_yaw)
{
	if (verbose)
		RCLCPP_INFO(get_logger(), "[NavAssistant] Deleting Node of type [%s], close to [%.3f, %.3f]", node_type.c_str(), pose_x, pose_y);
	// 1. Get all Nodes of type: node_type
	auto getNodesRequest = std::make_shared<topology_graph::srv::Graph::Request>();
	getNodesRequest->cmd = "GetNodesbyType"; // params: [node_type]
	getNodesRequest->params.push_back(node_type);

	auto future = graph_srv_client->async_send_request(getNodesRequest);
	auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
	auto response = future.get();

	if (response->success)
	{
		double min_dist;
		std::string node_label_to_delete = "";

		// 1. Get closest node (euclidean dist)
		for (std::string n : response->result)
		{
			// Node data = "id label type x y yaw"
			std::vector<std::string> node_data;
			node_data.clear();
			boost::split(node_data, n, boost::is_any_of(" "));

			// get Euclidean distance between clicked point and nodes (keep closest one)
			double d = std::sqrt(std::pow(pose_x - std::atof(node_data[3].c_str()), 2) + std::pow(pose_y - std::atof(node_data[4].c_str()), 2));
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

		// 2. Delete all the nodes matching this label (nodes of all types!)
		if (verbose)
			RCLCPP_INFO(get_logger(), "[NavAssistant] Deleting Nodes with label %s", node_label_to_delete.c_str());
		auto matchNodesRequest = std::make_shared<topology_graph::srv::Graph::Request>();
		matchNodesRequest->cmd = "GetNodesbyLabel"; // params: [node_label]
		matchNodesRequest->params.push_back(node_label_to_delete);

		auto future = graph_srv_client->async_send_request(matchNodesRequest);
		auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
		auto matchResponse = future.get();

		if (matchResponse->success)
		{
			auto deleteRequest = std::make_shared<topology_graph::srv::Graph::Request>();
			// for each node with the matching label (delete it)
			for (std::string n : matchResponse->result)
			{
				// Node data = "id label type x y yaw"
				std::vector<std::string> node_data;
				node_data.clear();
				boost::split(node_data, n, boost::is_any_of(" "));

				deleteRequest->cmd = "DeleteNodeById"; // params: node_id
				deleteRequest->params.clear();
				deleteRequest->params.push_back(node_data[0]);
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
bool CNavAssistant::addPointCB(NAS::NavAssistantPoint::Request::SharedPtr req, NAS::NavAssistantPoint::Response::SharedPtr res)
{
	pointRequestQueue.push_back(req);
	return true;
}


void CNavAssistant::HandleGraphRequests()
{
	while (!pointRequestQueue.empty())
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
			counter++;
			if (verbose)
				RCLCPP_INFO(get_logger(), "[NavAssistant] Request to Add a Node,");

			// Execute action and return result
			if (addNode(label, req->type, req->pose.pose.position.x, req->pose.pose.position.y, yaw_angle))
			{
				if (req->type == "CP" || req->type == "CNP" || req->type == "passage")
					regenerate_arcs();
			}
		}
		else if (req->action == "delete")
		{
			if (verbose)
				RCLCPP_INFO(get_logger(), "[NavAssistant] Request to Delete a Node,");
			if (deleteNode(req->type, req->pose.pose.position.x, req->pose.pose.position.y, yaw_angle))
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
				// File contains a boost::graphviz description of the nodes and arcs
				auto saveRequest = std::make_shared<topology_graph::srv::Graph::Request>();
				saveRequest->cmd = "SaveGraph"; // params: [file_path]
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
