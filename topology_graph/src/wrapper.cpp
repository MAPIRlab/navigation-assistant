#include "topology_graph/wrapper.h"
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>

using namespace std;
using json = nlohmann::json;

//-----------------------------------------------------------------------------------
//                              GRAPH ACTIONS
//-----------------------------------------------------------------------------------

// Define here an ENUM type with all the actions
enum Action
{
	AddNode,
	AddArc,
	AddArcbyLabel,

	FindPath,
	GetNavDistTwoPoses,

	GetNodesbyLabel,
	GetNodesbyType,
	GetNodesbyLabelandType,
	GetNodebyId,
	GetAllNodes,
	GetAllArcs,
	GetClosestNode,

	DeleteNodeByLabel,
	DeleteNodeById,
	DeleteAllArcs,

	LoadGraph,
	SaveGraph,

	PrintGraph,
	GetNodeNeighbors,
	GetNodeByLocation,
	GetNodeLocation,
	SetNodeLocation,
	SetNodeLabel,
	DeleteOutcommingArcs,
	DeleteIncommingArcs,

	DeleteArcsBetweenNodes,
	ExistsNodeLabel,

	UnknownAction
};

// This function translates from Strings to Enum (Actions)
Action string_to_enum(std::string action_str)
{
	if (action_str == "AddNode")
		return AddNode;
	else if (action_str == "AddArc")
		return AddArc;
	else if (action_str == "AddArcbyLabel")
		return AddArcbyLabel;

	else if (action_str == "FindPath")
		return FindPath;
	else if (action_str == "GetNavDistTwoPoses")
		return GetNavDistTwoPoses;

	else if (action_str == "GetNodesbyLabel")
		return GetNodesbyLabel;
	else if (action_str == "GetNodesbyType")
		return GetNodesbyType;
	else if (action_str == "GetNodesbyLabelandType")
		return GetNodesbyLabelandType;
	else if (action_str == "GetNodebyId")
		return GetNodebyId;

	else if (action_str == "GetAllNodes")
		return GetAllNodes;
	else if (action_str == "GetAllArcs")
		return GetAllArcs;
	else if (action_str == "GetClosestNode")
		return GetClosestNode;

	else if (action_str == "LoadGraph")
		return LoadGraph;
	else if (action_str == "SaveGraph")
		return SaveGraph;

	else if (action_str == "DeleteNodeByLabel")
		return DeleteNodeByLabel;
	else if (action_str == "DeleteNodeById")
		return DeleteNodeById;
	else if (action_str == "DeleteAllArcs")
		return DeleteAllArcs;

	else if (action_str == "SetNodeLabel")
		return SetNodeLabel;
	else if (action_str == "GetNodeNeighbors")
		return GetNodeNeighbors;
	else if (action_str == "GetNodeByLocation")
		return GetNodeByLocation;
	else if (action_str == "DeleteOutcommingArcs")
		return DeleteOutcommingArcs;
	else if (action_str == "DeleteIncommingArcs")
		return DeleteIncommingArcs;

	else if (action_str == "DeleteArcsBetweenNodes")
		return DeleteArcsBetweenNodes;
	else if (action_str == "SetNodeLocation")
		return SetNodeLocation;
	else if (action_str == "GetNodeLocation")
		return GetNodeLocation;
	else if (action_str == "ExistsNodeLabel")
		return ExistsNodeLabel;
	else if (action_str == "PrintGraph")
		return PrintGraph;
	else
	{
		spdlog::error("[topology_graph] Requested action not implemented.");
		return UnknownAction;
	}
}

// ------------------
// CGraphWrapper
//-------------------
CGraphWrapper::CGraphWrapper() : Node("Topology_graph")
{
	// Read Parameters
	//----------------
	verbose = declare_parameter<bool>("verbose", false);

	// Marker publisher
	marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("topology_graph", 1);
	path_pub = create_publisher<nav_msgs::msg::Path>("topology_graph_paths", 1);

	// Make plan service client
	std::string get_plan_server = declare_parameter<std::string>("get_plan_server", "compute_path_to_pose");
	getPlanClient = rclcpp_action::create_client<GetPlan>(this, get_plan_server);

	using namespace std::chrono_literals;
	while (rclcpp::ok() && !getPlanClient->wait_for_action_server(5.0s))
		spdlog::warn("[topology_graph] Waiting for move_base MAKE_PLAN srv to come online.");

	// Ropot pose subscriber
	using namespace std::placeholders;
	// Advertise service
	service = create_service<topology_graph::srv::Graph>("topology_graph/graph", std::bind(&CGraphWrapper::srvCB, this, _1, _2));

	// Debug
	free_paths.clear();
	num_path = 0;
	spdlog::info("[topology_graph] Ready for Operation");
}

CGraphWrapper::~CGraphWrapper()
{
}

// ------------------
// Graph srv handler
// ------------------
bool CGraphWrapper::srvCB(topology_graph::srv::Graph::Request::SharedPtr req, topology_graph::srv::Graph::Response::SharedPtr res)
{
	Action req_action = string_to_enum(req->cmd);
	if (req_action == UnknownAction)
	{
		res->result.clear();
		res->success = false;
		return true;
	}

	// Execute the requested graph action
	switch (req_action)
	{
	case AddNode:
		// params: [node_label, node_type, pos_x, pos_y, [pose_yaw]]
		// result: Success/Failure. On success the new node_ID
		if (req->params.size() == 4 || req->params.size() == 5)
		{
			double pos_x, pos_y, pos_yaw;
			pos_x = atof(req->params[2].c_str());
			pos_y = atof(req->params[3].c_str());
			if (req->params.size() == 5)
				pos_yaw = atof(req->params[4].c_str());
			else
				pos_yaw = 0.0;

			// graph call
			size_t node_id;
			if (my_graph.AddNode(req->params[0], req->params[1], pos_x, pos_y, pos_yaw, node_id))
			{
				// return the node ID
				res->success = true;
				res->result.push_back(std::to_string(node_id));
				if (verbose)
					spdlog::info("[topology_graph-AddNode] New node added with id=%lu", node_id);
			}
			else
			{
				res->success = false;
				if (verbose)
					spdlog::warn("[topology_graph-AddNode] Error adding new node.");
			}
		}
		else if (verbose)
			spdlog::warn("[topology_graph-AddNode] Incorrect Number of parameters (4 required)");

		break;

	case AddArc:
		// params: [idfrom, idto, label, type, bidirectional]
		// result: Success/Failure. On success the new arc_ID
		if (req->params.size() == 5)
		{
			size_t id_from, id_to, arc_id;
			id_from = atoi(req->params[0].c_str());
			id_to = atoi(req->params[1].c_str());
			bool bidi = (req->params[4] == "yes" || req->params[4] == "Yes" || req->params[4] == "true" || req->params[4] == "True" || req->params[4] == "1");

			// graph call
			if (my_graph.AddArc(id_from, id_to, req->params[2], req->params[3], bidi, arc_id))
			{
				// return Arc ID
				res->success = true;
				res->result.push_back(std::to_string(arc_id));
				if (verbose)
					spdlog::info("[topology_graph-AddArc] New Arc added with id=%lu", arc_id);
			}
			else
			{
				res->success = false;
				if (verbose)
					spdlog::warn("[topology_graph-AddArc] Error adding new arc.");
			}
		}
		else if (verbose)
			spdlog::warn("[topology_graph-AddArc] Error: Incorrect Number of parameters (5 required)");

		break;

	case AddArcbyLabel:
		// params: [Label_from, Label_to, arc_label, arc_type, bidirectional]
		// result: Success/Failure. On success the new arc_ID
		if (req->params.size() == 5)
		{
			size_t arc_id;
			bool bidi = (req->params[4] == "yes" || req->params[4] == "Yes" || req->params[4] == "true" || req->params[4] == "True" || req->params[4] == "1");

			// graph call
			if (my_graph.AddArcbyLabel(req->params[0], req->params[1], req->params[2], req->params[3], bidi, arc_id))
			{
				// return Arc ID
				res->success = true;
				res->result.push_back(std::to_string(arc_id));
				if (verbose)
					spdlog::info("[topology_graph-AddArcbyLabel] New Arc added with id=%lu", arc_id);
			}
			else
			{
				res->success = false;
				if (verbose)
					spdlog::warn("[topology_graph-AddArcbyLabel] Error adding new arc.");
			}
		}
		else
		{
			if (verbose)
				spdlog::warn("[topology_graph-AddArcbyLabel] Error: Incorrect Number of parameters (5 required)");
		}

		break;

	case FindPath:
		// params: [nodeID_start, nodeID_end]
		// result: Success/Failure. On success the list of Nodes ["id1 label1 type1 x1 y1 yaw", ..., ""idN labelN typeN xN yN yawN"]
		try
		{
			if (req->params.size() == 2)
			{
				size_t id_start, id_end;
				id_start = atoi(req->params[0].c_str());
				id_end = atoi(req->params[1].c_str());

				// get graph path
				std::vector<std::string> path;
				if (my_graph.FindPath(id_start, id_end, path))
				{
					res->success = true;
					res->result = path;
					if (verbose)
						spdlog::info("[topology_graph-FindPathInfo] Find path [{}]-->[{}] done. ", req->params[0].c_str(), req->params[1].c_str());
				}
				else
				{
					res->success = false;
					if (verbose)
						spdlog::warn("[topology_graph-FindPathInfo] Error Finding path [%s]-->[%s] ", req->params[0].c_str(), req->params[1].c_str());
				}
			}
			else
				spdlog::warn("[topology_graph-FindPathInfo]Error: Incorrect Number of parameters (2 required)");
		}
		catch (exception e)
		{
			spdlog::error("[topology_graph-FindPathInfo] Exception: %s\n", e.what());
			res->result.clear();
			res->success = false;
			return true;
		}
		catch (...)
		{
			spdlog::error("[topology_graph-FindPathInfo] Unknown Exception");
			res->result.clear();
			res->success = false;
			return true;
		}
		break;

	case GetNavDistTwoPoses:
		// params: [p1(x,y,yaw), p2(x,y,yaw), [avoid_node_types] ]
		// result: Success/Failure. On success the Navigation Distance (m)
		try
		{
			if (req->params.size() >= 6)
			{
				geometry_msgs::msg::Pose p1;
				p1.position.x = std::atof(req->params[0].c_str());
				p1.position.y = std::atof(req->params[1].c_str());
				p1.position.z = 0.0;
				p1.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), std::atof(req->params[2].c_str())));

				geometry_msgs::msg::Pose p2;
				p2.position.x = std::atof(req->params[3].c_str());
				p2.position.y = std::atof(req->params[4].c_str());
				p2.position.z = 0.0;
				p2.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), std::atof(req->params[5].c_str())));

				double d;

				// avoid_node_types?
				if (req->params.size() > 6)
				{
					vector<std::string>::const_iterator first = req->params.begin() + 6;
					vector<std::string>::const_iterator last = req->params.end();
					vector<std::string> avoiding_node_types(first, last);
					// Get Nav Distance
					d = get_nav_distance_two_poses(p1, p2, avoiding_node_types);
				}
				else
					// Get Nav Distance
					d = get_nav_distance_two_poses(p1, p2, {});

				// Set result
				res->result.push_back(std::to_string(d));
				if (d < 0.0)
					res->success = false;
				else
					res->success = true;
			}
			else
				spdlog::warn("[topology_graph-GetNavDistTwoPoses]Error: Incorrect Number of parameters (+6 required)");
		}
		catch (exception e)
		{
			spdlog::error("[topology_graph-GetNavDistTwoPoses] Exception: %s\n", e.what());
			res->result.clear();
			res->success = false;
			return true;
		}
		catch (...)
		{
			spdlog::error("[topology_graph-GetNavDistTwoPoses] Unknown Exception");
			res->result.clear();
			res->success = false;
			return true;
		}
		break;

	case LoadGraph:
		// params: [file_path]
		if (req->params.size() == 1)
		{
			my_graph.LoadGraph(req->params[0]);
			res->success = true;
			if (verbose)
				spdlog::info("[topology_graph-LoadGraph] Load graph from file done.");
		}
		else
		{
			if (verbose)
				spdlog::info("[topology_graph-LoadGraph] Error loading graph from file.");
			res->success = false;
		}
		break;

	case SaveGraph:
		// params: [file_path]
		if (req->params.size() == 1)
		{
			my_graph.SaveGraph(req->params[0]);
			if (verbose)
				spdlog::info("[topology_graph-SaveGraph] Saving graph to file done");
			res->success = true;
		}
		else
		{
			if (verbose)
				spdlog::info("[topology_graph-SAveGraph] Error saving graph to file. Incorrect Number of parameters (1 required)");
			res->success = false;
		}
		break;

	case GetNodesbyLabel:
		// params: [node_label]
		if (req->params.size() == 1)
		{
			std::vector<string> nodeList;

			// get nodes with specifit type
			my_graph.GetNodesbyLabel(req->params[0], nodeList);

			// Srv response
			res->success = true;
			res->result = nodeList;
			if (verbose)
				spdlog::info("[topology_graph-GetNodesbyLabel] Done");
		}
		else if (verbose)
			spdlog::info("[topology_graph-GetNodesbyLabel] Error: Incorrect Number of parameters (1 required)");

		break;

	case GetNodesbyType:
		// params: [node_type]
		if (req->params.size() == 1)
		{
			std::vector<string> nodeList;

			// get nodes with specifit type
			my_graph.GetNodesbyType(req->params[0], nodeList);

			// Srv response
			res->success = true;
			res->result = nodeList;
			if (verbose)
				spdlog::info("[topology_graph-GetNodesbyType] Done");
		}
		else if (verbose)
			spdlog::info("[topology_graph-GetNodesbyType] Error: Incorrect Number of parameters (1 required)");

		break;

	case GetNodesbyLabelandType:
		// params: [node_type]
		if (req->params.size() == 2)
		{
			std::vector<string> nodeList;

			// get nodes with specifit type
			my_graph.GetNodesbyLabelandType(req->params[0], req->params[1], nodeList);

			// Srv response
			res->success = true;
			res->result = nodeList;
			if (verbose)
				spdlog::info("[topology_graph-GetNodesbyLabelandType] Done");
		}
		else if (verbose)
			spdlog::info("[topology_graph-GetNodesbyLabelandType] Error: Incorrect Number of parameters (2 required)");

		break;

	case GetNodebyId:
		// params: [node_id]
		if (req->params.size() == 1)
		{
			string node_data;
			int node_id = atoi(req->params[0].c_str());

			// get node
			node_data = my_graph.GetNodebyId(node_id);

			if (node_data != "")
			{
				res->success = true;
				res->result.push_back(node_data);
				if (verbose)
					spdlog::info("[topology_graph-GetNodeLabel] Done");
			}
			else
			{
				res->success = false;
				if (verbose)
					spdlog::info("[topology_graph-GetNodeLabel] Error: Node ID not found");
			}
		}
		else
			spdlog::info("[topology_graph-GetNodeLabel] Error: Incorrect Number of parameters (1 required)");

		break;

	case GetAllNodes:
	{
		std::vector<string> nodeList;

		// Get node list
		my_graph.GetAllNodes(nodeList);

		// Srv response
		res->success = true;
		res->result = nodeList;
		if (verbose)
			spdlog::info("[topology_graph-GetAllNodes] Done");
	}
	break;

	case GetAllArcs:
	{
		std::vector<string> arcList;

		// Get list of arcs
		my_graph.GetAllArcs(arcList);

		// Prepare output
		res->success = true;
		res->result = arcList;
		spdlog::info("[topology_graph-GetAllArcs] Done");
	}
	break;

	case GetClosestNode:
		// params: [pose_x, pose_y, pose_yaw, [node_type], [node_label]]
		// result: [id label type x y yaw]
		try
		{
			if (req->params.size() < 3)
			{
				if (verbose)
					spdlog::warn("[topology_graph-GetClosestNode] Error: Incorrect Number of parameters (3 required - 5 alowed)");
				// Srv response
				res->success = false;
				res->result.clear();
				return true;
			}
			else
			{
				geometry_msgs::msg::Pose p; // Origin Pose
				p.position.x = atof(req->params[0].c_str());
				p.position.y = atof(req->params[1].c_str());
				p.position.z = 0.0;
				p.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), std::atof(req->params[2].c_str())));

				// get closest node (using navigation distance)
				// [optional] without passing through passages
				std::vector<std::string> node_list;

				if (req->params.size() == 3) // No extra filters
					my_graph.GetAllNodes(node_list);
				else if (req->params.size() == 4) // Type filter
					my_graph.GetNodesbyType(req->params[3], node_list);
				else // Label & Type filter
					my_graph.GetNodesbyLabelandType(req->params[4], req->params[3], node_list);

				std::vector<string> closest_node;
				double min_dist;
				bool first_candidate = true;
				for (auto n : node_list)
				{
					// Node data = "id label type x y yaw"
					std::vector<string> node_data;
					boost::split(node_data, n, boost::is_any_of(" "));

					// Check if node is valid
					if (node_data.size() != 6)
					{
						spdlog::warn("[topology_graph-GetClosestNode] Error incorrect node data, skipping it.");
						for (auto i : node_data)
							std::cout << i << " ";
						continue;
					}

					// 1. Get path (make_plan) from p to Node, and estimate distance
					geometry_msgs::msg::Pose gp; // Goal Pose
					gp.position.x = atof(node_data[3].c_str());
					gp.position.y = atof(node_data[4].c_str());
					gp.position.z = 0.0;
					gp.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), std::atof(req->params[5].c_str())));

					// get distance (m) -> result is (-1) if not valid path or intersects with a Passage or CP
					double nav_distance;
					if (req->params.size() >= 4)
					{
						if (req->params[3] == "ING")
							nav_distance = get_nav_distance_two_poses(p, gp, { "CNP", "CP" });
						else
							nav_distance = get_nav_distance_two_poses(p, gp, { "passage", "CP" });
					}
					else
						nav_distance = get_nav_distance_two_poses(p, gp, { "passage", "CP" });

					if (verbose)
						spdlog::info("[topology_graph-GetClosestNode] Nav Distance between pose [%.2f, %.2f] and Node(%s)=[%.2f, %.2f] is %.3f[m]",
							p.position.x, p.position.y, node_data[0].c_str(), gp.position.x, gp.position.y, nav_distance);

					if (nav_distance < 0.0)
						continue;

					// Keep Node with min_dist
					if (first_candidate)
					{
						first_candidate = false;
						min_dist = nav_distance;
						closest_node = node_data;
					}
					else if (nav_distance < min_dist)
					{
						min_dist = nav_distance;
						closest_node.clear();
						closest_node = node_data;
					}
				} // end-for

				// Prepare output
				if (first_candidate)
				{
					// No closest node found (Error)
					if (verbose)
						spdlog::info("[topology_graph-GetClosestNode] No closet node found. Error!\n\n");
					res->success = false;
					res->result.clear();
				}
				else
				{
					if (verbose)
						spdlog::info("[topology_graph-GetClosestNode] Closet node is [%s]=%s.\n\n", closest_node[0].c_str(), closest_node[1].c_str());
					res->success = true;
					res->result = closest_node;
				}
			}
		}
		catch (exception e)
		{
			spdlog::error("[topology_graph-GetClosestNode] Exception: %s\n", e.what());
			res->result.clear();
			res->success = false;
			return true;
		}
		catch (...)
		{
			spdlog::error("[topology_graph-GetClosestNode] Unknown Exception");
			res->result.clear();
			res->success = false;
			return true;
		}
		break;

	case DeleteNodeByLabel:
		// params: [node_label]
		// result: [none]
		if (req->params.size() == 1)
		{
			if (my_graph.DeleteNode(req->params[0]))
			{
				res->success = true;
				spdlog::info("[topology_graph-DeleteNode] Node has been deleted.");
			}
			else
			{
				res->success = false;
				spdlog::info("[topology_graph-DeleteNode] Node not found. Unable to delete node.");
			}
		}
		else
		{
			res->success = false;
			spdlog::info("[topology_graph-DeleteNode] Error: Incorrect Number of parameters (1 required)");
		}
		break;

	case DeleteNodeById:
		// params: [node_id]
		// result: [none]
		if (req->params.size() == 1)
		{
			if (my_graph.DeleteNode(std::atoi(req->params[0].c_str())))
			{
				res->success = true;
				spdlog::info("[topology_graph-DeleteNode] Node has been deleted.");
			}
			else
			{
				res->success = false;
				spdlog::info("[topology_graph-DeleteNode] Node not found. Unable to delete node.");
			}
		}
		else
		{
			res->success = false;
			spdlog::info("[topology_graph-DeleteNode] Error: Incorrect Number of parameters (1 required)");
		}
		break;

	case DeleteAllArcs:
		// params: [ node_id]
		// result: [none]
		if (req->params.size() == 1)
		{
			int node_id = std::atoi(req->params[0].c_str());
			if (node_id == -1) // Delete Arcs from ALL nodes
			{
				// 1. Get all nodes
				std::vector<std::string> node_list;
				my_graph.GetAllNodes(node_list);

				// 2. Delete Arcs
				for (auto n : node_list)
				{
					// Node data = "id label type x y yaw"
					std::vector<string> node_data;
					boost::split(node_data, n, boost::is_any_of(" "));
					my_graph.DeleteAllArcs(std::atoi(node_data[0].c_str()));
				}
			}
			else
				my_graph.DeleteAllArcs(node_id);

			res->success = true;
			spdlog::info("[topology_graph-DeleteAllArcs] Arcs have been deleted.");
		}
		else
		{
			res->success = false;
			spdlog::info("[topology_graph-DeleteAllArcs] Error: Incorrect Number of parameters (1 required)");
		}
		break;

	default:
	{
		spdlog::error("[topology_graph] cmd not implemented!");
		return false;
	}
	} // end swith

	// Serive always return TRUE.
	// The result is withing the srv.response parameter
	return true;
}

/*

		case SetNodeLabel: // params: [id,label]

			if (req->params.size()==2)

			{

				int valor_id;

				valor_id=atoi(req->params[0].c_str());

				my_graph.SetNodeLabel(valor_id,req->params[1]);

				spdlog::info("[topology_graph - SetNodeLabel]La accion SetNodeLabel se ha realizado con exito");

			}

			else

			{

				spdlog::info("[topology_graph - SetNodeLabel]Error: Incorrect Number of parameters (2 required)");

			}

			break;


		case GetNodeNeighbors: // params: [idnode,arc_type]

			if (req->params.size()==2)

			{

				vector<size_t> mi_vector;

				int valor_idnode,f;

				valor_idnode=atoi(req->params[0].c_str());

				my_graph.GetNodeNeighbors(valor_idnode,req->params[1],mi_vector);



					for (f=0;f<mi_vector.size();f++)
					{

						res->resp.push_back(std::to_string(mi_vector[f]));

					}



				spdlog::info("[topology_graph - GetNodeNeighbors]La accion GetNodeNeighbors se ha realizado con exito");
			}
			else

			{

				spdlog::info("[topology_graph - GetNodeNeighbors]Error: Incorrect Number of parameters (2 required)");

			}

		   break;





		case GetNodeByLocation: // params: [x,y,tolerance]

			if (req->params.size()==3)

			{

				double x,y,tolerance;

				size_t id_node;

				x = atof(req->params[0].c_str());

				y = atof(req->params[1].c_str());

				tolerance = atof(req->params[2].c_str());

				my_graph.GetNodeByLocation(x,y,tolerance,id_node);

				res->resp.push_back( std::to_string(id_node) );

				spdlog::info("[topology_graph - GetNodeByLocation]La accion GetNodeByLocation se ha realizado con exito");


			}

			else

			{

				spdlog::info("[topology_graph - GetNodeByLocation]Error: Incorrect Number of parameters (3 required)");

			}

		break;


		case DeleteOutcommingArcs: // params: [idnode,arc_type]

			if (req->params.size()==2)

			{

				long valor_idnode;

				valor_idnode=atol(req->params[0].c_str());

				my_graph.DeleteOutcommingArcs(valor_idnode,req->params[1]);

				spdlog::info("[topology_graph - DeleteOutcommingArcs]La accion DeleteOutcommingArcs se ha realizado con exito");

			}

			else

			{

				spdlog::info("[topology_graph - DeleteOutcommingArcs]Error: Incorrect Number of parameters (2 required)");

			}

			break;


		case DeleteIncommingArcs: // params: [idnode,arc_type]

			if (req->params.size()==2)

			{

				long valor_idnode;

				valor_idnode=atol(req->params[0].c_str());

				my_graph.DeleteIncommingArcs(valor_idnode,req->params[1]);

				spdlog::info("[topology_graph - DeleteIncommingArcs]La accion DeleteIncommingArcs se ha realizado con exito");


			}

			else

			{

				spdlog::info("[topology_graph - DeleteIncommingArcs]Error: Incorrect Number of parameters (2 required)");

			}

			break;


		case DeleteAllArcs: // params: [idnode,arc_type /arc_type]

			if (req->params.size()==2)

			{

				long valor_idnode;

				valor_idnode=atol(req->params[0].c_str());

				my_graph.DeleteAllArcs(valor_idnode,req->params[1]);

				spdlog::info("[topology_graph - DeleteAllArcs]La accion DeleteAllArcs se ha realizado con exito");


			}


			else if (req->params.size()==1)

			{

				long valor_idnode;

				valor_idnode=atol(req->params[0].c_str());

				my_graph.DeleteAllArcs(valor_idnode);

				spdlog::info("[topology_graph - DeleteAllArcs]La accion DeleteAllArcs se ha realizado con exito");


			}

			else

			{

				spdlog::info("[topology_graph - DeleteAllArcs]Error: Incorrect Number of parameters (1 or 2 required)");

			}

			break;


		case DeleteArcsBetweenNodes: // params: [idfrom,idto,type]

			if (req->params.size()==3)

			{

				long valor_idfrom,valor_idto;

				valor_idfrom=atol(req->params[0].c_str());

				valor_idto=atol(req->params[1].c_str());

				my_graph.DeleteArcsBetweenNodes(valor_idfrom,valor_idto,req->params[2]);

				spdlog::info("[topology_graph - DeleteArcsBetweenNodes]La accion DeleteArcsBetweenNodes se ha realizado con exito");


				   if(draw==1)
				   {

					  string label_from,label_to;

					  double x_from,x_to,y_from,y_to;

					  my_graph.GetNodeLabel(valor_idfrom,label_from);

					  my_graph.GetNodeLabel(valor_idto,label_to);

					  my_graph.GetNodeLocation(label_from,x_from,y_from);

					  my_graph.GetNodeLocation(label_to,x_to,y_to);

					  EraseArcByLocation(x_from,y_from,x_to,y_to);

				   }


			}

			else

			{

				spdlog::info("[topology_graph - DeleteArcsBetweenNodes]Error: Incorrect Number of parameters (3 required)");

			}

			break;











		case SetNodeLocation: // params: [node_label,x,y]

			if (req->params.size()==3)

			{

				double x,y;

				x=atof(req->params[1].c_str());

				y=atof(req->params[2].c_str());

				my_graph.SetNodeLocation(req->params[0],x,y);

				spdlog::info("[topology_graph - SetNodeLocation]La accion SetNodeLocation se ha realizado con exito");


			}

			else

			{

				spdlog::info("[topology_graph - SetNodeLocation]Error: Incorrect Number of parameters (3 required)");

			}

			break;


		case GetNodeLocation: // params: [node_label]

			if (req->params.size()==1)

			{

				double x,y;

				my_graph.GetNodeLocation(req->params[0],x,y);

				res->resp.push_back(std::to_string(x));

				res->resp.push_back(std::to_string(y));

				spdlog::info("[topology_graph - GetNodeLocation]La accion GetNodeLocation se ha realizado con exito");


			}

			else

			{

				spdlog::info("[topology_graph - GetNodeLocation]Error: Incorrect Number of parameters (1 required)");

			}

			break;


		case ExistsNodeLabel: // params: [label]

			if (req->params.size()==1)

			{

				my_graph.ExistsNodeLabel(req->params[0]);

				spdlog::info("[topology_graph - ExistsNodeLabel]La accion ExistsNodeLabel se ha realizado con exito");

			}

			else

			{

				spdlog::info("[topology_graph - ExistsNodeLabel]Error: Incorrect Number of parameters (1 required)");

			}

			break;



		case AddArcRvizTool:

			if(req->params.size()==4)
			{

			   double x_init,y_init,x_fin,y_fin,tolerance=0.5,x_node_init,y_node_init,x_node_fin,y_node_fin;

			   size_t id_init,id_fin,id_arc;

			   string label_init,label_fin;

			   x_init=atof(req->params[0].c_str());
			   y_init=atof(req->params[1].c_str());

			   my_graph.GetNodeByLocation(x_init,y_init,tolerance,id_init);

			   my_graph.GetNodeLabel(id_init,label_init);

			   my_graph.GetNodeLocation(label_init,x_node_init,y_node_init); // En este punto,ya se han obtenido todos los datos del nodo origen necesarios.

			   x_fin=atof(req->params[2].c_str());
			   y_fin=atof(req->params[3].c_str());

			   my_graph.GetNodeByLocation(x_fin,y_fin,tolerance,id_fin);

			   my_graph.GetNodeLabel(id_fin,label_fin);

			   my_graph.GetNodeLocation(label_fin,x_node_fin,y_node_fin); // En este punto,ya se han obtenido todos los datos del nodo destino necesarios.

			   my_graph.AddArc(id_init,id_fin,"arc_label","arc_type",id_arc);

			   DrawArc(x_node_init,y_node_init,x_node_fin,y_node_fin);

			   spdlog::info("[topology_graph - AddArcRvizTool]La accion AddArcRvizTool se ha realizado con exito");

			}

		break;


		case GetGraphRvizTool:

			if(req->params.size()==0)
			{

				string list_nodes;

				my_graph.GetAllNodes(list_nodes);

				res->resp.push_back(list_nodes);


				size_t node_id;

				string label_node;

				double x,y;

				vector <string> tokens_n;
				vector <string> tokens_nodes;

				boost::split(tokens_n,list_nodes,boost::is_any_of("#"));


						 for(int j=0;j<tokens_n.size();j++)
						 {

							 boost::split(tokens_nodes,tokens_n[j],boost::is_any_of(" "));

									if(tokens_nodes.size()==3)
									{

										label_node=tokens_nodes[0];

										x=atof(tokens_nodes[1].c_str());
										y=atof(tokens_nodes[2].c_str());

										node_id=my_graph.GetNodeId(label_node);

										DrawNode(label_node,node_id,x,y);

										sleep(1);

									}

						 }



				string list_arcs;

				my_graph.GetAllArcs(list_arcs);

				res->resp.push_back(list_arcs);



				int id_from,id_to;
				double x_from,y_from,x_to,y_to;
				string label_from,label_to,arc_type;

				vector <string> tokens_a;
				vector <string> tokens_arcs;

				boost::split(tokens_a,list_arcs,boost::is_any_of("#"));

						for(int j=0;j<tokens_a.size();j++)
						{

							boost::split(tokens_arcs,tokens_a[j],boost::is_any_of(" "));

								if(tokens_arcs.size()>=3)
								{

									id_from=atoi(tokens_arcs[0].c_str());
									id_to=atoi(tokens_arcs[1].c_str());
									arc_type=tokens_arcs[2];
									my_graph.GetNodeLabel(id_from,label_from);
									my_graph.GetNodeLabel(id_to,label_to);
									my_graph.GetNodeLocation(label_from,x_from,y_from);
									my_graph.GetNodeLocation(label_to,x_to,y_to);

									DrawArc(x_from,y_from,x_to,y_to);

									sleep(1);
								}

						}

			}

		break;


		case DeleteNodeRvizTool:

			if (req->params.size()==2)
			{


				double x,y,tolerance=0.5;

				size_t id_node;

				x=atof(req->params[0].c_str());

				y=atof(req->params[1].c_str());

				my_graph.GetNodeByLocation(x,y,tolerance,id_node);

				my_graph.DeleteNode(id_node);

				//EraseNode(id_node);

				EraseAll();

				lines.points.clear();


				spdlog::info("[topology_graph - DeleteNodeRvizTool]La accion DeleteNodeRvizTool se ha realizado con exito");

				}

		break;


		case PrintGraph:

		   if(req->params.size()==0)
		   {

			  my_graph.PrintGraph();

			  spdlog::info("[topology_graph - PrintGraph]La accion PrintGraph se ha realizado con exito");

		   }

		break;

*/

// Returns 1 if the segments intersect, 0 otherwise. In addition, if the lines
// intersect the intersection point may be stored in the floats i_x and i_y.
bool CGraphWrapper::get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
	float p2_x, float p2_y, float p3_x, float p3_y, float* i_x, float* i_y)
{
	float s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;
	s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;
	s2_y = p3_y - p2_y;

	try
	{
		float s, t;
		s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
		t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			if (i_x != NULL)
				*i_x = p0_x + (t * s1_x);
			if (i_y != NULL)
				*i_y = p0_y + (t * s1_y);

			// if (verbose) spdlog::info("[topology_graph] get_line_intersection = TRUE");
			return true;
		}

		// if (verbose) spdlog::info("[topology_graph] get_line_intersection = FALSE");
		return false; // No collision
	}
	catch (exception e)
	{
		spdlog::error("[topology_graph-get_line_intersection] Exception: %s\n", e.what());
		return false;
	}
	catch (...)
	{
		spdlog::error("[topology_graph-get_line_intersection] Unknown Exception");
		return false;
	}
}

bool CGraphWrapper::segment_intersect_node(geometry_msgs::msg::Pose segment_ini, geometry_msgs::msg::Pose segment_end, std::vector<std::string> sp_list)
{
	try
	{
		// for each Segment in sp_list
		geometry_msgs::msg::Point seg_ini, seg_end;
		for (size_t idx = 0; idx < sp_list.size(); idx += 2)
		{
			// A segment requires 2 points
			// Node data = "id label type x y yaw"
			std::vector<std::string> sp1, sp2;
			boost::split(sp1, sp_list[idx], boost::is_any_of(" "));
			boost::split(sp2, sp_list[idx + 1], boost::is_any_of(" "));

			// Check that SP belong to the same segment
			if (sp1[1] != sp2[1])
			{
				spdlog::error("[topology_graph-segment_intersect_node] Two SP have different labels. This should NOT happen.");
				return false;
			}

			// set start of the segment
			seg_ini.x = std::atof(sp1[3].c_str());
			seg_ini.y = std::atof(sp1[4].c_str());
			seg_ini.z = 0.0;
			// set end of the segment
			seg_end.x = std::atof(sp2[3].c_str());
			seg_end.y = std::atof(sp2[4].c_str());
			seg_end.z = 0.0;

			// Draw Segment
			// DrawSegment_intersection(seg_ini, seg_end, segment_ini.position, segment_end.position);

			// check if this pair of SP intersect with given segment
			float intersec_x, intersec_y;
			bool doIntersect = get_line_intersection(seg_ini.x, seg_ini.y, seg_end.x, seg_end.y, segment_ini.position.x, segment_ini.position.y, segment_end.position.x, segment_end.position.y, &intersec_x, &intersec_y);
			// system("read -p 'Press Enter to continue...' var");

			if (doIntersect)
				return true;
		}

		// No intersection found!
		return false;
	}
	catch (exception e)
	{
		spdlog::error("[topology_graph-segment_intersect_node] Exception: %s\n", e.what());
		return false;
	}
	catch (...)
	{
		spdlog::error("[topology_graph-segment_intersect_node] Unknown Exception");
		return false;
	}
}

// -----------------------------------------------------------------------------
//      Get Navigation distance between two poses in the map
// -----------------------------------------------------------------------------
double CGraphWrapper::get_nav_distance_two_poses(geometry_msgs::msg::Pose pose_origin, geometry_msgs::msg::Pose pose_goal, std::vector<std::string> avoiding_node_types)
{
	try
	{
		GetPlan::Goal request;
		request.start.header.frame_id = "map";
		request.start.pose = pose_origin;
		request.goal.header.frame_id = "map";
		request.goal.pose = pose_goal;

		nav_msgs::msg::Path plan;
		static auto result_cb = [&](const rclcpp_action::ClientGoalHandle<GetPlan>::WrappedResult w_result)
			{
				plan = w_result.result->path;
			};

		auto goal_options = rclcpp_action::Client<GetPlan>::SendGoalOptions();
		goal_options.result_callback = result_cb;

		auto future = getPlanClient->async_send_goal(request, goal_options);
		auto result = rclcpp::spin_until_future_complete(shared_from_this(), future);
		// Check if three is a valid path between both poses
		if (result != rclcpp::FutureReturnCode::SUCCESS)
		{
			// SRV is not available!! Report Error
			spdlog::error("[topology_graph] Unable to call MAKE_PLAN service from MoveBase");
			return -1.0;
		}
		else if (plan.poses.empty())
		{
			// Path not available --> We cannot estimate a path between these two locations
			if (verbose)
				spdlog::warn("[topology_graph] MAKE_PLAN returned empty. Unable to find a path between poses [%.2f, %.2f] and [%.2f, %.2f]",
					pose_origin.position.x, pose_origin.position.y, pose_goal.position.x, pose_goal.position.y);
			return -1.0;
		}
		else
		{
			// 1. Get the list of SP nodes to check (the segments according to avoiding_node_types)
			std::vector<std::string> node_list, sp_list, full_node_list;
			for (std::string avoid_type : avoiding_node_types)
			{
				my_graph.GetNodesbyType(avoid_type, node_list);

				// Get the Segment Points associated to each node
				for (std::string node : node_list)
				{
					// Node data = "id label type x y yaw"
					std::vector<string> node_data;
					boost::split(node_data, node, boost::is_any_of(" "));

					// get Segment Points
					my_graph.GetNodesbyLabelandType(node_data[1], "SP", sp_list);

					// Add to final list
					if (sp_list.size() != 0)
					{
						full_node_list.reserve(full_node_list.size() + sp_list.size()); // preallocate memory
						full_node_list.insert(full_node_list.end(), sp_list.begin(), sp_list.end());
					}
				}
			}

			// PRINT full_node_list

			// 2. Run the path estimating nav_distances while checking that the path does not intersect with any node in full_node_list
			double dist_step = 1.0; // m
			double Ax, Ay, d = 0.0, total_nav_dist = 0.0;
			geometry_msgs::msg::Pose p_ini, p_end;
			for (size_t h = 0; h < plan.poses.size(); h++)
			{
				if (h == 0)
				{
					Ax = request.start.pose.position.x - plan.poses[h].pose.position.x;
					Ay = request.start.pose.position.y - plan.poses[h].pose.position.y;
					p_ini = plan.poses[h].pose;
				}
				else
				{
					Ax = plan.poses[h - 1].pose.position.x - plan.poses[h].pose.position.x;
					Ay = plan.poses[h - 1].pose.position.y - plan.poses[h].pose.position.y;
				}
				d += sqrt(pow(Ax, 2) + pow(Ay, 2));
				total_nav_dist += sqrt(pow(Ax, 2) + pow(Ay, 2));

				// stop condition [m]
				if (d >= dist_step)
				{
					// Set poses of segment
					p_end = plan.poses[h].pose;

					// Check if current segment interesect any segment in full_node_list
					if (segment_intersect_node(p_ini, p_end, full_node_list))
					{
						// This path crosses a CNP
						return -1.0;
					}
					else
					{
						// restart threshold
						d = 0.0;
						p_ini = p_end;
					}
				}
			}

			// End of Path
			p_end = plan.poses[plan.poses.size() - 1].pose;
			if (segment_intersect_node(p_ini, p_end, full_node_list))
			{
				// This path crosses a CNP
				return -1.0;
			}

			// Reaching this points means the patch does not intersect any given node, return nav_distance
			// free_paths.push_back(response->plan);
			return total_nav_dist;
		}
	}
	catch (tf2::TransformException& ex)
	{
		spdlog::error("[topology_graph-{}] - Error: {}", __FUNCTION__, ex.what());
		return -1;
	}
	catch (std::runtime_error& ex)
	{
		spdlog::error("[topology_graph-%s] Exception when updating TOPOLOGICAL POSE: [%s]", __FUNCTION__, ex.what());
		return -1;
	}
	catch (...)
	{
		spdlog::error("[topology_graph-%s] Unknown Exception", __FUNCTION__);
		return -1;
	}
}

//---------------------------------------
// Draws a Node/Vertex as a sphere marker
//---------------------------------------
void CGraphWrapper::DrawNode(size_t id, string node_label, string node_type, double x, double y, double yaw)
{
	// Create new marker
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = now();
	marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(marker_lifespam));
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Sphere
	marker.id = marker_id;
	marker_id++;
	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.action = visualization_msgs::msg::Marker::ADD;

	marker.ns = node_type;

	if (node_type == "space")
	{
		marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	else if (node_type == "passage" || node_type == "CNP" || node_type == "CP")
	{
		marker.scale.x = 0.15;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}
	else if (node_type == "SP")
	{
		marker.scale.x = 0.15;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}
	else if (node_type == "ING")
	{
		marker.scale.x = 0.15;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.7;
		marker.color.b = 0.8;
	}
	else if (node_type == "docking")
	{
		marker.scale.x = 0.15;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.8;
		marker.color.g = 0.8;
		marker.color.b = 0.2;
	}
	graphMarkerList.markers.push_back(marker);

	// Label
	marker.id = marker_id;
	marker_id++;
	marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.text = "(" + std::to_string(id) + ") " + node_label;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.ns = node_type;

	if (node_type == "space")
	{
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	else if (node_type == "passage" || node_type == "CNP" || node_type == "SP")
	{
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}
	else if (node_type == "SP")
	{
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}
	else if (node_type == "ING")
	{
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.7;
		marker.color.b = 0.8;
	}
	marker.pose.position.z = 0.2;
	graphMarkerList.markers.push_back(marker);
}

void CGraphWrapper::DrawArc(double from_x, double from_y, double to_x, double to_y)
{
	// Create new marker
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = now();

	marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(marker_lifespam));

	marker.ns = "arcs";

	// Arrow
	marker.id = marker_id;
	marker_id++;
	marker.type = visualization_msgs::msg::Marker::ARROW;
	marker.action = visualization_msgs::msg::Marker::ADD;
	geometry_msgs::msg::Point from, to;
	from.x = from_x;
	from.y = from_y;
	marker.points.push_back(from);
	to.x = to_x;
	to.y = to_y;
	marker.points.push_back(to);

	marker.scale.x = 0.05; // shaft diameter
	marker.scale.y = 0.05; // head diameter
	marker.scale.z = 0.1;  // head length

	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	graphMarkerList.markers.push_back(marker);
}

void CGraphWrapper::DrawSegment_intersection(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point q1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point q2)
{
	graphMarkerList.markers.clear();

	// Create new marker
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = now();
	marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(marker_lifespam));
	marker.ns = "segment_inter";

	// Line List
	marker.id = 0;
	marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.points.push_back(p1);
	marker.points.push_back(q1);
	marker.points.push_back(p2);
	marker.points.push_back(q2);

	marker.scale.x = 0.05; // shaft diameter
	marker.scale.y = 0.05; // head diameter
	marker.scale.z = 0.05; // head length

	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	graphMarkerList.markers.push_back(marker);

	// Publish Marker
	marker_pub->publish(graphMarkerList);
}

//----------------------------------
// Publish Graph as Rviz Markers
//----------------------------------
void CGraphWrapper::publishGraphAsMarkers()
{
	try
	{
		// clear markers list to force RVIZ to update them
		graphMarkerList.markers.clear();
		marker_id = 0;

		// 1. Get All nodes (vertex)
		//---------------------------
		std::vector<string> node_list;
		my_graph.GetAllNodes(node_list);

		// for each node
		for (auto n : node_list)
		{
			// Node data = "id label type x y yaw"
			std::vector<string> node_data;
			boost::split(node_data, n, boost::is_any_of(" "));

			if (node_data.size() != 6)
			{
				if (verbose)
					spdlog::warn("[topology_graph-publishGraphAsMarkers] Error incorrect node data, skipping it.");
				for (auto i : node_data)
					std::cout << i << " ";
				continue;
			}

			// draw node
			// if (verbose) spdlog::info("[topology_graph] Drawing new node");
			DrawNode(atoi(node_data[0].c_str()), node_data[1], node_data[2], atof(node_data[3].c_str()), atof(node_data[4].c_str()), atof(node_data[5].c_str()));
		}

		// 2. Get All arcs (edges)
		//---------------------------
		std::vector<string> arc_list;
		my_graph.GetAllArcs(arc_list);

		// for each arc
		for (auto n : arc_list)
		{
			// Arc data = "id_node_origin id_node_destination arc_type arc_weight"
			std::vector<string> arc_data;
			boost::split(arc_data, n, boost::is_any_of(" "));

			if (arc_data.size() != 4)
			{
				if (verbose)
					spdlog::warn("[topology_graph-publishGraphAsMarkers] Error incorrect ARC data, skipping it.");
				continue;
			}

			// get Arc coordinates
			double from_x, from_y, to_x, to_y;
			int id_from = atoi(arc_data[0].c_str());
			my_graph.GetNodeLocation((size_t)id_from, from_x, from_y);
			int id_to = atoi(arc_data[1].c_str());
			my_graph.GetNodeLocation((size_t)id_to, to_x, to_y);

			// draw arc
			// if (verbose) spdlog::info("[topology_graph] Drawing new arc");
			DrawArc(from_x, from_y, to_x, to_y);
		}
		// Publish Marker
		marker_pub->publish(graphMarkerList);

		// 3. Add Free Paths
		/*
		if (num_path < free_paths.size())
		{
			path_pub.publish(free_paths[num_path]);
			spdlog::info("[Graph] publishing path %i", num_path);
			num_path ++;
		}
		else
		{
			num_path = 0;
			if (num_path < free_paths.size())
			{
				path_pub.publish(free_paths[num_path]);
				num_path ++;
			}
		}
		*/
	}
	catch (std::runtime_error& ex)
	{
		spdlog::error("[topology_graph-%s] Exception: [%s]", __FUNCTION__, ex.what());
		return;
	}
	catch (exception& ex)
	{
		spdlog::error("[topology_graph-%s] - Error: %s", __FUNCTION__, ex.what());
		return;
	}
	catch (...)
	{
		spdlog::error("[topology_graph-%s] Unknown Exception", __FUNCTION__);
		return;
	}
}

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	spdlog::info("[topology_graph] Initializing node... please wait!");
	rclcpp::init(argc, argv);
	auto myGraphWrapper = std::make_shared<CGraphWrapper>();

	// Loop
	rclcpp::Rate loop_rate(5);
	myGraphWrapper->marker_lifespam = 2.5;
	int count = 0;
	while (rclcpp::ok())
	{
		rclcpp::spin_some(myGraphWrapper); // Check for new srv request!
		if (count == 10)
		{
			myGraphWrapper->publishGraphAsMarkers();
			count = 0;
		}
		else
			count++;

		loop_rate.sleep();
	}

	// GoodBye
	return 1;
}
