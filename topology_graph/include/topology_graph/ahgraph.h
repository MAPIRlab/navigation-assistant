#ifndef __CAHGRAPH_H
#define __CAHGRAPH_H

#include <string>
#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/properties.hpp> 
#include <boost/property_map/property_map.hpp>


struct VertexProperty
{
  size_t        id;
  std::string   label;
  std::string   type;
  double        x, y, yaw;
  bool          invalid;
};



struct EdgeProperty
{
  size_t        id;
  std::string   label;
  std::string   type;
  float         weight;
};


// Graph created with vecS. In the current version of boots 1.58 vertices cannot be removed safely, so I just remove edges and mark
// node as invalid. A solution could have been used listS instead, but this make the rest of code has a bunch of errors :S
// By now it is enough. Also edges have an id which is kept "manually"

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperty,EdgeProperty> Graph;
typedef boost::directed_graph<> DGraph;

typedef boost::graph_traits<Graph>::vertex_iterator VertexItr;
typedef boost::graph_traits<Graph>::edge_iterator EdgeItr;
typedef boost::graph_traits < Graph >::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits < Graph >::edge_descriptor edge_descriptor;

typedef boost::property_map < Graph, boost::vertex_index_t >::type IndexMap;
typedef boost::iterator_property_map < vertex_descriptor*, IndexMap, vertex_descriptor, vertex_descriptor& > PredecessorMap;
typedef boost::iterator_property_map < float*, IndexMap, float, float& > DistanceMap;

class CAHGraph
{
public:
	CAHGraph();
	~CAHGraph();
	bool LoadGraph(const std::string file);
	bool SaveGraph(const std::string file);
	void PrintGraph ();

    // Add
    bool AddNode(const std::string label, const std::string type, double x, double y, double yaw, size_t &id);
    bool AddArc(size_t idfrom, size_t idto, const std::string label, const std::string type, bool bidi, size_t &id);
    bool AddArcbyLabel(std::string labelfrom, std::string labelto, const std::string label, const std::string type, bool bidi, size_t &id);

    // Gets
    void GetAllNodes(std::vector<std::string> &list);
    void GetAllArcs(std::vector<std::string> &list);
    void GetNodesbyLabel(const std::string given_label, std::vector<std::string> &list);
    void GetNodesbyType(const std::string given_type, std::vector<std::string> &list);
    void GetNodesbyLabelandType(const std::string given_label, const std::string given_type, std::vector<std::string> &list);

    std::string GetNodebyId(const size_t id);

    bool SetNodeLabel(const size_t id, std::string label);
	void GetNodeNeighbors(const size_t idnode,const std::string arc_type,std::vector<size_t> &neighbors);

	/** Delete all Arcs of type arc_type outcomming from a given node */
	void DeleteOutcommingArcs(const size_t idnode,const std::string arc_type);
	/** Delete all Arcs of type arc_type incomming from a given node */
	void DeleteIncommingArcs(const size_t idnode,const std::string arc_type);
	/** Delete all Arcs of type arc_type outcomming or incomming from a given node */
	void DeleteAllArcs(const uint64_t idnode,const std::string arc_type);
	/** Delete all Arcs outcomming or incomming from a given node */
    void DeleteAllArcs(const uint64_t id_node);





    //Return a path as a vector of nodes
    bool FindPath(size_t idstart, size_t idend, std::vector<std::string> &path);

	bool GetNodeLocation(std::string node_label,double &x,double &y);
    bool GetNodeLocation(size_t node_id, double &x, double &y);

	bool SetNodeLocation(std::string node_label,double x,double y);

	// Returns the (first) closest node to a give position (x,y). Tolerance is used to limit the search
	// It returns false and id=-1 if no node is found in the tolerance radius.
	bool GetNodeByLocation(double x,double y,double tolerance,size_t &id);
	
	
	// Check if node with given label already exists in topology
	bool ExistsNodeLabel (const std::string label);

	// Delete all arcs of type arc_type between two given nodes
	bool DeleteArcsBetweenNodes(size_t idfrom, size_t idto, const std::string type);

	// Delete a node from Topology
	bool DeleteNode(const std::string label);
	bool DeleteNode(const size_t id_node);

private:
    Graph grafo;        // Create a graph.
    unsigned edge_id=0;
};
#endif
