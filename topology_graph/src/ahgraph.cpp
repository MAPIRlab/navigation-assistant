#include "topology_graph/ahgraph.h"
using namespace std;


CAHGraph::CAHGraph()
{
}


CAHGraph::~CAHGraph()
{
}


bool CAHGraph::LoadGraph(const std::string file)
{
    std::ifstream fin(file.c_str());
    
    boost::dynamic_properties dp;
    dp.property("id", get(&VertexProperty::id, grafo));
    dp.property("node_id", get(&VertexProperty::id, grafo));
    dp.property("label", get(&VertexProperty::label, grafo));
    dp.property("type", get(&VertexProperty::type, grafo));
    dp.property("x", get(&VertexProperty::x, grafo));
    dp.property("y", get(&VertexProperty::y, grafo));
    dp.property("yaw", get(&VertexProperty::yaw, grafo));
    dp.property("invalid", get(&VertexProperty::invalid, grafo));
        
    dp.property("id", get(&EdgeProperty::id, grafo));
    dp.property("label", get(&EdgeProperty::label, grafo));
    dp.property("type", get(&EdgeProperty::type, grafo));
    dp.property("weight", get(&EdgeProperty::weight, grafo));
        
    boost::read_graphviz(fin,grafo,dp);//,"node_id");
    return true;
}


/** Saves current graph to file **/
bool CAHGraph::SaveGraph(const std::string file)
{
    std::ofstream fout(file.c_str());
    boost::dynamic_properties dp;
    dp.property("id", get(&VertexProperty::id, grafo));
    dp.property("node_id", get(&VertexProperty::id, grafo));
    dp.property("label", get(&VertexProperty::label, grafo));
    dp.property("type", get(&VertexProperty::type, grafo));
    dp.property("x", get(&VertexProperty::x, grafo));
    dp.property("y", get(&VertexProperty::y, grafo));
    dp.property("yaw", get(&VertexProperty::yaw, grafo));
    dp.property("invalid", get(&VertexProperty::invalid, grafo));    
    
    dp.property("id", get(&EdgeProperty::id, grafo));
    dp.property("label", get(&EdgeProperty::label, grafo));
    dp.property("type", get(&EdgeProperty::type, grafo));
    dp.property("weight", get(&EdgeProperty::weight, grafo));

    boost::write_graphviz_dp(fout,grafo,dp);
	return true;
}


/** Add new node to Topology */
bool CAHGraph::AddNode(const std::string label, const std::string type, double x, double y, double yaw, size_t &id)
{
    vertex_descriptor v = boost::add_vertex(grafo);
    grafo[v].label = label;
    grafo[v].type = type;
    grafo[v].x = x;
    grafo[v].y = y;
    grafo[v].yaw = yaw;
    grafo[v].id = (size_t)v;
    grafo[v].invalid = false;

    // return Node id
    id = grafo[v].id;
    //printf("[ahgraph] Added node (%lu): %s %s %f %f\n", id, label.c_str(), type.c_str(), x, y);

    // Always success
	return true;
}


/** Add new arc to Topology given the IDs of the nodes to connect */
bool CAHGraph::AddArc(size_t idfrom, size_t idto, const std::string label, const std::string type, bool bidi, size_t &id)
{    
    vertex_descriptor from, to;
    short c = 0;
    std::pair<VertexItr, VertexItr> vp;

    // get nodes to link
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if( (size_t)*vp.first == idfrom)
        {
            from = *vp.first;
            c++;
        }

        if( (size_t)*vp.first == idto)
        {
            to = *vp.first;
            c++;
        }

        if (c==2) break;
    }

    // Set edge properties
    EdgeProperty edge_data;
    edge_data.label = label;
    edge_data.type = type;
    edge_data.weight = 1;

    // Add the edge
    std::pair<Graph::edge_descriptor, bool> result;
    result = boost::add_edge(from, to, edge_data, grafo);
    if (result.second)
    {
        //success
        id = (size_t)edge_id;
        edge_id++;
        grafo[result.first].id = id;
        //grafo[result.first].weight = 1;

        // Bidirectional Edge??
        if (bidi)
        {
            // Add the reverse edge
            result = boost::add_edge(to, from, edge_data, grafo);
            if (result.second)
            {
                //success
                id = (size_t)edge_id;
                edge_id++;
                grafo[result.first].id = id;
                //grafo[result.first].weight = 1;
            }
        }
    }

    // return
    if (result.second)
        return true;
    else
        return false;
}


/** Add new arc to Topology given the label of the nodes to be connected. */
bool CAHGraph::AddArcbyLabel(std::string labelfrom, std::string labelto, const std::string label, const std::string type, bool bidi, size_t &id)
{
    Graph::vertex_descriptor from, to;
    short c=0;            
    std::pair<VertexItr, VertexItr> vp;

    // get nodes to link
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if( grafo[*vp.first].label == labelfrom)
        {
            from=*vp.first;
            c++;
        }
        if( grafo[*vp.first].label == labelto)
        {
            to=*vp.first;
            c++;
        }
        if (c==2) break;
    }

    //Add the edge
    std::pair<Graph::edge_descriptor, bool> result;
    result = boost::add_edge(from,to,grafo);
    if (result.second)
    {
        grafo[result.first].label = label;
        grafo[result.first].type = type;
        grafo[result.first].weight = 1;
        id = (size_t)edge_id;
        grafo[result.first].id = id;
        edge_id++;

        // Bidirectional Edge??
        if (bidi)
        {
          result = boost::add_edge(to,from,grafo);
          grafo[result.first].label = label;
          grafo[result.first].type = type;
          grafo[result.first].weight = 1;
          id = (size_t)edge_id;
          grafo[result.first].id = id;
          edge_id++;
        }
    }

    // return
    if (result.second)
        return true;
    else
        return false;
}




void CAHGraph::PrintGraph ()
{
    std::pair<VertexItr, VertexItr> vp;
    std::pair<EdgeItr,EdgeItr> ve;
    std::string label,type;
    double x,y;
    size_t id;
    unsigned num_nodes,num_arcs;
    num_nodes=boost::num_vertices(grafo);
    num_arcs=boost::num_edges(grafo);
    
    //printf("Graph: %d nodes -- %d arcs\n",num_nodes,num_arcs);
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
      if (!grafo[*vp.first].invalid)
      {
	label=grafo[*vp.first].label;
	x=grafo[*vp.first].x;
	y=grafo[*vp.first].y;
	id=grafo[*vp.first].id;
      
	cout << "Node:(" << id<< ")" << label << " at [" << x<< ","<< y <<"]" <<endl;
      }
      

     }
     printf("---------\n");
     
    for (ve = edges(grafo); ve.first != ve.second; ++ve.first)
    {
      id=grafo[*ve.first].id;
      label=grafo[*ve.first].label;
      type=grafo[*ve.first].type;
      
    
      std::cout << "Arc: ("<<id<<")" << label <<", with type: " << type << " between " << grafo[boost::source(*ve.first,grafo)].label <<" and " << grafo[boost::target(*ve.first,grafo)].label <<endl;
      

     }

     printf("=============\n");

}



/** Get all nodes with a given label */
void CAHGraph::GetNodesbyLabel(const std::string given_label, std::vector<std::string> &list)
{
    list.clear();

    std::pair<VertexItr, VertexItr> vp;
    std::string label, type;
    size_t id;
    double x,y,yaw;

    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if (!grafo[*vp.first].invalid)
        {
            if (grafo[*vp.first].label == given_label)
            {
                // get vertex data
                label = grafo[*vp.first].label;
                id = grafo[*vp.first].id;
                type = grafo[*vp.first].type;
                x = grafo[*vp.first].x;
                y = grafo[*vp.first].y;
                yaw = grafo[*vp.first].yaw;

                // Compose string (id, label, type, x, y, yaw)
                std::ostringstream stringStream;
                stringStream << id << " " << label << " " << type << " " << x << " " << y << " " << yaw;
                list.push_back(stringStream.str());
            }
        }
    }
}


/** Get all nodes with a given label */
void CAHGraph::GetNodesbyType(const std::string given_type, std::vector<std::string> &list)
{
    list.clear();

    std::pair<VertexItr, VertexItr> vp;
    std::string label, type;
    size_t id;
    double x,y,yaw;

    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if (!grafo[*vp.first].invalid)
        {
            if (grafo[*vp.first].type == given_type)
            {
                // get vertex data
                label = grafo[*vp.first].label;
                id = grafo[*vp.first].id;
                type = grafo[*vp.first].type;
                x = grafo[*vp.first].x;
                y = grafo[*vp.first].y;
                yaw = grafo[*vp.first].yaw;

                // Compose string (id, label, type, x, y, yaw)
                std::ostringstream stringStream;
                stringStream << id << " " << label << " " << type << " " << x << " " << y << " " << yaw;
                list.push_back(stringStream.str());
            }
        }
    }
}


/** Get all nodes with a given label */
void CAHGraph::GetNodesbyLabelandType(const std::string given_label, const std::string given_type, std::vector<std::string> &list)
{
    list.clear();

    std::pair<VertexItr, VertexItr> vp;
    std::string label, type;
    size_t id;
    double x,y,yaw;

    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if (!grafo[*vp.first].invalid)
        {
            if ( (grafo[*vp.first].label == given_label) && (grafo[*vp.first].type == given_type) )
            {
                // get vertex data
                label = grafo[*vp.first].label;
                id = grafo[*vp.first].id;
                type = grafo[*vp.first].type;
                x = grafo[*vp.first].x;
                y = grafo[*vp.first].y;
                yaw = grafo[*vp.first].yaw;

                // Compose string (id, label, type, x, y, yaw)
                std::ostringstream stringStream;
                stringStream << id << " " << label << " " << type << " " << x << " " << y << " " << yaw;
                list.push_back(stringStream.str());
            }
        }
    }
}


/** Get the node given its ID */
std::string CAHGraph::GetNodebyId(const size_t id)
{
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if( (size_t)*vp.first == id)
        {
            std::ostringstream stringStream;
            stringStream << grafo[*vp.first].id << " " << grafo[*vp.first].label << " " << grafo[*vp.first].type << " " <<
                            grafo[*vp.first].x << " " << grafo[*vp.first].y << " " << grafo[*vp.first].yaw;
            return stringStream.str();
        }
    }

    // Node ID not found
    return "";
}


/** Set a new label of an existing node ID */
bool CAHGraph::SetNodeLabel (const size_t id,std::string label)
{

    bool res=false;
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
          if( (size_t)*vp.first==id)
          {
              grafo[(Graph::vertex_descriptor)id].label=label;
              res=true;
              break;
          }

     }

    return res;
}


/** Get a list of neighbourd nodes IDs */
void CAHGraph::GetNodeNeighbors(const size_t idnode,const std::string arc_type,std::vector<size_t> &neighbors)
{/*
    CHMHMapNode::Ptr node;
    node = grafo.getNodeByID(idnode);
	if (node)
	{
		mrpt::hmtslam::TArcList arcs;
		node->getArcs(arcs);
		TArcList::const_iterator it;
        for (it=arcs.begin(); it!=arcs.end(); ++it)
		{
            if ((*it)->m_arcType == arc_type)
				neighbors.push_back((size_t)(*it)->getNodeTo());
		}
	}
    */
}

/** Get a list containing all the nodes in the Topology. 
  * Each list element is composed of "node_id node_label node_type pos_x pos_y pos_yaw "
  */
void CAHGraph::GetAllNodes(std::vector<string> &list)
{    
    list.clear();

    std::pair<VertexItr, VertexItr> vp;
    std::string label, type;
    size_t id;
    double x,y,yaw;

    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if (!grafo[*vp.first].invalid)
        {
            // get vertex data
            label = grafo[*vp.first].label;
            id = grafo[*vp.first].id;
            type = grafo[*vp.first].type;
            x = grafo[*vp.first].x;
            y = grafo[*vp.first].y;
            yaw = grafo[*vp.first].yaw;

            // Compose string (id, label, type, x, y, yaw)
            std::ostringstream stringStream;
            stringStream << id << " " << label << " " << type << " " << x << " " << y << " " << yaw;
            list.push_back(stringStream.str());
        }
    }
}

/** Get a list of all arcs in the Topology.
  * Each element in the List is made of "id_node_origin id_node_destination arc_type arc_weight"
  */
void CAHGraph::GetAllArcs(std::vector<string> &list)
{
    list.clear();

    std::pair<EdgeItr,EdgeItr> ve;
    std::string type;
    vertex_descriptor v1,v2;
    float w;
     
    for (ve = edges(grafo); ve.first != ve.second; ++ve.first)
    {
        // get edge data
        type = grafo[*ve.first].type;
        w = grafo[*ve.first].weight;
        v1 = boost::source(*ve.first, grafo);
        v2 = boost::target(*ve.first, grafo);

        // Compose string
        std::ostringstream stringStream;
        stringStream << grafo[v1].id << " " << grafo[v2].id << " " << type << " " << w;
        list.push_back(stringStream.str() );
     }
}



/** Delete all nodes of type arc_type outcomming from given node */
void CAHGraph::DeleteOutcommingArcs(const size_t idnode,const std::string arc_type)
{/*
    CHMHMapNode::Ptr node;
	node=grafo.getNodeByID(idnode);
	if (node)
	{
		mrpt::hmtslam::TArcList arcs;
		node->getArcs(arcs);
		TArcList::iterator it;
		for (it=arcs.begin();it!=arcs.end();++it)
		{
            if (((*it)->getNodeFrom()==idnode) && (*it)->m_arcType==arc_type)
			{
                //it->clear();
                it->reset();
			}
		}
        printf("[ahgraph] Sucessfully deleted Outcomming arcs from node: %s\n",node->m_label.c_str());
	}
	else
        printf("[ahgraph] Error deleting OutcommingArcs from node_ID: %lu. Node NOT found.\n",idnode);
    */

}

/** Delete all nodes of type arc_type incomming from given node */
void CAHGraph::DeleteIncommingArcs(const size_t idnode,const std::string arc_type)
{/*
    CHMHMapNode::Ptr node;
	node=grafo.getNodeByID(idnode);
	if (node)
	{
		mrpt::hmtslam::TArcList arcs;
		node->getArcs(arcs);
		TArcList::iterator it;

		for (it=arcs.begin();it!=arcs.end();++it)
		{
            if (((*it)->getNodeTo()==idnode) && (*it)->m_arcType==arc_type)
			{
                //it->clear();
                it->reset();
			}
		}
        printf("[ahgraph] Sucessfully deleted Incomming arcs from node: %s\n",node->m_label.c_str());
	}
	else
        printf("[ahgraph] Error deleting IncommingArcs from node_ID: %d. Node NOT found.\n",idnode);
    */

}


/** Delete all nodes of type arc_type outcomming or incomming from given node */
void CAHGraph::DeleteAllArcs(const uint64_t idnode,const std::string arc_type)
{/*
    CHMHMapNode::Ptr node;
	node=grafo.getNodeByID(idnode);
	if (node)
	{
		mrpt::hmtslam::TArcList arcs;
		node->getArcs(arcs);

		TArcList::iterator it;

		for (it=arcs.begin();it!=arcs.end();++it)
		{
            if( (*it)->m_arcType==arc_type )
			{
                //it->clear();
                it->reset();
			}
		}
        printf("[ahgraph] Sucessfully deleted All arcs from node: %s\n",node->m_label.c_str());
	}
	else
        printf("[ahgraph] Error deleting AllArcs from node_ID: %lu. Node NOT found.\n",idnode);
    */

}

/** Delete all nodes outcomming or incomming from given node */
void CAHGraph::DeleteAllArcs(const uint64_t id_node)
{
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if( (size_t)*vp.first == id_node)
        {
            boost::clear_vertex(*vp.first, grafo);   //Remove all edges connected to this node
            return;
        }
    }
    return;

  /*  CHMHMapNode::Ptr node;
	node=grafo.getNodeByID(idnode);
	if (node)
	{
		mrpt::hmtslam::TArcList arcs;
		node->getArcs(arcs);
		TArcList::iterator it;

		for (it=arcs.begin();it!=arcs.end();++it)
        {
            //it->clear();
            it->reset();
        }
        printf("[ahgraph] Sucessfully deleted All arcs from node: %s\n",node->m_label.c_str());
	}
	else
        printf("[ahgraph] Error deleting AllArcs from node_ID: %lu. Node NOT found.\n",idnode);
        */

}

/** Delete all arcs of type arc_type between two given nodes */
bool CAHGraph::DeleteArcsBetweenNodes(size_t idfrom, size_t idto, const std::string type)
{	
  //I do not care about types....
    std::pair<EdgeItr,EdgeItr> ve;
    
    vertex_descriptor v1,v2;
          
     
    for (ve = edges(grafo); ve.first != ve.second; ++ve.first)
    {
      
    
      v1=boost::source(*ve.first,grafo);
      v2=boost::target(*ve.first,grafo);
      if (grafo[v1].id==idfrom && grafo[v2].id==idto)
      {
	
	boost::remove_edge(*ve.first,grafo);
      }

      
      

     }

           
  
  
  
return true;
}




/** Find a path between node Start and node End
  * if found, PATH = ["id1 label1 type1 x1 y1 yaw1", "id2 label2 x2 y2", ...]"
  */
bool CAHGraph::FindPath(size_t idstart, size_t idend, std::vector<std::string> &path)
{
    vertex_descriptor s = boost::vertex(idstart, grafo);
    vertex_descriptor goal = boost::vertex(idend, grafo);
    std::vector<vertex_descriptor> p(num_vertices(grafo));
    std::vector<float> d(num_vertices(grafo));

    IndexMap indexMap = boost::get(boost::vertex_index, grafo);
    PredecessorMap predecessorMap(&p[0], indexMap);
    DistanceMap distanceMap(&d[0], indexMap);
    boost::dijkstra_shortest_paths(grafo, s, boost::predecessor_map(&p[0]).weight_map(boost::get(&EdgeProperty::weight,grafo)));
     
  
    typedef std::vector<Graph::edge_descriptor> PathType;
    PathType boostpath;
   
    for(vertex_descriptor u = predecessorMap[goal]; // Start by setting 'u' to the destintaion node's predecessor
                          u != goal; // Keep tracking the path until we get to the source
                          goal = u, u = predecessorMap[goal]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        std::pair<edge_descriptor, bool> edgePair = boost::edge(u, goal, grafo);
        edge_descriptor edge = edgePair.first;
        boostpath.push_back( edge );
    }
 
 
    // Prepare output
    path.clear();
    PathType::reverse_iterator pathIterator;
    for(pathIterator = boostpath.rbegin(); pathIterator != boostpath.rend(); ++pathIterator)
    {
        std::ostringstream stringStream;
        stringStream << grafo[boost::source(*pathIterator,grafo)].id << " " <<
                        grafo[boost::source(*pathIterator,grafo)].label << " " <<
                        grafo[boost::source(*pathIterator,grafo)].type << " " <<
                        grafo[boost::source(*pathIterator,grafo)].x << " " <<
                        grafo[boost::source(*pathIterator,grafo)].y << " " <<
                        grafo[boost::source(*pathIterator,grafo)].yaw;
        path.push_back( stringStream.str() );
    }

    // Add goal vertex
    goal = boost::vertex(idend, grafo);
    std::ostringstream stringStream;
    stringStream << grafo[goal].id << " " <<
                    grafo[goal].label << " " <<
                    grafo[goal].type << " " <<
                    grafo[goal].x << " " <<
                    grafo[goal].y << " " <<
                    grafo[goal].yaw;
    path.push_back( stringStream.str() );

    return true;
}


/** change the location (x,y) of an existing Node */
bool CAHGraph::SetNodeLocation(string node_label,double x,double y)
{
  
  
    size_t id = -1;     // default invalid id
    std::string comp="";
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {

        comp=grafo[*vp.first].label;
        if (comp==node_label)
        {
	  grafo[*vp.first].x=x;
	  grafo[*vp.first].y=y;
	  return true;
            
        }

     }
  return false;
   
}

/** Get location (x,y) of exisitng node */
bool CAHGraph::GetNodeLocation(string node_label,double &x,double &y)
{
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if (grafo[*vp.first].label == node_label)
        {
            x = grafo[*vp.first].x;
            y = grafo[*vp.first].y;
            return true;
        }
    }
    return false;
}

/** Get location (x,y) of exisitng node */
bool CAHGraph::GetNodeLocation(size_t node_id,double &x,double &y)
{
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if( (size_t)*vp.first == node_id)
        {
            x = grafo[*vp.first].x;
            y = grafo[*vp.first].y;
            return true;
        }
     }
    return false;
}


bool CAHGraph::GetNodeByLocation(double x,double y,double tolerance,size_t &id)
{
  
    id = -1;     // default invalid id
    double vx,vy,distance;
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
	  vx=grafo[*vp.first].x;
	  vy=grafo[*vp.first].y;
	  distance=fabs(vx-x)+fabs(vy-y);
	  if (distance < tolerance)
	  {
	    id=grafo[*vp.first].id;
	    return true;
	  }
            
     }
    return false;
  
}


/** Check if node with given label already exists in topology */
bool CAHGraph::ExistsNodeLabel(const std::string label)
{
 
  size_t id = -1;     // default invalid id
    std::string comp="";
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {

        comp=grafo[*vp.first].label;
        if (comp==label)
        {
	  return true;
            
        }

     }
    return false;  
}

bool CAHGraph::DeleteNode(const size_t id_node)
{
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if( (size_t)*vp.first == id_node)
        {
            boost::clear_vertex(*vp.first, grafo);   //Remove all edges connected to this node
            //boost::remove_vertex_and_renumber_indices(*vp.first,grafo);   //Problems
            grafo[*vp.first].invalid = true;        // Set invalid
            return true;
        }
    }
    return false;
}


/**  Delete a node from Topology */
bool CAHGraph::DeleteNode(const std::string label)
{
    std::pair<VertexItr, VertexItr> vp;
    for (vp = vertices(grafo); vp.first != vp.second; ++vp.first)
    {
        if (grafo[*vp.first].label == label)
        {
            boost::clear_vertex(*vp.first, grafo);   //Remove all edges connected to this node
            //boost::remove_vertex_and_renumber_indices(*vp.first,grafo);   //Problems
            grafo[*vp.first].invalid = true;        // Set invalid
            return true;
        }
    }
    return false;
}

/*
bool CAHGraph::DeleteNode(const std::string label)
{
std::string comp="";
VertexItr vi, vi_end, next;
tie(vi, vi_end) = vertices(grafo);
for (next = vi; vi != vi_end; vi = next) {
  ++next;
  
    comp=grafo[*vi].label;
        if (comp==label)
        {
	    remove_vertex(*vi, grafo);
	}
}
return true;
}*/
