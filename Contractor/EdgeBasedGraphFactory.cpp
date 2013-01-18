/*
 open source routing machine
 Copyright (C) Dennis Luxen, others 2010

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU AFFERO General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU Affero General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 or see http://www.gnu.org/licenses/agpl.txt.
 */

#include "EdgeBasedGraphFactory.h"

template<>
EdgeBasedGraphFactory::EdgeBasedGraphFactory(int nodes, std::vector<NodeBasedEdge> & inputEdges, std::vector<NodeID> & bn, std::vector<NodeID> & tl, std::vector<_Restriction> & irs, std::vector<NodeInfo> & nI, SpeedProfileProperties sp) : m_node_info_list_vector(nI), m_number_of_turn_restrictions(irs.size()), speedProfile(sp) {
	BOOST_FOREACH(_Restriction & restriction, irs) {
		std::pair<NodeID, NodeID> restrictionSource = std::make_pair(restriction.fromNode, restriction.viaNode);
		unsigned index;
		RestrictionMap::iterator restrIter = m_restriction_map.find(restrictionSource);
		if(restrIter == m_restriction_map.end()) {
			index = m_restriction_buckets_vector.size();
			m_restriction_buckets_vector.resize(index+1);
			m_restriction_map[restrictionSource] = index;
		} else {
			index = restrIter->second;
			//Map already contains an is_only_*-restriction
			if(m_restriction_buckets_vector.at(index).begin()->second)
				continue;
			else if(restriction.flags.isOnly){
				//We are going to insert an is_only_*-restriction. There can be only one.
				m_restriction_buckets_vector.at(index).clear();
			}
		}

		m_restriction_buckets_vector.at(index).push_back(std::make_pair(restriction.toNode, restriction.flags.isOnly));
	}

	BOOST_FOREACH(NodeID id, bn) {
		m_barrier_nodes_map[id] = true;
	}
	BOOST_FOREACH(NodeID id, tl) {
		m_traffic_lights_map[id] = true;
	}

	DeallocatingVector< InternalNodeBasedEdge > edges;
	for ( std::vector< NodeBasedEdge >::const_iterator i = inputEdges.begin(); i != inputEdges.end(); ++i ) {

		InternalNodeBasedEdge edge;
		if(!i->isForward()) {
			edge.source = i->target();
			edge.target = i->source();
			edge.data.backward = i->isForward();
			edge.data.forward = i->isBackward();
		} else {
			edge.source = i->source();
			edge.target = i->target();
			edge.data.forward = i->isForward();
			edge.data.backward = i->isBackward();
		}
		if(edge.source == edge.target)
			continue;

		if(38056 == edge.source || 38056 == edge.target || 33548 == edge.source || 38056 == edge.target)
			INFO("[" << edges.size() << "] (" << edge.source << "," << edge.target << ")");

		edge.data.distance = (std::max)((int)i->weight(), 1 );
		assert( edge.data.distance > 0 );
		edge.data.shortcut = false;
		edge.data.roundabout = i->isRoundabout();
		edge.data.ignoreInGrid = i->ignoreInGrid();
		edge.data.nameID = i->name();
		edge.data.type = i->type();
		edge.data.isAccessRestricted = i->isAccessRestricted();
		edge.data.edgeBasedNodeID = UINT_MAX;//edges.size();
		edges.push_back( edge );
		if( edge.data.backward ) {
			std::swap( edge.source, edge.target );
			edge.data.forward = i->isBackward();
			edge.data.backward = i->isForward();
			edge.data.edgeBasedNodeID = UINT_MAX;//edges.size();
			edges.push_back( edge );
		}
	}
	std::vector<NodeBasedEdge>().swap(inputEdges);

	//remove any duplicate edges.
	std::sort( edges.begin(), edges.end() );
	NodeID number_of_non_duplicate_edges = 0;
	for ( NodeID i = 0; i < edges.size(); ) {
		InternalNodeBasedEdge current_edge  =  edges[i];
        while ( i < edges.size() && edges[i].source == current_edge.source && edges[i].target == current_edge.target ) {
            if ( edges[i].data.forward )
            	current_edge.data.distance = std::min( edges[i].data.distance, current_edge.data.distance );
            ++i;
        }
        edges[number_of_non_duplicate_edges++] = current_edge;
    }
    INFO("removed " << edges.size() - number_of_non_duplicate_edges << " of " << edges.size() << " edges as duplicates");
    edges.resize( number_of_non_duplicate_edges );


    //build node-based graph
    m_node_based_dynamic_graph = boost::make_shared<InternalNodeBasedDynamicGraph>( nodes, edges );
}

void EdgeBasedGraphFactory::GetEdgeBasedEdges(DeallocatingVector< EdgeBasedEdge >& outputEdgeList ) {
	GUARANTEE(0 == outputEdgeList.size(), "Vector passed to EdgeBasedGraphFactory::GetEdgeBasedEdges(..) is not empty");
	m_edge_based_edges_vector.swap(outputEdgeList);
}

void EdgeBasedGraphFactory::GetEdgeBasedNodes( DeallocatingVector< EdgeBasedNode> & nodes) {
#ifndef NDEBUG
	BOOST_FOREACH(EdgeBasedNode & node, m_edge_based_nodes_vector){
		assert(node.lat1 != INT_MAX); assert(node.lon1 != INT_MAX);
		assert(node.lat2 != INT_MAX); assert(node.lon2 != INT_MAX);
	}
#endif
	nodes.swap(m_edge_based_nodes_vector);
}

void EdgeBasedGraphFactory::GetOriginalEdgeData( std::vector< OriginalEdgeData> & oed) {
	oed.swap(m_original_edge_data_vector);
}

NodeID EdgeBasedGraphFactory::CheckForEmanatingIsOnlyTurn(const NodeID u, const NodeID v) const {
	std::pair < NodeID, NodeID > restriction_source = std::make_pair(u, v);
	RestrictionMap::const_iterator restriction_iterator = m_restriction_map.find(restriction_source);
	if (restriction_iterator != m_restriction_map.end()) {
		unsigned index = restriction_iterator->second;
		BOOST_FOREACH(RestrictionSource restriction_target, m_restriction_buckets_vector.at(index)) {
			if(restriction_target.second) {
				return restriction_target.first;
			}
		}
	}
	return UINT_MAX;
}

bool EdgeBasedGraphFactory::CheckIfTurnIsRestricted(const NodeID u, const NodeID v, const NodeID w) const {
	//only add an edge if turn is not a U-turn except it is the end of dead-end street.
	std::pair < NodeID, NodeID > restriction_source = std::make_pair(u, v);
	RestrictionMap::const_iterator restriction_iterator = m_restriction_map.find(restriction_source);
	if (restriction_iterator != m_restriction_map.end()) {
		unsigned index = restriction_iterator->second;
		BOOST_FOREACH(RestrictionTarget restriction_target, m_restriction_buckets_vector.at(index)) {
			if(w == restriction_target.first)
				return true;
		}
	}
	return false;
}

void EdgeBasedGraphFactory::InsertEdgeBasedNode(
		const InternalNodeBasedDynamicGraph::EdgeIterator edge_iterator,
		const InternalNodeBasedDynamicGraph::NodeIterator u,
		const InternalNodeBasedDynamicGraph::NodeIterator v,
		const bool belongsToTinyComponent) {
	InternalNodeBasedDynamicGraph::EdgeData & data = m_node_based_dynamic_graph->GetEdgeData(edge_iterator);
	EdgeBasedNode current_node;
	current_node.nameID = data.nameID;
	current_node.lat1 = m_node_info_list_vector[u].lat;
	current_node.lon1 = m_node_info_list_vector[u].lon;
	current_node.lat2 = m_node_info_list_vector[v].lat;
	current_node.lon2 = m_node_info_list_vector[v].lon;
	current_node.belongsToTinyComponent = belongsToTinyComponent;
	current_node.id = data.edgeBasedNodeID;
	current_node.ignoreInGrid = data.ignoreInGrid;
	current_node.weight = data.distance;
	m_edge_based_nodes_vector.push_back(current_node);
}

void EdgeBasedGraphFactory::Run(const char * originalEdgeDataFilename) {
	Percent percentage_logger(m_node_based_dynamic_graph->GetNumberOfNodes());
	int number_of_skipped_turns(0);
	int number_of_node_based_edges(0);
	unsigned number_of_original_edges(0);
	std::ofstream originalEdgeDataOutFile(originalEdgeDataFilename, std::ios::binary);
	originalEdgeDataOutFile.write((char*)&number_of_original_edges, sizeof(unsigned));

	//variables needed to remove trivial degree-2-nodes
	int number_of_skipped_edges(0);
	std::vector<NodeID> deleted_nodes;
	std::vector<NodeID> in_degrees_vector(m_node_based_dynamic_graph->GetNumberOfNodes(), 0);

	// count the in-degree for each node
	for(InternalNodeBasedDynamicGraph::NodeIterator u = 0; u < m_node_based_dynamic_graph->GetNumberOfNodes(); ++u ) {
		for(InternalNodeBasedDynamicGraph::EdgeIterator edge_uv_iterator = m_node_based_dynamic_graph->BeginEdges(u); edge_uv_iterator < m_node_based_dynamic_graph->EndEdges(u); ++edge_uv_iterator) {
			const InternalNodeBasedDynamicGraph::NodeIterator v = m_node_based_dynamic_graph->GetTarget(edge_uv_iterator);
			++in_degrees_vector[v];
		}
	}
	//
	//	for(unsigned i = 0; i < in_degrees_vector.size(); ++i) {
	//		if(in_degrees_vector[i] >= 10) {
	//			INFO("Node " << i << " has degree " << in_degrees_vector[i] << " at " << inputNodeInfoList[i].lat/100000. << "," << inputNodeInfoList[i].lon/100000.);
	//		}
	//	}

	INFO("Before trivial contraction: " << m_node_based_dynamic_graph->GetNumberOfEdges());

	percentage_logger.reinit(m_node_based_dynamic_graph->GetNumberOfNodes());
	//loop over all edges and generate new set of nodes.
	//loop over graph and remove all trivially contractable edges.
	for(InternalNodeBasedDynamicGraph::NodeIterator u = 0; u < m_node_based_dynamic_graph->GetNumberOfNodes(); ++u ) {
		for(InternalNodeBasedDynamicGraph::EdgeIterator edge_uv_iterator = m_node_based_dynamic_graph->BeginEdges(u); edge_uv_iterator < m_node_based_dynamic_graph->EndEdges(u); ++edge_uv_iterator) {
			InternalNodeBasedDynamicGraph::NodeIterator v = m_node_based_dynamic_graph->GetTarget(edge_uv_iterator);
			const InternalNodeBasedDynamicGraph::EdgeData & edge_uv_data = m_node_based_dynamic_graph->GetEdgeData(edge_uv_iterator);
			for(InternalNodeBasedDynamicGraph::EdgeIterator edge_vw_iterator = m_node_based_dynamic_graph->BeginEdges(v); edge_vw_iterator < m_node_based_dynamic_graph->EndEdges(v); ++edge_vw_iterator) {
				InternalNodeBasedDynamicGraph::NodeIterator w = m_node_based_dynamic_graph->GetTarget(edge_vw_iterator);
				const InternalNodeBasedDynamicGraph::EdgeData & edge_vw_data = m_node_based_dynamic_graph->GetEdgeData(edge_vw_iterator);

				//indicators if we have a true 'degree-2-edge'
				bool node_is_degree_two_in_oneways = ((u != w) && (1 == in_degrees_vector[v]) && (1 == m_node_based_dynamic_graph->GetOutDegree(v)) );
				bool node_is_degree_four_in_twoways = ((u != w) && (2 == in_degrees_vector[v] && 2 == m_node_based_dynamic_graph->GetOutDegree(v)) && (edge_uv_data.forward && edge_uv_data.backward) && (edge_vw_data.forward && edge_vw_data.backward));
				//edge pair is good. let's remove it
				if( node_is_degree_two_in_oneways || node_is_degree_four_in_twoways ) {
					//Don't contract via nodes of turn restrictions
					if(CheckIfTurnIsRestricted(u,v,w))
						continue;
					//Is the angle close to geometric unimportance (<-- lovely phrase)
					double angle_of_uvw_turn = GetAngleBetweenTwoEdges(m_node_info_list_vector[u], m_node_info_list_vector[v], m_node_info_list_vector[w]);
					//Check if edge data is sufficiently equal
					if((angle_of_uvw_turn > 179. && angle_of_uvw_turn < 181.)
							&& edge_uv_data.nameID 				== edge_vw_data.nameID
							&& edge_uv_data.forward 			== edge_vw_data.forward
							&& edge_uv_data.backward 			== edge_vw_data.backward
							&& edge_uv_data.type 				== edge_vw_data.type
							&& edge_uv_data.isAccessRestricted 	== edge_vw_data.isAccessRestricted) {


						const InternalNodeBasedDynamicGraph::EdgeData & edge_vu_data = m_node_based_dynamic_graph->GetEdgeData(m_node_based_dynamic_graph->FindEdge(v,u));
						const InternalNodeBasedDynamicGraph::EdgeData & edge_wv_data = m_node_based_dynamic_graph->GetEdgeData(m_node_based_dynamic_graph->FindEdge(w,v));

						//remove edges (u,v), (v,w)
						m_node_based_dynamic_graph->DeleteEdgesTo(u, v);
						m_node_based_dynamic_graph->DeleteEdgesTo(v, w);
						m_node_based_dynamic_graph->DeleteEdgesTo(v, u);
						m_node_based_dynamic_graph->DeleteEdgesTo(w, v);

						//create (v,w) with combined data
						InternalNodeBasedEdgeData new_edge_data;
						new_edge_data = edge_uv_data; new_edge_data.distance += edge_vw_data.distance;

						//count the edges appropriately
						if(node_is_degree_four_in_twoways)
							number_of_skipped_edges+=2;
						else
							++number_of_skipped_edges;

						//add new edge (v,w) only if it doen not exist yet
						if(m_node_based_dynamic_graph->EndEdges(u) == m_node_based_dynamic_graph->FindEdge(u,w))
							m_node_based_dynamic_graph->InsertEdge(u, w, new_edge_data);

						//adjust edge data
						new_edge_data = edge_vu_data; new_edge_data.distance += edge_wv_data.distance;

						if(m_node_based_dynamic_graph->EndEdges(w) == m_node_based_dynamic_graph->FindEdge(w,u))
							m_node_based_dynamic_graph->InsertEdge(w, u, new_edge_data);
						deleted_nodes.push_back(v);
					}
				}
			}
		}
	}
	INFO("trivial contraction removed " << deleted_nodes.size() << " nodes and " << number_of_skipped_edges << " edges ");

#ifndef NDEBUG
	//check if nodes get deleted twice
	int vector_size_before_unique = deleted_nodes.size();
	std::sort(deleted_nodes.begin(), deleted_nodes.end());
	deleted_nodes.erase(std::unique(deleted_nodes.begin(), deleted_nodes.end()), deleted_nodes.end());
	INFO("double deleted1 " << (vector_size_before_unique - deleted_nodes.size()) << " nodes");
	assert(vector_size_before_unique == deleted_nodes.size());
#endif

	//cleanup
	std::vector<unsigned>().swap(in_degrees_vector);
	std::vector<unsigned>().swap(deleted_nodes);

	//loop over graph and generate contigous(!) edgebasedNodeIDs
	unsigned edge_based_nodeID_counter = 0;
	for(InternalNodeBasedDynamicGraph::NodeIterator u = 0; u < m_node_based_dynamic_graph->GetNumberOfNodes(); ++u ) {
		for(InternalNodeBasedDynamicGraph::EdgeIterator forward_edge_iterator = m_node_based_dynamic_graph->BeginEdges(u); forward_edge_iterator < m_node_based_dynamic_graph->EndEdges(u); ++forward_edge_iterator) {
			InternalNodeBasedDynamicGraph::NodeIterator v = m_node_based_dynamic_graph->GetTarget(forward_edge_iterator);
			InternalNodeBasedDynamicGraph::EdgeData & forward_edge_data = m_node_based_dynamic_graph->GetEdgeData(forward_edge_iterator);
			if(UINT_MAX == forward_edge_data.edgeBasedNodeID && forward_edge_data.forward) {
				forward_edge_data.edgeBasedNodeID = edge_based_nodeID_counter++;
//				if(edge_based_nodeID_counter == 46074) {
//					INFO("(" << u << "," << v << "): " << m_node_info_list_vector[u].lat/100000. << "," << m_node_info_list_vector[u].lon/100000. << "->" << m_node_info_list_vector[v].lat/100000. << "," << m_node_info_list_vector[v].lon/100000.)
//					INFO(forward_edge_data.edgeBasedNodeID << ": fw: " << (forward_edge_data.forward ? "yes" : "no") << ", bw: " << (forward_edge_data.backward ? "yes" : "no"))
//					ERR("has reverse: " << ( ( m_node_based_dynamic_graph->FindEdge(v,u) < m_node_based_dynamic_graph->EndEdges(v))? "yes":"no"))
//				}
				//Check if edge (v,u) also exists and if yes, then give it ID+1
				InternalNodeBasedDynamicGraph::EdgeIterator reverse_edge_iterator = m_node_based_dynamic_graph->FindEdge(v,u);
				if(reverse_edge_iterator < m_node_based_dynamic_graph->EndEdges(v)) {
					InternalNodeBasedDynamicGraph::EdgeData & reverse_edge_data = m_node_based_dynamic_graph->GetEdgeData(reverse_edge_iterator);
					if(UINT_MAX == reverse_edge_data.edgeBasedNodeID) {
						reverse_edge_data.edgeBasedNodeID = edge_based_nodeID_counter++;
//						if(edge_based_nodeID_counter == 46074) {
//							ERR(reverse_edge_data.edgeBasedNodeID << ": fw: " << (reverse_edge_data.forward ? "yes" : "no") << ", bw: " << (reverse_edge_data.backward ? "yes" : "no"))
//						}
//					} else {
//						INFO("fwd-edge-id: " << forward_edge_data.edgeBasedNodeID << ", rev-edge-id: " <<  reverse_edge_data.edgeBasedNodeID);
//						for(InternalNodeBasedDynamicGraph::EdgeIterator f2 = m_node_based_dynamic_graph->BeginEdges(u); f2 < m_node_based_dynamic_graph->EndEdges(u); ++f2) {
//							INFO("edge (" << u << "," << m_node_based_dynamic_graph->GetTarget(f2) << ")")
//						}
//						for(InternalNodeBasedDynamicGraph::EdgeIterator f2 = m_node_based_dynamic_graph->BeginEdges(v); f2 < m_node_based_dynamic_graph->EndEdges(v); ++f2) {
//							INFO("edge (" << v << "," << m_node_based_dynamic_graph->GetTarget(f2) << ")")
//						}
//
//						ERR("Should not happen, u:" << u << ", v:" << v << ", fwd-iter: " << forward_edge_iterator << ", rev-iter: " << reverse_edge_iterator);
					}
				}
			}
		}
	}



	//Identify small components
	INFO("Identifying small components");
	//Run a BFS on the undirected graph and identify small components
	std::queue<std::pair<NodeID, NodeID> > bfs_queue;
	std::vector<unsigned> component_indices_vector(m_node_based_dynamic_graph->GetNumberOfNodes(), UINT_MAX);
	std::vector<NodeID> vectorOfComponentSizes;
	unsigned current_component_id = 0, current_component_size = 0;
	//put unexplorered node with parent pointer into queue
	for(NodeID node = 0, endNodes = m_node_based_dynamic_graph->GetNumberOfNodes(); node < endNodes; ++node) {
		if(UINT_MAX == component_indices_vector[node]) {
			bfs_queue.push(std::make_pair(node, node));
			//mark node as read
			component_indices_vector[node] = current_component_id;
			percentage_logger.printIncrement();
			while(!bfs_queue.empty()) {
				//fetch element from BFS queue
				std::pair<NodeID, NodeID> currentQueueItem = bfs_queue.front();
				bfs_queue.pop();
				//                INFO("sizeof queue: " << bfsQueue.size() <<  ", sizeOfCurrentComponents: " <<  sizeOfCurrentComponent << ", settled nodes: " << settledNodes++ << ", max: " << endNodes);
				const NodeID v = currentQueueItem.first;  //current node
				const NodeID u = currentQueueItem.second; //parent
				//increment size counter of current component
				++current_component_size;
				const bool isBollardNode = (m_barrier_nodes_map.find(v) != m_barrier_nodes_map.end());
				if(!isBollardNode) {
					const NodeID onlyToNode = CheckForEmanatingIsOnlyTurn(u, v);

					//relaxieren edge outgoing edge like below where edge-expanded graph
					for(InternalNodeBasedDynamicGraph::EdgeIterator e2 = m_node_based_dynamic_graph->BeginEdges(v); e2 < m_node_based_dynamic_graph->EndEdges(v); ++e2) {
						InternalNodeBasedDynamicGraph::NodeIterator w = m_node_based_dynamic_graph->GetTarget(e2);

						if(onlyToNode != UINT_MAX && w != onlyToNode) { //We are at an only_-restriction but not at the right turn.
							continue;
						}
						if( u != w ) { //only add an edge if turn is not a U-turn except it is the end of dead-end street.
							if (!CheckIfTurnIsRestricted(u, v, w) ) { //only add an edge if turn is not prohibited
								//insert next (node, parent) only if w has not yet been explored
								if(UINT_MAX == component_indices_vector[w]) {
									//mark node as read
									component_indices_vector[w] = current_component_id;
									bfs_queue.push(std::make_pair(w,v));
									percentage_logger.printIncrement();
								}
							}
						}
					}
				}
			}
			//push size into vector
			vectorOfComponentSizes.push_back(current_component_size);
			//reset counters;
			current_component_size = 0;
			++current_component_id;
		}
	}
	INFO("identified: " << vectorOfComponentSizes.size() << " many components");

	/*********/
	INFO("generating edge-expanded nodes");

	//loop over all edges and generate new set of nodes.
	for(InternalNodeBasedDynamicGraph::NodeIterator u = 0; u < m_node_based_dynamic_graph->GetNumberOfNodes(); ++u ) {
		for(InternalNodeBasedDynamicGraph::EdgeIterator edge_uv_iterator = m_node_based_dynamic_graph->BeginEdges(u); edge_uv_iterator < m_node_based_dynamic_graph->EndEdges(u); ++edge_uv_iterator) {
			InternalNodeBasedDynamicGraph::NodeIterator v = m_node_based_dynamic_graph->GetTarget(edge_uv_iterator);
			if(m_node_based_dynamic_graph->GetEdgeData(edge_uv_iterator).type != SHRT_MAX) {
				assert(edge_uv_iterator != UINT_MAX);
				assert(u != UINT_MAX);
				assert(v != UINT_MAX);
				//                INFO("u: " << u << ", v: " << v << ", componentsIndex[u]: " << componentsIndex[u] << ", componentsIndex[v]: " << componentsIndex[v]);
				//edges that end on bollard nodes may actually be in two distinct components
				InsertEdgeBasedNode(edge_uv_iterator, u, v, (std::min(vectorOfComponentSizes[component_indices_vector[u]], vectorOfComponentSizes[component_indices_vector[v]]) < 1000) );
			}
		}
	}

	std::vector<NodeID>().swap(vectorOfComponentSizes);
	std::vector<NodeID>().swap(component_indices_vector);

	INFO("generating edge-expanded edges");

	//Loop over all turns and generate new set of edges.
	//Three nested loop look super-linear, but we are dealing with a linear number of turns only.
	for(InternalNodeBasedDynamicGraph::NodeIterator u = 0; u < m_node_based_dynamic_graph->GetNumberOfNodes(); ++u ) {
		for(InternalNodeBasedDynamicGraph::EdgeIterator e1 = m_node_based_dynamic_graph->BeginEdges(u); e1 < m_node_based_dynamic_graph->EndEdges(u); ++e1) {
			++number_of_node_based_edges;
			InternalNodeBasedDynamicGraph::NodeIterator v = m_node_based_dynamic_graph->GetTarget(e1);
			if(UINT_MAX == u || UINT_MAX == v) //u and v may be UINT_MAX when they were (trivially) contracted
				continue;
			//EdgeWeight heightPenalty = ComputeHeightPenalty(u, v);
			NodeID onlyToNode = CheckForEmanatingIsOnlyTurn(u, v);
			for(InternalNodeBasedDynamicGraph::EdgeIterator e2 = m_node_based_dynamic_graph->BeginEdges(v); e2 < m_node_based_dynamic_graph->EndEdges(v); ++e2) {
				const InternalNodeBasedDynamicGraph::NodeIterator w = m_node_based_dynamic_graph->GetTarget(e2);

				if(UINT_MAX == w) //w may be UINT_MAX when it was (trivially) contracted
					continue;
				if(onlyToNode != UINT_MAX && w != onlyToNode) { //We are at an only_-restriction but not at the right turn.
					++number_of_skipped_turns;
					continue;
				}
				bool isBollardNode = (m_barrier_nodes_map.find(v) != m_barrier_nodes_map.end());
				if(u == w && 1 != m_node_based_dynamic_graph->GetOutDegree(v) ) {
					continue;
				}

				if( !isBollardNode ) { //only add an edge if turn is not a U-turn except it is the end of dead-end street.
					if (!CheckIfTurnIsRestricted(u, v, w) || (onlyToNode != UINT_MAX && w == onlyToNode)) { //only add an edge if turn is not prohibited
						const InternalNodeBasedDynamicGraph::EdgeData edgeData1 = m_node_based_dynamic_graph->GetEdgeData(e1);
						const InternalNodeBasedDynamicGraph::EdgeData edgeData2 = m_node_based_dynamic_graph->GetEdgeData(e2);
						assert(edgeData1.edgeBasedNodeID < m_node_based_dynamic_graph->GetNumberOfEdges());
						if(edgeData2.edgeBasedNodeID >= m_node_based_dynamic_graph->GetNumberOfEdges())
							INFO("edgeData2.edgeBasedNodeID: " << edgeData2.edgeBasedNodeID << ", _nodeBasedGraph->GetNumberOfEdges(): " << m_node_based_dynamic_graph->GetNumberOfEdges());
						assert(edgeData2.edgeBasedNodeID < m_node_based_dynamic_graph->GetNumberOfEdges());

						if(!edgeData1.forward || !edgeData2.forward) {
							continue;
						}

						unsigned distance = edgeData1.distance;
						if(m_traffic_lights_map.find(v) != m_traffic_lights_map.end()) {
							distance += speedProfile.trafficSignalPenalty;
						}
						TurnInstruction turnInstruction = AnalyzeTurn(u, v, w);
						if(turnInstruction == TurnInstructions.UTurn)
							distance += speedProfile.uTurnPenalty;
						//                        if(!edgeData1.isAccessRestricted && edgeData2.isAccessRestricted) {
						//                            distance += TurnInstructions.AccessRestrictionPenalty;
						//                            turnInstruction |= TurnInstructions.AccessRestrictionFlag;
						//                        }


						//distance += heightPenalty;
						//distance += ComputeTurnPenalty(u, v, w);
						assert(edgeData1.edgeBasedNodeID != edgeData2.edgeBasedNodeID);
						if(m_original_edge_data_vector.size() == m_original_edge_data_vector.capacity()-3) {
							m_original_edge_data_vector.reserve(m_original_edge_data_vector.size()*1.2);
						}
						OriginalEdgeData oed(v,edgeData2.nameID, turnInstruction);
						EdgeBasedEdge newEdge(edgeData1.edgeBasedNodeID, edgeData2.edgeBasedNodeID, m_edge_based_edges_vector.size(), distance, true, false );
						m_original_edge_data_vector.push_back(oed);
						if(m_original_edge_data_vector.size() > 100000) {
							originalEdgeDataOutFile.write((char*)&(m_original_edge_data_vector[0]), m_original_edge_data_vector.size()*sizeof(OriginalEdgeData));
							m_original_edge_data_vector.clear();
						}
						++number_of_original_edges;
						++number_of_node_based_edges;
						m_edge_based_edges_vector.push_back(newEdge);
					} else {
						++number_of_skipped_turns;
					}
				}
			}
		}
		percentage_logger.printIncrement();
	}
	number_of_original_edges += m_original_edge_data_vector.size();
	originalEdgeDataOutFile.write((char*)&(m_original_edge_data_vector[0]), m_original_edge_data_vector.size()*sizeof(OriginalEdgeData));
	originalEdgeDataOutFile.seekp(std::ios::beg);
	originalEdgeDataOutFile.write((char*)&number_of_original_edges, sizeof(unsigned));
	originalEdgeDataOutFile.close();

	INFO("sorting edge-expanded nodes");
	std::sort(m_edge_based_nodes_vector.begin(), m_edge_based_nodes_vector.end());
	INFO("removing duplicate nodes (if any)");
	INFO("node-based graph contains " << number_of_node_based_edges     << " edges");
	INFO("edge-expanded graph skipped "  << number_of_skipped_turns     << " turns, defined by " << m_number_of_turn_restrictions << " restrictions.");
	INFO("generated " << m_edge_based_nodes_vector.size() << " edge-expanded nodes");
}

TurnInstruction EdgeBasedGraphFactory::AnalyzeTurn(const NodeID u, const NodeID v, const NodeID w) const {
	if(u == w) {
		return TurnInstructions.UTurn;
	}

	InternalNodeBasedDynamicGraph::EdgeIterator edge1 = m_node_based_dynamic_graph->FindEdge(u, v);
	InternalNodeBasedDynamicGraph::EdgeIterator edge2 = m_node_based_dynamic_graph->FindEdge(v, w);

	InternalNodeBasedDynamicGraph::EdgeData & data1 = m_node_based_dynamic_graph->GetEdgeData(edge1);
	InternalNodeBasedDynamicGraph::EdgeData & data2 = m_node_based_dynamic_graph->GetEdgeData(edge2);

	//roundabouts need to be handled explicitely
	if(data1.roundabout && data2.roundabout) {
		//Is a turn possible? If yes, we stay on the roundabout!
		if( 1 == (m_node_based_dynamic_graph->EndEdges(v) - m_node_based_dynamic_graph->BeginEdges(v)) ) {
			//No turn possible.
			return TurnInstructions.NoTurn;
		} else {
			return TurnInstructions.StayOnRoundAbout;
		}
	}
	//Does turn start or end on roundabout?
	if(data1.roundabout || data2.roundabout) {
		//We are entering the roundabout
		if( (!data1.roundabout) && data2.roundabout)
			return TurnInstructions.EnterRoundAbout;
		//We are leaving the roundabout
		else if(data1.roundabout && (!data2.roundabout) )
			return TurnInstructions.LeaveRoundAbout;
	}

	//If street names stay the same and if we are certain that it is not a roundabout, we skip it.
	if( (data1.nameID == data2.nameID) && (0 != data1.nameID))
		return TurnInstructions.NoTurn;
	if( (data1.nameID == data2.nameID) && (0 == data1.nameID) && (m_node_based_dynamic_graph->GetOutDegree(v) <= 2) )
		return TurnInstructions.NoTurn;

	double angle = GetAngleBetweenTwoEdges(m_node_info_list_vector[u], m_node_info_list_vector[v], m_node_info_list_vector[w]);
	return TurnInstructions.GetTurnDirectionOfInstruction(angle);
}

unsigned EdgeBasedGraphFactory::GetNumberOfNodes() const {
	return m_node_based_dynamic_graph->GetNumberOfEdges();
}

/* Get angle of line segment (A,C)->(C,B), atan2 magic, formerly cosine theorem*/
template<class CoordinateT>
double EdgeBasedGraphFactory::GetAngleBetweenTwoEdges(const CoordinateT& A, const CoordinateT& C, const CoordinateT& B) const {
	const int v1x = A.lon - C.lon;
	const int v1y = A.lat - C.lat;
	const int v2x = B.lon - C.lon;
	const int v2y = B.lat - C.lat;

	double angle = (atan2((double)v2y,v2x) - atan2((double)v1y,v1x) )*180/M_PI;
	while(angle < 0)
		angle += 360;
	return angle;
}
