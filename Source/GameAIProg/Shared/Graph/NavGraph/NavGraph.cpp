#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	const auto& edges = pNavPoly->GetEdges();
	const auto& triangles = pNavPoly->GetTriangles();

	for (int i = 0; i < edges.size(); i++)
	{
		//check if an edge is shared between 2 triangles
		int sharedCount = 0;
		for (int a = 0; a < triangles.size(); a++)
		{
			if (triangles[a].HasEdge(edges[i]))
			{
				sharedCount++;
			}
		}

		if (sharedCount < 2)
		{
			continue;
		}

		//create node

		auto p1 = edges[i].GetP1(*pNavPoly);
		auto p2 = edges[i].GetP2(*pNavPoly);

		auto midpoint = (FVector2D{ (p1.X + p2.X) / 2 , (p1.Y + p2.Y) / 2 });

		AddNode(std::make_unique<NavGraphNode>(midpoint, i));
	}

	for (int a = 0; a < triangles.size(); a++)
	{
		const auto& triangleEdges = triangles[a].GetEdges();
		std::vector<int> validNodeIds; 

		for (int e = 0; e < triangleEdges.size(); e++)
		{
			auto indx = pNavPoly->FindEdgeIndex(triangleEdges[e]);
			if (indx.has_value())
			{
				int nodeId = GetNodeIdFromEdgeIndex(indx.value());
				if (nodeId != Graphs::InvalidNodeId)
				{
					validNodeIds.push_back(nodeId);
				}
			}
		}

		if (validNodeIds.size() == 2)
		{
			AddConnection(validNodeIds[0], validNodeIds[1]);
		}

		if (validNodeIds.size() == 3)
		{
			AddConnection(validNodeIds[0], validNodeIds[1]);
			AddConnection(validNodeIds[0], validNodeIds[2]);
			AddConnection(validNodeIds[1], validNodeIds[2]);
		}

	}
	
	SetConnectionCostsToDistances();
	//3. Set the connections cost to the actual distance
}
