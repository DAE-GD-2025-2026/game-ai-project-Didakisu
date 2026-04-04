#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	auto* pNavPoly = pNavGraph->GetNavPolygon();
	auto* startTriangle = pNavPoly->GetTriangleAtPosition(startPos, true);
	auto* endTriangle = pNavPoly->GetTriangleAtPosition(endPos, true);

	if(!startTriangle || !endTriangle)
	{
		return finalPath;
	}

	if (startTriangle == endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	
	//startNode
	auto pGraphClone = pNavGraph->Clone();
	std::unique_ptr<NavGraphNode> startNode = std::make_unique<NavGraphNode>(startPos, -1);
	int startNodeId = pGraphClone->AddNode(std::move(startNode));//start node cannot be owned by more than 1 owner

	const auto& startTriangleEdges = startTriangle->GetEdges();
	for (int i = 0; i < startTriangleEdges.size(); i++)
	{
		auto edgeId = pNavGraph->GetNavPolygon()->FindEdgeIndex(startTriangleEdges[i]);
		if (edgeId.has_value())
		{
			int connectionNodeId = pGraphClone->GetNodeIdFromEdgeIndex(edgeId.value());
			if (connectionNodeId != Graphs::InvalidNodeId)
			{
				pGraphClone->AddConnection(startNodeId, connectionNodeId);
				auto* pConnection = pGraphClone->FindConnection(startNodeId, connectionNodeId);//find conn
				auto* pConnectedNode = pGraphClone->GetNodeAs<NavGraphNode>(connectionNodeId);//get the node from the conn as a navgraphnoce
				pConnection->SetWeight(FVector2D::Distance(startPos, pConnectedNode->GetPosition()));
			}
		}
	}


	//endNode
	std::unique_ptr<NavGraphNode> endNode = std::make_unique<NavGraphNode>(endPos, -1);
	int endNodeId = pGraphClone->AddNode(std::move(endNode));

	const auto& endTriangleEdges = endTriangle->GetEdges();
	for (int i = 0; i < endTriangleEdges.size(); i++)
	{
		auto edgeId = pNavGraph->GetNavPolygon()->FindEdgeIndex(endTriangleEdges[i]);
		if (edgeId.has_value())
		{
			int connectionNodeId = pGraphClone->GetNodeIdFromEdgeIndex(edgeId.value());
			if (connectionNodeId != Graphs::InvalidNodeId)
			{
				pGraphClone->AddConnection(endNodeId, connectionNodeId);
				auto* pConnection = pGraphClone->FindConnection(endNodeId, connectionNodeId);//find conn
				auto* pConnectedNode = pGraphClone->GetNodeAs<NavGraphNode>(connectionNodeId);//get the node from the conn as a navgraphnoce
				pConnection->SetWeight(FVector2D::Distance(endPos, pConnectedNode->GetPosition()));
			}
		}
	}

	AStar aStar(pGraphClone.get(), HeuristicFunctions::Euclidean);
	auto path = aStar.FindPath(pGraphClone->GetNodeAs<NavGraphNode>(startNodeId), pGraphClone->GetNodeAs<NavGraphNode>(endNodeId));

	for (int p = 0; p < path.size(); p++)
	{
		finalPath.push_back(path[p]->GetPosition());
		debugNodePositions.push_back(path[p]->GetPosition());
	}

	//Run A star on new graph

	//Debug Visualisation

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	if (!path.empty())
	{
		debugPortals = SSFA::FindPortals(path, *pNavGraph->GetNavPolygon());
		finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	}
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}