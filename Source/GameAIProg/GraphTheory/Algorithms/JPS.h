#pragma once

#include <vector>
#include "../../Shared/Graph/Graph.h"
#include "Heuristics.h"

namespace GameAI
{
	class JPS
	{
	public:
		JPS(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord final
		{
			Node* pNode = nullptr;
			Connection* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		std::vector<Node*> FindPath(Node* const pStartNode, Node* const pDestinationNode);

	private:
		float GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const;
		int GetDirection(float value);
		Node* Jump(Node* currentNode, FVector2D direction, Node* pGoalNode);
		bool HasForcedNeighbour(Node* currentNode, FVector2D direction);

		Node* GetNodeAtPosition(const FVector2D& pos) const;

		Graph* pGraph;
		HeuristicFunctions::Heuristic HeuristicFunction;

		const float m_CellSize{ 200.f };
	};
}