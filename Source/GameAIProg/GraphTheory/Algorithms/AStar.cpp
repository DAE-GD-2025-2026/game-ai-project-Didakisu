#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	NodeRecord currentNodeRecord{};

	NodeRecord startRecord{};

	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

	openList.push_back(startRecord);

	while (!openList.empty())
	{
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());

		if (currentNodeRecord.pNode == pGoalNode)
		{
			break;
		}
		else
		{
			auto connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());

			for (int i = 0; i < connections.size(); i++)
			{
				auto nodeId = connections[i]->GetToId();
				Node* pNextNode = pGraph->GetNode(nodeId).get();

				auto totalGCost = currentNodeRecord.costSoFar + connections[i]->GetWeight();

				bool skipConnection = false; //added this flag to continue correctly

				for (int a = 0; a < closedList.size(); a++)
				{
					if (closedList[a].pNode == pNextNode)
					{
						if (totalGCost >= closedList[a].costSoFar)
						{
							skipConnection = true; 
						}
						else
						{
							closedList.erase(closedList.begin() + a);
						}
						break;
					}
				}

				if (skipConnection)
				{
					continue;
				}

				for (int a = 0; a < openList.size(); a++)
				{
					if (openList[a].pNode == pNextNode)
					{
						if (totalGCost >= openList[a].costSoFar)
						{
							skipConnection = true;
						}
						else
						{
							openList.erase(openList.begin() + a);
						}
						break;
					}
				}

				if (skipConnection)
				{
					continue;
				}

				NodeRecord newNodeRecord{};

				newNodeRecord.pConnection = connections[i];
				newNodeRecord.pNode = pNextNode;
				newNodeRecord.costSoFar = totalGCost;
				newNodeRecord.estimatedTotalCost = totalGCost + GetHeuristicCost(pNextNode, pGoalNode);

				openList.push_back(newNodeRecord);
			}

			auto it = std::find(openList.begin(), openList.end(), currentNodeRecord);
			openList.erase(it);
			closedList.push_back(currentNodeRecord);
		}
	}

	//backtracking

	while (currentNodeRecord.pNode != pStartNode)//currentNodeRecord contains start node at the beginning of backtracking
	{
		path.push_back(currentNodeRecord.pNode);
		auto previousNode = currentNodeRecord.pConnection->GetFromId();

		for (int i = 0; i < closedList.size(); i++)
		{
			if (closedList[i].pNode->GetId() == previousNode)
			{
				currentNodeRecord = closedList[i];
			}
		}
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}