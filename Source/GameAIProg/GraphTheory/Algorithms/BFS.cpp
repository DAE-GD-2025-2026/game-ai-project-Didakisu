#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;

	std::queue<Node*> openList{};
	std::vector<Node*> closedList{};
	std::map<Node*, Node*> previousNode{};

	bool foundInClosedList{ false };

	openList.push(pStartNode);
	closedList.push_back(pStartNode);

	while (!openList.empty())
	{
		Node* currentNode = openList.front();
		openList.pop();

		if (currentNode == pDestinationNode)
		{
			break;
		}
		else
		{
			auto connections = pGraph->FindConnectionsFrom(currentNode->GetId());

			for (int i = 0; i < connections.size(); i++)
			{
				foundInClosedList = false;

				auto nodeId = connections[i]->GetToId();
				Node* pNextNode = pGraph->GetNode(nodeId).get();

				for (int a = 0; a < closedList.size(); a++)
				{
					if (closedList[a] == pNextNode)
					{
						foundInClosedList = true;
						break;
					}
				}

				if (!foundInClosedList)
				{
					previousNode[pNextNode] = currentNode;//the previous node of the next node is the current node
					closedList.push_back(pNextNode);
					openList.push(pNextNode);
				}
			}
		}
	}

	Node* currentNode = pDestinationNode;
	while (currentNode != pStartNode)
	{
		path.push_back(currentNode);
		currentNode = previousNode[currentNode];//the previous node of the current node is the new current node
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	
	
	return path;
}