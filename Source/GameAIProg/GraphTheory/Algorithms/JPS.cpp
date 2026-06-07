#include "JPS.h"

using namespace GameAI;

//TODO: check if i can get the cell size from somewhere, not hardcode it!

GameAI::JPS::JPS(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{}

std::vector<Node*>JPS::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	NodeRecord currentNodeRecord{};

	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;

	openList.push_back(startRecord);

	while (!openList.empty())
	{
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());//get node with lowest cost

		if (currentNodeRecord.pNode == pGoalNode)
		{
			UE_LOG(LogTemp, Warning, TEXT("Goal was FOUND!"));
			break;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Goal was NOT FOUND!"));
		}

		std::vector<FVector2D> directions;

		if (currentNodeRecord.pConnection == nullptr)//if node has no parent
		{
			//start node has no parent,so dir is unknown, could be all 8

			directions =
			{
				{1,0} , {0,1} , {-1,0} , {0,-1} ,
				{1,1} , {-1,1} , {1,-1} , {-1,-1}
			};
		}
		else //node has a parent, we came from somewhere
		{
			//get the parent id
			Node* parentNode = pGraph->GetNode(currentNodeRecord.pConnection->GetFromId()).get();
			//get a vector pointing from parent to current node
			FVector2D dir = currentNodeRecord.pNode->GetPosition() - parentNode->GetPosition();

			auto dirX = GetDirection(dir.X);
			auto dirY = GetDirection(dir.Y);

			directions.push_back({ (float)dirX, (float)dirY });

			auto pos = currentNodeRecord.pNode->GetPosition();

			if (dirX != 0 && dirY == 0)
			{
				Node* nodeAbove = nullptr;
				Node* nodeBelow = nullptr;

				nodeAbove = GetNodeAtPosition(pos + FVector2D(0, 1) * m_CellSize);
				nodeBelow = GetNodeAtPosition(pos + FVector2D(0, -1) * m_CellSize);

				if (nodeAbove == nullptr)
				{
					directions.push_back({ (float)dirX , 1 });
				}
				if (nodeBelow == nullptr)
				{
					directions.push_back({ (float)dirX , -1 });
				}
			}
			else if (dirX == 0 && dirY != 0)
			{
				Node* nodeLeft = nullptr;
				Node* nodeRight = nullptr;

				nodeLeft = GetNodeAtPosition(pos + FVector2D(-1, 0) * m_CellSize);
				nodeRight = GetNodeAtPosition(pos + FVector2D(1, 0) * m_CellSize);

				if (nodeLeft == nullptr)
				{
					directions.push_back({ -1,(float)dirY });
				}
				if (nodeRight == nullptr)
				{
					directions.push_back({ 1, (float)dirY });
				}
			}
			else
			{
				Node* nodeLeft = nullptr;
				Node* nodeBelow = nullptr;

				nodeLeft = GetNodeAtPosition(pos + FVector2D(-dirX, 0) * m_CellSize);
				nodeBelow = GetNodeAtPosition(pos + FVector2D(0, -dirY) * m_CellSize);

				if (nodeLeft == nullptr)
				{
					directions.push_back({ (float)-dirX,(float)dirY });
				}
				if (nodeBelow == nullptr)
				{
					directions.push_back({ (float)dirX, (float)-dirY });
				}

				//when moving diagonally explore straigt directions too
				directions.push_back({ (float)dirX , 0 });
				directions.push_back({ 0 , (float)dirY });
			}
		}

		for (int i = 0; i < directions.size(); i++)
		{
			auto jumpPoint = Jump(currentNodeRecord.pNode, directions[i], pGoalNode);
			UE_LOG(LogTemp, Warning, TEXT("Jump result: %s"), jumpPoint ? TEXT("FOUND") : TEXT("NULL"));

			if (jumpPoint == nullptr)
			{
				continue;
			}
			else
			{
				//g(y) = g(x) + dist(x; y)
				FVector2D distanceToJumpPoint = jumpPoint->GetPosition() - currentNodeRecord.pNode->GetPosition();
				auto totalGCost = currentNodeRecord.costSoFar + distanceToJumpPoint.Length();

				bool skipConnection = false;

				for (int a = 0; a < closedList.size(); a++)
				{
					if (closedList[a].pNode == jumpPoint)
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

				for (int b = 0; b < openList.size(); b++)
				{
					if (openList[b].pNode == jumpPoint)
					{
						if (totalGCost >= openList[b].costSoFar)
						{
							skipConnection = true;
						}
						else
						{
							openList.erase(openList.begin() + b);
						}
						break;
					}
				}

				if (skipConnection)
				{
					continue;
				}

				Connection* con = nullptr;
				auto connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());//we find the connection from the current node we are

				//then we loop through them
				for (int c = 0; c < connections.size(); c++)
				{
					Node* neighbor = pGraph->GetNode(connections[c]->GetToId()).get(); //get each neihbor from the 8 neighbors surrounding the current node
					FVector2D neighborDir = neighbor->GetPosition() - currentNodeRecord.pNode->GetPosition();
					//get dirs
					int neighborDirX = GetDirection(neighborDir.X);
					int neighborDirY = GetDirection(neighborDir.Y);
					//check which neighbor is in the direction of the jump
					if (neighborDirX == directions[i].X && neighborDirY == directions[i].Y)
					{
						con = connections[c];
						break;
					}
				}

				NodeRecord newNodeRecord{};

				newNodeRecord.pNode = jumpPoint;
				newNodeRecord.pConnection = con;
				newNodeRecord.costSoFar = totalGCost;
				newNodeRecord.estimatedTotalCost = totalGCost + GetHeuristicCost(jumpPoint, pGoalNode);

				openList.push_back(newNodeRecord);
			}
		}

		auto it = std::find(openList.begin(), openList.end(), currentNodeRecord);
		UE_LOG(LogTemp, Warning, TEXT("Open list size: %d"), (int)openList.size());
		openList.erase(it);
		closedList.push_back(currentNodeRecord);
	}

	//backtracking

	if (currentNodeRecord.pNode == pGoalNode)
	{
		while (currentNodeRecord.pNode != pStartNode)//currentNodeRecord contains start node at the beginning of backtracking
		{
			path.push_back(currentNodeRecord.pNode);
			if (currentNodeRecord.pConnection == nullptr)
			{
				UE_LOG(LogTemp, Warning, TEXT("pConnection is nullptr during backtracking!"));
				break;
			}
			auto previousNode = currentNodeRecord.pConnection->GetFromId();

			for (int i = 0; i < closedList.size(); i++)
			{
				if (closedList[i].pNode->GetId() == previousNode)
				{
					currentNodeRecord = closedList[i];
				}
			}
		}
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}

float JPS::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}

int JPS::GetDirection(float value)
{
	if (value > 0.f)
	{
		return 1;
	}
	if (value < 0.f)
	{
		return -1;
	}

	return 0;
}

Node* JPS::Jump(Node* currentNode, FVector2D direction, Node* pGoalNode)
{
	UE_LOG(LogTemp, Warning, TEXT("Jumping from: %f, %f in direction: %f, %f"),
		currentNode->GetPosition().X,
		currentNode->GetPosition().Y,
		direction.X,
		direction.Y);

	//take one step further from the current node in direction
	FVector2D nextPos = currentNode->GetPosition() + direction * m_CellSize;
	Node* nextNode = nullptr;

	nextNode = GetNodeAtPosition(nextPos);

	if (nextNode == nullptr)
	{
		return nullptr;
	}

	if (direction.X != 0 && direction.Y != 0)
	{
		auto pos = nextNode->GetPosition();
		Node* straightX = GetNodeAtPosition(pos + FVector2D(direction.X, 0) * m_CellSize);
		Node* straightY = GetNodeAtPosition(pos + FVector2D(0, direction.Y) * m_CellSize);

		if (straightX == nullptr && straightY == nullptr)
		{
			return nullptr;
		}
	}

	/*else
	{*/
	if (nextNode == pGoalNode)
	{
		return nextNode;
	}

	if (HasForcedNeighbour(nextNode, direction))
	{
		return nextNode;
	}
	else
	{
		if (direction.X != 0 && direction.Y != 0) //moving diagonally
		{
			//jump vertically and horizontally
			if (Jump(nextNode, FVector2D(direction.X, 0), pGoalNode) != nullptr)
			{
				//if we find a jump point, then nextNode itself ia a jump point and we retrun it
				return nextNode;
			}

			if (Jump(nextNode, FVector2D(0, direction.Y), pGoalNode) != nullptr)
			{
				return nextNode;
			}
		}
		return Jump(nextNode, direction, pGoalNode);
	}
	/*}*/
}

bool JPS::HasForcedNeighbour(Node* currentNode, FVector2D direction) //identifying successors
{
	Node* nodeAbove = nullptr;
	Node* nodeBelow = nullptr;
	Node* nodeDiagAbove = nullptr;
	Node* nodeDiagBelow = nullptr;

	Node* nodeLeft = nullptr;
	Node* nodeRight = nullptr;
	Node* nodeDiagLeft = nullptr;
	Node* nodeDiagRight = nullptr;

	auto pos = currentNode->GetPosition();

	if (direction.X != 0 && direction.Y == 0) //horizontal
	{
		nodeAbove = GetNodeAtPosition(pos + FVector2D(0, 1) * m_CellSize);
		nodeDiagAbove = GetNodeAtPosition(pos + FVector2D(direction.X, 1) * m_CellSize);

		nodeBelow = GetNodeAtPosition(pos + FVector2D(0, -1) * m_CellSize);
		nodeDiagBelow = GetNodeAtPosition(pos + FVector2D(direction.X, -1) * m_CellSize);

		if (nodeAbove == nullptr && nodeDiagAbove != nullptr) //if there is a wall above me , and the node above diagoanlly is walkable
		{
			return true; //it has a forced neighbor
		}

		if (nodeBelow == nullptr && nodeDiagBelow != nullptr) //if there is a wall above me , and the node above diagoanlly is walkable
		{
			return true; //it has a forced neighbor
		}
	}
	else if (direction.X == 0 && direction.Y != 0) //vertical
	{
		nodeRight = GetNodeAtPosition(pos + FVector2D(1, 0) * m_CellSize);
		nodeDiagRight = GetNodeAtPosition(pos + FVector2D(1, direction.Y) * m_CellSize);

		nodeLeft = GetNodeAtPosition(pos + FVector2D(-1, 0) * m_CellSize);
		nodeDiagLeft = GetNodeAtPosition(pos + FVector2D(-1, direction.Y) * m_CellSize);

		if (nodeLeft == nullptr && nodeDiagLeft != nullptr)
		{
			return true;
		}

		if (nodeRight == nullptr && nodeDiagRight != nullptr)
		{
			return true;
		}
	}
	else if (direction.X != 0 && direction.Y != 0) //diagonally
	{
		//we check for a wall on our left , and below us
		nodeLeft = GetNodeAtPosition(pos + FVector2D(-direction.X, 0) * m_CellSize);
		nodeBelow = GetNodeAtPosition(pos + FVector2D(0, -direction.Y) * m_CellSize);

		//we check oif the diagonal above lft and below right are walkable
		nodeDiagLeft = GetNodeAtPosition(pos + FVector2D(-direction.X, direction.Y) * m_CellSize);
		nodeDiagBelow = GetNodeAtPosition(pos + FVector2D(direction.X, -direction.Y) * m_CellSize);

		if (nodeLeft == nullptr && nodeDiagLeft != nullptr)
		{
			return true;
		}

		if (nodeBelow == nullptr && nodeDiagBelow != nullptr)
		{
			return true;
		}
	}

	return false;
}

Node* JPS::GetNodeAtPosition(const FVector2D& pos) const
{
	const auto& activeNodes = pGraph->GetActiveNodes();

	for (int i = 0; i < activeNodes.size(); i++)
	{
		if (FVector2D::Distance(activeNodes[i]->GetPosition(), pos) < 0.1f)
		{
			TerrainNode* terrainNode = dynamic_cast<TerrainNode*>(activeNodes[i]);
			if (terrainNode && terrainNode->GetType() == TerrainNode::Type::Water)
			{
				return nullptr;
			}

			return activeNodes[i];
		}
	}
	return nullptr;
}