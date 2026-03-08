#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
		{
			return Eulerianity::notEulerian;
		}
		// TODO Count nodes with odd degree 
		int oddDegreeCount = 0;
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();

		for (int i = 0; i < nodes.size(); i++)
		{
			auto connections = m_pGraph->FindConnectionsFrom(nodes[i]->GetId());
			if (connections.size() % 2 != 0)
			{
				oddDegreeCount++;
			}
		}

		if (oddDegreeCount > 2)
		{
			return Eulerianity::notEulerian;
		}

		if (oddDegreeCount == 2)
		{
			return Eulerianity::semiEulerian;
		}
		
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// TODO Check if there can be an Euler path
		// TODO If this graph is not eulerian, return the empty path
		eulerianity = IsEulerian();
		if (eulerianity == Eulerianity::notEulerian)
		{
			return Path;
		}

		if (eulerianity == Eulerianity::eulerian)
		{
			currentNodeId = Nodes[0]->GetId(); //path is eulerian,start from anywhere
		}
		else if (eulerianity == Eulerianity::semiEulerian)
		{
			for (int i = 0; i < Nodes.size(); i++)
			{
				auto connections = graphCopy.FindConnectionsFrom(Nodes[i]->GetId());
				if (connections.size() % 2 != 0) //path is semi eulerian, start at one of the odd degree nodes
				{
					currentNodeId = Nodes[i]->GetId();
					break;
				}
			}
		}

		// TODO Start algorithm loop
		std::stack<int> nodeStack;

		while (!graphCopy.FindConnectionsFrom(currentNodeId).empty() || !nodeStack.empty())
		{
			if (!graphCopy.FindConnectionsFrom(currentNodeId).empty())
			{
				nodeStack.push(currentNodeId);
				int neighbor = graphCopy.FindConnectionsFrom(currentNodeId)[0]->GetToId();
				graphCopy.RemoveConnection(currentNodeId, neighbor);//removeing so we cant use it again
				currentNodeId = neighbor;
			}
			else
			{
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}

		Path.push_back(m_pGraph->GetNode(currentNodeId).get());

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node
		visited[startIndex] = true;
		// TODO Ask the graph for the connections from that node
		auto connections = m_pGraph->FindConnectionsFrom(Nodes[startIndex]->GetId());
		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
		for (int i = 0; i < connections.size(); i++)
		{
			int neighbor = connections[i]->GetToId();
			for (int a = 0; a < Nodes.size(); a++)
			{
				if (Nodes[a]->GetId() == neighbor && !visited[a])
				{
					VisitAllNodesDFS(Nodes, visited, a);
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		std::vector<bool> visited(Nodes.size(), false);//for each node

		VisitAllNodesDFS(Nodes, visited, 0);
		
		for (int i = 0; i < Nodes.size(); i++)
		{
			if (!visited[i])
			{
				return false;
			}
		}

		return true;
	}
}