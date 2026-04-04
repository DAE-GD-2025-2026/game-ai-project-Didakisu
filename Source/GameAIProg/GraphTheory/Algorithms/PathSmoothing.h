#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		
		for (int i = 0; i < Path.size(); i++)
		{
			NavGraphNode* navNode = static_cast<NavGraphNode*>(Path[i]);
			const auto& edges = NavPoly.GetEdges();

			int edgeIndex = navNode->GetEdgeIdx();
			if (edgeIndex < 0 || edgeIndex >= edges.size())
			{
				continue;
			}

			auto p1 = edges[edgeIndex].GetP1(NavPoly);
			auto p2 = edges[edgeIndex].GetP2(NavPoly);

			NavLine portal(FVector2D(p1.X , p1.Y), FVector2D(p2.X, p2.Y));

			FVector2D pathDir;
			if (i < Path.size() - 1)
			{
				pathDir = Path[i + 1]->GetPosition() - Path[i]->GetPosition();
			}
			else if (i > 0)
			{
				pathDir = Path[i]->GetPosition() - Path[i - 1]->GetPosition();
			}

			FVector2D portalDir = portal.P2 - portal.P1;

			if (Cross2D(pathDir, portalDir) < 0)
			{
				std::swap(portal.P1, portal.P2);
			}

			Portals.push_back(portal);
		}

		FVector2D goalPos = Path.back()->GetPosition();
		NavLine degeneratePortal(goalPos, goalPos);
		Portals.push_back(degeneratePortal);


		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};
		FVector2D apexPoint = Portals[0].P1;
		Path.push_back(apexPoint);

		auto leftLeg = Portals[0].P2 - apexPoint;
		auto rightLeg = Portals[0].P1 - apexPoint;

		int rightLegIndex = 0;
		int leftLegIndex = 0;
		int apexIndex = 0;
		int portalIndex = 0;

		for (int i = 1; i < Portals.size(); i++)
		{
			auto portal = Portals[i];

			FVector2D newRightLeg = portal.P1 - apexPoint;
			FVector2D newLeftLeg = portal.P2 - apexPoint;

			if (Cross2D(newRightLeg, rightLeg) >= 0)
			{
				//going inwards
				//check if we cross over the leftLeg
				if (Cross2D(newRightLeg, leftLeg) > 0)
				{
					//collapse
					apexPoint += leftLeg;
					apexIndex = leftLegIndex;
					portalIndex = leftLegIndex + 1;
					leftLegIndex = rightLegIndex = portalIndex;

					Path.push_back(apexPoint);
					
					i = portalIndex - 1;
					if (portalIndex < Portals.size())
					{
						leftLeg = Portals[portalIndex].P2 - apexPoint;
						rightLeg = Portals[portalIndex].P1 - apexPoint;
						continue;
					}
				}
				else
				{
					rightLeg = newRightLeg;
					rightLegIndex = i;
				}
			}


			if (Cross2D(newLeftLeg, leftLeg) <= 0)
			{
				if (Cross2D(newLeftLeg, rightLeg) < 0)
				{
					//collapse
					apexPoint += rightLeg;
					apexIndex = rightLegIndex;
					portalIndex = rightLegIndex + 1;
					rightLegIndex = leftLegIndex = portalIndex;

					Path.push_back(apexPoint);

					i = portalIndex - 1;
					if (portalIndex < Portals.size())
					{
						leftLeg = Portals[portalIndex].P2 - apexPoint;
						rightLeg = Portals[portalIndex].P1 - apexPoint;
						continue;
					}
				}
				else
				{
					leftLeg = newLeftLeg;
					leftLegIndex = i;
				}
			}
			
		}

		Path.push_back(Portals.back().P1);
		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};

	static float Cross2D(const FVector2D& a, const FVector2D& b)
	{
		return a.X * b.Y - a.Y * b.X;
	}
};
}
