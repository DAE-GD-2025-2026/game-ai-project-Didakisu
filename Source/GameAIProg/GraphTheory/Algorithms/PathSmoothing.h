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
		
		//For each node received, get it's corresponding line
		
			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point

			//Store portal

		//Add degenerate portal to force end evaluation

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
