#include "SpacePartitioning.h"
#include <algorithm>

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	CellOrigin = { -Width / 2 , -Height / 2 };

	for (int i = 0; i < Rows; i++)
	{
		for (int t = 0; t < Cols; t++)
		{
			Cell cell = { (float)CellOrigin.X + CellWidth * t, (float)CellOrigin.Y + CellHeight * i, CellWidth, CellHeight};
			Cells.push_back(cell);
		}
	}

}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	FVector2D agentPos = Agent.GetPosition();
	auto indx = PositionToIndex(agentPos);

	Cells[indx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	auto currPos = Agent.GetPosition();

	auto newPosIndex = PositionToIndex(currPos);
	auto oldPosIndex = PositionToIndex(OldPos);

	if (newPosIndex != oldPosIndex)
	{
		Cells[oldPosIndex].Agents.remove(&Agent);
		Cells[newPosIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius , bool debugDraw)
{
	NrOfNeighbors = 0;

	FVector extent = { CellWidth/2 , CellHeight/2 , 0 };
	FRect rect1
	{ 
		FVector2D{Agent.GetActorLocation().X - QueryRadius , Agent.GetActorLocation().Y - QueryRadius},
		FVector2D{Agent.GetActorLocation().X + QueryRadius , Agent.GetActorLocation().Y + QueryRadius}
	};

	for (int i = 0; i < Cells.size(); i++)
	{
		FVector2D midPoint = { (Cells[i].BoundingBox.Min + Cells[i].BoundingBox.Max) / 2 };
		FVector center = { midPoint.X , midPoint.Y , 0 };

		FRect rect2{ Cells[i].BoundingBox.Min , Cells[i].BoundingBox.Max};
		if (DoRectsOverlap(rect1, rect2))
		{
			if (debugDraw)
			{
				DrawDebugSolidBox(pWorld, center, extent, FColor{ 255,0,0,100 }, false, 0.f);
			}

			for (ASteeringAgent* pNeighbor : Cells[i].Agents)
			{
				if (pNeighbor == nullptr || pNeighbor == &Agent)
				{
					continue;
				}

				auto toTarget = pNeighbor->GetPosition() - Agent.GetPosition();
				auto distance = toTarget.Length();
				if (distance <= QueryRadius)
				{
					Neighbors[NrOfNeighbors] = pNeighbor;
					NrOfNeighbors++;
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	for (int i = 0; i < Cells.size(); i++)
	{
		FVector2D midPoint = { (Cells[i].BoundingBox.Min + Cells[i].BoundingBox.Max) / 2 };
		FVector center = { midPoint.X , midPoint.Y , 0 };
		FVector extent = { CellWidth / 2 , CellHeight / 2 , 0 };
		FVector textLocation = { center.X , center.Y , 0 };
		auto agents = Cells[i].Agents.size();

		DrawDebugBox(pWorld, center, extent, FColor::Red);
		DrawDebugString(pWorld, textLocation, FString::FromInt(agents), 0, FColor::Red, 0.f);
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	auto col = floor((Pos.X - CellOrigin.X) / CellWidth);
	auto row = floor((Pos.Y - CellOrigin.Y) / CellHeight);

	row = std::clamp(row, 0.0, (double) (NrOfRows - 1));
	col = std::clamp(col, 0.0, (double) (NrOfCols - 1));

	return (NrOfCols * row) + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}