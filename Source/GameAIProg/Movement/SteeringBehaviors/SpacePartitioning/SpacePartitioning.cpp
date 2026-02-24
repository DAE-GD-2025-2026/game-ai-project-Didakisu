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

	// TODO create the cells
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

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	// TODO Render the cells with the number of agents inside of it
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