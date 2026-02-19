#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	pAgents.SetNum(FlockSize);

	//it will be reused for each agent
	pNeighbors.Empty();
	m_NrOfNeighbors = 0;

	for (int i = 0; i < FlockSize; i++)
	{
		FVector spawnPos{ 0.f , 0.f , 0.f }; //for now for simplicity all of them will spawn at this location

		//now spawn the agent actor
		ASteeringAgent* agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, spawnPos, FRotator::ZeroRotator);

		if (agent)
		{
			pAgents[i] = agent; //store each agent in the array
		}
	}

    // TODO: initialize the flock and the memory pool
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	for (int i = 0; i < pAgents.Num() ; i++)
	{
		ASteeringAgent*agent = pAgents[i];
		RegisterNeighbors(agent);
	}




  // TODO: update the flock
  // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	// TODO: Implement
	// calculates the neigbors based on the "neighborhood radius"
	// updates the pNeighbors container and their neighbor count
	// do not include an agent in its own neighborhood

	//clear previos neighbors and reset the neighbor count
	pNeighbors.Empty();
	m_NrOfNeighbors = 0;
	
	for (int i = 0; i < pAgents.Num(); i++)
	{
		if (pAgents[i] == pAgent) //skip myself
		{
			continue;
		}

		auto toTarget = pAgents[i]->GetPosition() - pAgent->GetPosition();//calculate vector from neighbor to agent
		auto distance = toTarget.Length();//get length
		if (distance <= m_NeighborhoodRadius)//if length is smaller than 200.f the agent in pAgents becomes a neighbor
		{
			pNeighbors.Add(pAgents[i]);
			m_NrOfNeighbors++;//and increade the number of neighbors
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	for (int i = 0; i < pNeighbors.Num(); i++)
	{
		avgPosition = pNeighbors[i]->GetPosition() + avgPosition;
	}

	return avgPosition / m_NrOfNeighbors;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	if (m_NrOfNeighbors == 0)
	{
		return FVector2D::ZeroVector;;
	}

	for (int i = 0; i < pNeighbors.Num(); i++)
	{
		FVector2D neighborVelocity = { pNeighbors[i]->GetVelocity().X , pNeighbors[i]->GetVelocity().Y };
		avgVelocity += neighborVelocity;
	}

	return avgVelocity / m_NrOfNeighbors;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
}

