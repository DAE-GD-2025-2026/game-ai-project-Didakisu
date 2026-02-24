#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "GameAIProg/Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h"

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
	auto width = WorldSize * 2;
	auto height = WorldSize * 2;

	auto rows = 10;
	auto cols = 10;

	pAgents.SetNum(FlockSize);

	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, width, height, rows, cols, FlockSize);

	//it will be reused for each agent
	pNeighbors.Empty();
	m_NrOfNeighbors = 0;

	for (int i = 0; i < FlockSize; i++)
	{
		float spawnPosX = FMath::RandRange(-WorldSize, WorldSize);
		float spawnPosY = FMath::RandRange(-WorldSize, WorldSize);
		FVector spawnPos{ spawnPosX , spawnPosY , 0.f };
	
		//now spawn the agent actor
		ASteeringAgent* agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, spawnPos, FRotator::ZeroRotator);

		if (agent)
		{
			pAgents[i] = agent; //store each agent in the array

			if (pPartitionedSpace)
			{
				pPartitionedSpace->AddAgent(*agent);
			}
		}
	}

	if (!pAgentToEvade)
	{
		
		FVector spawnPos{ 0.f , 90.f , 0.f }; 

		ASteeringAgent * agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, spawnPos, FRotator::ZeroRotator);

		if (agent)
		{
			this->pAgentToEvade = agent;
			DrawDebugSphere(pWorld, this->pAgentToEvade->GetActorLocation(), 15.f, 8, FColor::Red, false, -1.f, true);
		}
	}

	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);

	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>();
	pEvadeBehavior->SetEvadeRadius(200.f);

	//blended
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
		{ pSeparationBehavior.get(), m_SeparationWeight},
		{ pCohesionBehavior.get() , m_CohesionWeight },
		{ pVelMatchBehavior.get() , m_VelMatchWeight },
		{ pSeekBehavior.get() , m_SeekWeight },
		{ pWanderBehavior.get() , m_WanderWeight }
	});

	//priority
	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*> {pEvadeBehavior.get(), pBlendedSteering.get()});
    // TODO: initialize the flock and the memory pool
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
	for (int i = 0; i < FlockSize; i++)
	{
		if (pAgents[i])
		{
			pAgents[i]->Destroy();
		}
	}
}

void Flock::Tick(float DeltaTime)
{
	if (pEvadeBehavior && pAgentToEvade)
	{
		FSteeringParams evadeParams;
		evadeParams.Position = pAgentToEvade->GetPosition();
		pEvadeBehavior->SetTarget(evadeParams);

		DrawDebugCircle(pWorld, pAgentToEvade->GetActorLocation(), pEvadeBehavior->GetEvadeRadius(), 32, FColor::Yellow, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0), false);
	}

	for (int i = 0; i < pAgents.Num() ; i++)
	{
		ASteeringAgent*agent = pAgents[i];
		if (!agent) continue;

		RegisterNeighbors(agent);
		
		if (pPrioritySteering)
		{
			agent->SetSteeringBehavior(pPrioritySteering.get());
		}
	}

	if (pAgentToEvade)
	{
		pAgentToEvade->SetSteeringBehavior(pWanderBehavior.get());
		DrawDebugSphere(pWorld, pAgentToEvade->GetActorLocation(), 15.f, 8, FColor::Red, false, -1.f, 0, 2.f);
	}
}

void Flock::RenderDebug()
{
	RenderNeighborhood();

	if (DebugRenderSteering)
	{
		for (int i = 0; i < pAgents.Num(); i++)
		{
			ASteeringAgent* agent = pAgents[i];
			if (!agent) continue;

			const SteeringOutput& steering = agent->GetLastSteeringOutput();
			FVector start = agent->GetActorLocation();

			FVector steeringVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.f);

			FVector end = start + steeringVector * 50.f;
			DrawDebugLine(pWorld, start, end, FColor::Green);
		}
	}
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

		ImGui::Text("Debug Rendering");

		ImGui::Checkbox("Render Steering", &DebugRenderSteering);
		ImGui::Checkbox("Render Neighbors", &DebugRenderNeighborhood);
		ImGui::Checkbox("Render Partitions", &DebugRenderPartitions);

		ImGui::Text("Behavior Weights");
		//sliders
		if (pBlendedSteering)
		{
			auto& behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

			ImGui::SliderFloat("Separation Value", &behaviors[0].Weight, 0.f, 5.f);
			ImGui::SliderFloat("Cohesion Value", &behaviors[1].Weight, 0.f, 5.f);
			ImGui::SliderFloat("Velocity Match Value", &behaviors[2].Weight, 0.f, 5.f);
			ImGui::SliderFloat("Seek Value", &behaviors[3].Weight, 0.f, 5.f);
			ImGui::SliderFloat("Wander Value", &behaviors[4].Weight, 0.f, 5.f);
		}

		ImGui::Checkbox("Use spatial partitioning", &usePartitioning);

		ImGui::Spacing();
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	return pNeighbors;
}

int Flock::GetNrOfNeighbors() const
{
	return m_NrOfNeighbors;
}

void Flock::RenderNeighborhood()
{
	// TODO: Debugrender the neighbors for the first agent in the flock

	//neighbord have a pink sphere over them
	if (!DebugRenderNeighborhood)
	{
		return;
	}

	if (pAgents.Num() == 0 || !pAgents[0])
	{
		return;
	}

	ASteeringAgent* firstAgent = pAgents[0];
	RegisterNeighbors(firstAgent);
	//draw neigborhood radius
	DrawDebugCircle(pWorld, firstAgent->GetActorLocation(), m_NeighborhoodRadius, 32, FColor::Magenta ,false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0), false);

	for (int i = 0; i < pNeighbors.Num(); i++)
	{
		if (!pNeighbors[i]) continue;
		DrawDebugSphere(pWorld, pNeighbors[i]->GetActorLocation(), 15.f, 8, FColor::Magenta, false, -1.f, 0, 2.f);
		DrawDebugLine(pWorld, firstAgent->GetActorLocation(), pNeighbors[i]->GetActorLocation(), FColor::Magenta);//line from first agent to neighbors
	}

}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	//clear previos neighbors and reset the neighbor count
	pNeighbors.Empty();
	m_NrOfNeighbors = 0;
	
	if (!pAgent)
	{
		return;
	}

	if (usePartitioning && pPartitionedSpace)
	{
		pPartitionedSpace->RegisterNeighbors(*pAgent, m_NeighborhoodRadius);

		pNeighbors = pPartitionedSpace->GetNeighbors();
		m_NrOfNeighbors = pNeighbors.Num();

		return;
	}

	for (int i = 0; i < pAgents.Num(); i++)
	{

		if (pAgents[i] == nullptr ||pAgents[i] == pAgent) //skip myself
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
	if (m_NrOfNeighbors == 0)
	{
		return FVector2D::ZeroVector;
	}

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
	if (pSeekBehavior)
	{
		pSeekBehavior->SetTarget(Target);
	}
}