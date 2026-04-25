#include "PatrolState.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "DecisionMaking/GameAIController.h"

namespace GameAI::FSM
{
	PatrolState::PatrolState(ASteeringAgent* Agent, TArray<FVector> WayPoints):
		Agent(Agent) , WayPoints(WayPoints)
	{

	}

	void PatrolState::OnEnter()
	{

	}

	void PatrolState::OnExit()
	{

	}

	void PatrolState::Update(float deltaTime)
	{
		if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
		{
			AIController->MoveToLocation(WayPoints[CurrentWaypointIndex]);

			float distance = FVector::Distance(Agent->GetActorLocation(), WayPoints[CurrentWaypointIndex]);

			if (distance < threshold)
			{
				CurrentWaypointIndex = (CurrentWaypointIndex + 1) % WayPoints.Num();
			}
		}
	}
}