#include "SearchState.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DecisionMaking/GameAIController.h"

GameAI::FSM::SearchState::SearchState(ASteeringAgent* Agent):
	Agent(Agent), WanderBehavior(std::make_unique<Wander>())
{
	//WanderBehavior = std::make_unique<Wander>();
}

void GameAI::FSM::SearchState::OnEnter()
{
	SearchTimer = 0.f;
	bHasReachedLastTargetPosition = false;

	if (Blackboard)
	{
		LastKnownPosition = Blackboard->GetValueAsVector("LastKnownPos");
	}
}

void GameAI::FSM::SearchState::OnExit()
{

}

void GameAI::FSM::SearchState::Update(float deltaTime)
{
	SearchTimer += deltaTime;
	//first go to the thief's last known position
	if (!bHasReachedLastTargetPosition)
	{
		if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
		{
			AIController->MoveToLocation(LastKnownPosition);

			//check if player arrived at the last known pos
			float distance = FVector::Distance(Agent->GetActorLocation(), FVector(LastKnownPosition.X, LastKnownPosition.Y, 90.f));


			if (distance < threshold)
			{
				bHasReachedLastTargetPosition = true;
				//then wander
				Agent->SetSteeringBehavior(WanderBehavior.get());
			}
		}
	}

}