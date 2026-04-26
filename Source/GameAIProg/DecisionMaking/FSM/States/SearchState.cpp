#include "SearchState.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DecisionMaking/GameAIController.h"

GameAI::FSM::SearchState::SearchState(ASteeringAgent* Agent):
	Agent(Agent)
{

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

    if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
    {
        if (!bHasReachedLastTargetPosition)
        {
            AIController->MoveToLocation(LastKnownPosition);

            float distance = FVector::Distance(Agent->GetActorLocation(), LastKnownPosition);
            if (distance < threshold)
            {
                bHasReachedLastTargetPosition = true;
                WanderTimer = 0.f;
            }
        }
        else
        {
            WanderTimer += deltaTime;

            if (WanderTimer > 2.f)
            {
                FVector RandomOffset = FVector(FMath::RandRange(-400.f, 400.f), FMath::RandRange(-400.f, 400.f), 0.f);
                FVector WanderTarget = LastKnownPosition + RandomOffset;
                AIController->MoveToLocation(WanderTarget);
                WanderTimer = 0.f;
            }
        }
    }
}