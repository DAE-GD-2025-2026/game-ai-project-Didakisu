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
    UE_LOG(LogTemp, Warning, TEXT("SEARCH ENTER")); 

	SearchTimer = 0.f;
	bHasReachedLastTargetPosition = false;
    LastKnownPosition = FVector::ZeroVector;
}

void GameAI::FSM::SearchState::OnExit()
{

}

void GameAI::FSM::SearchState::Update(float deltaTime)
{
    SearchTimer += deltaTime;

    if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
    {
        if (!Blackboard)
        {
            Blackboard = AIController->GetBlackboardComponent();
        }

        if (LastKnownPosition == FVector::ZeroVector && Blackboard)
        {
            LastKnownPosition = Blackboard->GetValueAsVector("LastKnownPos");
        }

        float distance = FVector::Distance(Agent->GetActorLocation(), LastKnownPosition);

        if (!bHasReachedLastTargetPosition)
        {
            AIController->MoveToLocation(LastKnownPosition);

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
                AIController->MoveToLocation(LastKnownPosition + RandomOffset);
                WanderTimer = 0.f;
            }
        }
    }
}