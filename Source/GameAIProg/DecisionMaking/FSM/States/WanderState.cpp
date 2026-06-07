#include "WanderState.h"
#include "DecisionMaking/ThiefAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

GameAI::FSM::WanderState::WanderState(ASteeringAgent* Agent):
	Agent(Agent)
{

}

void GameAI::FSM::WanderState::OnEnter()
{
	bHasDestination = false;
	TimeSinceLastRepath = 0.f;
}

void GameAI::FSM::WanderState::OnExit()
{
	bHasDestination = false;

	if (AThiefAIController* AIController = Cast<AThiefAIController>(Agent->GetController()))
	{
		AIController->StopMovement();
	}
}

void GameAI::FSM::WanderState::Update(float deltaTime)
{
	if (!Agent)
		return;

	AThiefAIController* AIController = Cast<AThiefAIController>(Agent->GetController());
	if (!AIController)
		return;

	TimeSinceLastRepath += deltaTime;

	FVector Origin = Agent->GetActorLocation();

	if (TimeSinceLastRepath >= RepathInterval || !bHasDestination)
	{
		TimeSinceLastRepath = 0.f;

		FVector Forward = Agent->GetActorForwardVector();
		FVector Random = FMath::VRand() * 0.6f + Forward * 0.4f;
		Random.Z = 0.f;
		Random.Normalize();

		float Distance = 600.f;

		CurrentDestination = Origin + Random * Distance;

		bHasDestination = true;

		AIController->MoveToLocation(CurrentDestination);
	}

	float Dist = FVector::Dist(Origin, CurrentDestination);
	if (Dist < 100.f)
	{
		bHasDestination = false;
	}
}