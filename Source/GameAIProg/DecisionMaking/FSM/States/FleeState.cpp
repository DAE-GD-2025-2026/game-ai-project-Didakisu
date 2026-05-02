#include "FleeState.h"
#include "DecisionMaking/ThiefAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

GameAI::FSM::FleeState::FleeState(ASteeringAgent* Agent, ASteeringAgent* Guard):
	Agent(Agent), Guard(Guard)
{

}

void GameAI::FSM::FleeState::OnEnter()
{
	UE_LOG(LogTemp, Warning, TEXT("ENTER FLEE STATE"));
	if (AThiefAIController* AIController = Cast<AThiefAIController>(Agent->GetController()))
	{
		AIController->StopMovement();
	}
}

void GameAI::FSM::FleeState::OnExit()
{
	if (AThiefAIController* AIController = Cast<AThiefAIController>(Agent->GetController()))
	{
		AIController->StopMovement();
	}
}

void GameAI::FSM::FleeState::Update(float deltaTime)
{
	if (!Agent || !Guard)
	{
		return;
	}

	FVector AgentLocation = Agent->GetActorLocation();
	FVector GuardLocation = Guard->GetActorLocation();

	//direction away from guard
	FVector direction = AgentLocation - GuardLocation;
	direction.Z = 0.f;
	direction.Normalize();

	float fleeDistance = 800.f;
	FVector destination = AgentLocation + direction * fleeDistance;

	if (AThiefAIController* AIController = Cast<AThiefAIController>(Agent->GetController()))
	{
		AIController->MoveToLocation(destination);
	}
}