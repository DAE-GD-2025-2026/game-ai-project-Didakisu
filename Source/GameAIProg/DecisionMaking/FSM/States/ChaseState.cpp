#include "ChaseState.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "DecisionMaking/GameAIController.h"

GameAI::FSM::ChaseState::ChaseState(ASteeringAgent* Agent, ASteeringAgent* Target):
	Agent(Agent) , Target(Target)
{

}

void GameAI::FSM::ChaseState::OnEnter()
{
	Agent->SetSteeringBehavior(nullptr);
	ChaseTimer = 0.f;
}

void GameAI::FSM::ChaseState::OnExit()
{
	if (Blackboard && Target)
	{
		Blackboard->SetValueAsVector("LastKnownPos", Target->GetActorLocation());
	}
}

void GameAI::FSM::ChaseState::Update(float deltaTime)
{
	ChaseTimer += deltaTime;
	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
	{
		AIController->MoveToLocation(Target->GetActorLocation());
	}
}