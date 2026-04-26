#pragma once
#include "CoreMinimal.h"
#include "../FSM.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

class ASteeringAgent;
//class Wander;

namespace GameAI::FSM
{
	class SearchState : public State
	{
	public:
		SearchState(ASteeringAgent* Agent);

		void OnEnter() override;
		void OnExit() override;
		void Update(float deltaTime) override;

		float SearchTimer{ 0.f };
		float MaxSearchingTime{ 15.f };
	private:
		ASteeringAgent* Agent{ nullptr };
		FVector LastKnownPosition;
		bool bHasReachedLastTargetPosition{ false };
		float threshold{ 50.f };
		float WanderTimer{ 0.f };
	};
}