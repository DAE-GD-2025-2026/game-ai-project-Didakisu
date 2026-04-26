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
	private:
		ASteeringAgent* Agent{ nullptr };
		FVector LastKnownPosition;
		float SearchTimer{ 0.f };
		float MaxSearchingTime{ 5.f };
		bool bHasReachedLastTargetPosition{ false };
		std::unique_ptr<Wander> WanderBehavior;
		float threshold{ 50.f };
	};
}