#pragma once
#include "CoreMinimal.h"
#include "../FSM.h"

class ASteeringAgent;

namespace GameAI::FSM
{
	class WanderState : public State
	{
	public:
		WanderState(ASteeringAgent* Agent);

		void OnEnter() override;
		void OnExit() override;
		void Update(float deltaTime) override;

	private:
		ASteeringAgent* Agent{ nullptr };

		FVector CurrentDestination;
		bool bHasDestination = false;
		float TimeSinceLastRepath = 0.f;
		float RepathInterval = 2.0f;
	};
}