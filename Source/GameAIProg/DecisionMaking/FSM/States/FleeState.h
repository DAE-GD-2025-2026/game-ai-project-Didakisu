#pragma once
#include "CoreMinimal.h"
#include "../FSM.h"

class ASteeringAgent;

namespace GameAI::FSM
{
	class FleeState : public State
	{
	public:
		FleeState(ASteeringAgent* Agent, ASteeringAgent* Guard);

		void OnEnter() override;
		void OnExit() override;
		void Update(float deltaTime) override;

	private:
		ASteeringAgent* Agent{ nullptr };
		ASteeringAgent* Guard{ nullptr };
	};
}