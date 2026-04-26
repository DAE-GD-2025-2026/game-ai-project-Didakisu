#pragma once
#include "CoreMinimal.h"
#include "../FSM.h"

class ASteeringAgent;

namespace GameAI::FSM
{
	class ChaseState : public State
	{
	public:
		ChaseState(ASteeringAgent* Agent, ASteeringAgent* Target);

		void OnEnter() override;
		void OnExit() override;
		void Update(float deltaTime) override;
	private:
		ASteeringAgent* Agent{ nullptr };
		ASteeringAgent* Target{ nullptr };
	};
}