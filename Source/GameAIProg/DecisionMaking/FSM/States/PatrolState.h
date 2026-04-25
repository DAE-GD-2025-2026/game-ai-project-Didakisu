#pragma once
#include "CoreMinimal.h"
#include "../FSM.h"

class ASteeringAgent;

namespace GameAI::FSM
{
	class PatrolState : public State
	{
	public:
		PatrolState(ASteeringAgent* Agent, TArray<FVector> Waypoints);

		void OnEnter() override;
		void OnExit() override;
		void Update(float deltaTime) override;
	private:
		TArray<FVector> WayPoints;
		int CurrentWaypointIndex{ 0 };
		ASteeringAgent* Agent{ nullptr };
		float threshold{ 50.f };
	};
}