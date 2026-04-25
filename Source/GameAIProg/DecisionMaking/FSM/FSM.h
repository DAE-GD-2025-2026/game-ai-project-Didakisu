#pragma once
#include <functional>
#include <vector>
#include <memory>

#include "BehaviorTree/BlackboardComponent.h"

namespace GameAI::FSM
{
	class State
	{
	public:
		virtual ~State() = default;

		virtual void OnEnter() {};
		virtual void OnExit() {};
		virtual void Update(float deltaTime) = 0;

		UBlackboardComponent* Blackboard{ nullptr };
	};

	class Transition
	{
	public:
		State* From{ nullptr };
		State* To{ nullptr }; //to what we will transition
		std::function<bool() > Condition; //AddTransition() requires a std::function
	};

	class FSM
	{
	public:
		void AddState(std::unique_ptr<State> state)
		{
			States.push_back(std::move(state));
		}

		void AddTransition(State* From, State* To, std::function<bool()> EvalFunc)
		{
			Transitions.push_back({ From,To, EvalFunc });
		}

		void SetInitialState(State* state)
		{
			CurrentState = state;
		}

		const std::vector<std::unique_ptr<State>>& GetStates()
		{
			return States;
		}

		State* GetInitialState()
		{
			return States[0].get();
		}

		void Update(float deltaTime)
		{
			CurrentState->Update(deltaTime);

			for (int i = 0; i < Transitions.size(); i++)
			{
				if (Transitions[i].From == CurrentState)
				{
					if (Transitions[i].Condition())
					{
						CurrentState->OnExit();
						CurrentState = Transitions[i].To;
						CurrentState->OnEnter();
						break;
					}
				}
			}
		}
	private:
		std::vector<std::unique_ptr<State>> States{};
		std::vector<Transition> Transitions{};
		State* CurrentState{ nullptr };
	};
}