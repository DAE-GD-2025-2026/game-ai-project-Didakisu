// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "DecisionMaking/ThiefAIController.h"

#include "States/PatrolState.h"
#include "States/ChaseState.h"
#include "States/SearchState.h"
#include "States/WanderState.h"
#include "States/FleeState.h"

#include "BehaviorTree/BlackboardData.h"

#include "InputCoreTypes.h"

#include "Perception/AIPerceptionStimuliSourceComponent.h"
#include "Perception/AISense_Sight.h"

// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	UBlackboardData* BBAsset = LoadObject<UBlackboardData>(nullptr, TEXT("/Game/DecisionMaking/BB_TEST.BB_TEST"));

	//spawn thief first
	Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{ -500, -500, 90 }, FRotator::ZeroRotator);

	if (Thief)
	{
		//tag thief (for perception)
		Thief->Tags.Add(FName("Thief"));
		Thief->SetDebugRenderingEnabled(false);
		Thief->GetCharacterMovement()->MaxWalkSpeed = 500.f;

		UAIPerceptionStimuliSourceComponent* StimuliSource = NewObject<UAIPerceptionStimuliSourceComponent>(Thief);
		if (StimuliSource)
		{
			StimuliSource->RegisterComponent();
			StimuliSource->RegisterWithPerceptionSystem();
			StimuliSource->RegisterForSense(UAISense_Sight::StaticClass());
		}
	}

	//spawn guard
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,0,90 }, FRotator::ZeroRotator);

	if (Agent)
	{
		Agent->SetDebugRenderingEnabled(false);
		Agent->GetCharacterMovement()->MaxWalkSpeed = 300.f;

		UAIPerceptionStimuliSourceComponent* StimuliSource = NewObject<UAIPerceptionStimuliSourceComponent>(Agent);
		if (StimuliSource)
		{
			StimuliSource->RegisterComponent();
			StimuliSource->RegisterWithPerceptionSystem();
			StimuliSource->RegisterForSense(UAISense_Sight::StaticClass());
		}

		//guard controller  
		AGameAIController* GuardController = GetWorld()->SpawnActor<AGameAIController>(AGameAIController::StaticClass());

		if (GuardController)
		{
			if (BBAsset)
			{
				GuardController->FSMBlackboardAsset = BBAsset;
			}

			GuardController->SetTargetActor(Thief);
			GuardController->Possess(Agent);

			//guard FSM setup
			if (UFSMComponent* FSM = Cast<UFSMComponent>(GuardController->GetBrainComponent()))
			{
				TArray<FVector> PatrolPath =
				{
					FVector(100, 0, 90),
					FVector(400, 200, 90),
					FVector(300, 600, 90),
					FVector(-200, 400, 90)
				};

				for (const FVector& Point : PatrolPath)
				{
					DrawDebugSphere(GetWorld(), Point, 50.f, 12, FColor::Magenta, true, -1.f, 0, 5.f);
				}

				auto* patrolState = new GameAI::FSM::PatrolState(Agent, PatrolPath);
				auto* chaseState = new GameAI::FSM::ChaseState(Agent, Thief);
				auto* searchState = new GameAI::FSM::SearchState(Agent);

				searchState->Blackboard = GuardController->GetBlackboardComponent();

				FSM->AddState(std::unique_ptr<GameAI::FSM::State>(patrolState));
				FSM->AddState(std::unique_ptr<GameAI::FSM::State>(chaseState));
				FSM->AddState(std::unique_ptr<GameAI::FSM::State>(searchState));

				FSM->AddTransition(patrolState, chaseState, [GuardController]() -> bool {
					return GuardController && GuardController->CanSeeTarget();
					});

				FSM->AddTransition(chaseState, searchState, [GuardController, chaseState]() -> bool {
					return GuardController && !GuardController->CanSeeTarget() && chaseState->ChaseTimer >= chaseState->MinChaseTime;
					});

				FSM->AddTransition(searchState, chaseState, [GuardController]() -> bool {
					bool result = GuardController && GuardController->CanSeeTarget();
					return result;
					});

				FSM->AddTransition(searchState, patrolState, [searchState]() -> bool {
					return searchState->SearchTimer > searchState->MaxSearchingTime;
					});

				GuardController->RunFiniteStateMachine();
			}
		}
	}

	//thief controller
	AThiefAIController* ThiefController = GetWorld()->SpawnActor<AThiefAIController>(AThiefAIController::StaticClass());

	if (ThiefController)
	{
		ThiefController->Possess(Thief);
	}

	AThiefAIController* ThiefAIController = Cast<AThiefAIController>(Thief->GetController());

	if (ThiefAIController)
	{
		if (UFSMComponent* FSM = Cast<UFSMComponent>(ThiefAIController->GetBrainComponent()))
		{
			auto* wanderState = new GameAI::FSM::WanderState(Thief);
			auto* fleeState = new GameAI::FSM::FleeState(Thief, Agent);

			FSM->AddState(std::unique_ptr<GameAI::FSM::State>(wanderState));
			FSM->AddState(std::unique_ptr<GameAI::FSM::State>(fleeState));

			FSM->AddTransition(wanderState, fleeState, [ThiefAIController]() -> bool 
				{
					return ThiefAIController && ThiefAIController->IsGuardDetected();
				});

			FSM->AddTransition(fleeState, wanderState, [ThiefAIController]() -> bool 
				{
					return ThiefAIController && !ThiefAIController->IsGuardDetected();
				});

			ThiefAIController->RunFiniteStateMachine();
		}
	}
}

void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Thief) return;
}