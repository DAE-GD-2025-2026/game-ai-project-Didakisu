// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "DecisionMaking/ThiefAIController.h"

#include "States/PatrolState.h"
#include "States/ChaseState.h"
#include "States/SearchState.h"

#include "InputCoreTypes.h"

#include "Perception/AIPerceptionStimuliSourceComponent.h"
#include "Perception/AISense_Sight.h"

// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();
	
	//spawn guard
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);

	if (Agent)
	{
		Agent->GetCharacterMovement()->MaxWalkSpeed = 300.f;
	}

	//spawn thief
	Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{ -500, -500, 90 }, FRotator::ZeroRotator);

	if (Thief)
	{
		Thief->SetDebugRenderingEnabled(false);
		Thief->GetCharacterMovement()->MaxWalkSpeed = 600.f;

		UAIPerceptionStimuliSourceComponent* StimuliSource = NewObject<UAIPerceptionStimuliSourceComponent>(Thief);
		StimuliSource->RegisterComponent();
		StimuliSource->RegisterWithPerceptionSystem();
		StimuliSource->RegisterForSense(UAISense_Sight::StaticClass());
		//
		AThiefAIController* ThiefController = GetWorld()->SpawnActor<AThiefAIController>(AThiefAIController::StaticClass());

		if (ThiefController)
		{
			ThiefController->Possess(Thief);
		}
	}
	
	AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController());

	if (AIController)
	{
		if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
		{

			TArray<FVector> PatrolPath =
			{
				FVector(100, 0, 90),
				FVector(400, 200, 90),
				FVector(300, 600, 90),
				FVector(-200, 400, 90)
			};

			//debug
			for (const FVector& Point : PatrolPath)
			{
				DrawDebugSphere(GetWorld(), Point, 50.f, 12, FColor::Magenta, true, -1.f, 0, 5.f);
			}

			auto* patrolState = new GameAI::FSM::PatrolState(Agent, PatrolPath);
			auto* chaseState = new GameAI::FSM::ChaseState(Agent, Thief);
			auto* searchState = new GameAI::FSM::SearchState(Agent);

			FSM->AddState(std::unique_ptr<GameAI::FSM::State>(patrolState));
			FSM->AddState(std::unique_ptr<GameAI::FSM::State>(chaseState));
			FSM->AddState(std::unique_ptr<GameAI::FSM::State>(searchState));

			FSM->AddTransition(patrolState, chaseState, [AIController]() -> bool {
				//return IsTargetVisible();
				return AIController && AIController->CanSeeTarget();
				});

			FSM->AddTransition(chaseState, searchState, [AIController]() -> bool {
				return AIController && !AIController->CanSeeTarget();
				});

			FSM->AddTransition(searchState, chaseState, [AIController]() -> bool {
				return AIController && AIController->CanSeeTarget();
				});

			FSM->AddTransition(searchState, patrolState, [searchState]()-> bool {
				return searchState->SearchTimer > searchState->MaxSearchingTime;
				});


			AIController->RunFiniteStateMachine();
		}
	}
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Thief) return;
}

