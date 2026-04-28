// Fill out your copyright notice in the Description page of Project Settings.


#include "GameAIController.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "FSM/FSMComponent.h"


// Sets default values
AGameAIController::AGameAIController()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	BrainComponent = CreateDefaultSubobject<UFSMComponent>(TEXT("FSMComponent"));;

	AIPerceptionComponent = CreateDefaultSubobject<UAIPerceptionComponent>(TEXT("AIPerception"));
	UAISenseConfig_Sight* SightConfig = CreateDefaultSubobject<UAISenseConfig_Sight>(TEXT("SightConfig"));

	if (SightConfig)
	{
		//how far the guard sees
		SightConfig->SightRadius = 800.f;

		//at what distance does the guard loses sight
		SightConfig->LoseSightRadius = 1000.f;

		//FOV
		SightConfig->PeripheralVisionAngleDegrees = 90.f;

		SightConfig->SetMaxAge(2.0f);
		SightConfig->AutoSuccessRangeFromLastSeenLocation = 50.f;

		SightConfig->DetectionByAffiliation.bDetectEnemies = true;
		SightConfig->DetectionByAffiliation.bDetectFriendlies = true;
		SightConfig->DetectionByAffiliation.bDetectNeutrals = true;


		AIPerceptionComponent->ConfigureSense(*SightConfig);

		AIPerceptionComponent->SetDominantSense(SightConfig->GetSenseImplementation());
	}
}

// Called when the game starts or when spawned
void AGameAIController::BeginPlay()
{
	Super::BeginPlay();
	
	// Create Blackboard if need be
	InitFiniteStateMachine();

	if (AIPerceptionComponent)
	{
		AIPerceptionComponent->OnTargetPerceptionUpdated.AddDynamic(this, &AGameAIController::OnTargetPerceptionUpdated);
	}
}

// Called every frame
void AGameAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AGameAIController::InitFiniteStateMachine()
{
	UFSMComponent* FSMComp = FindComponentByClass<UFSMComponent>();
	if (ensure(FSMComp) && FSMBlackboardAsset)
	{
		UBlackboardComponent* BlackboardComp = Blackboard;
		UseBlackboard(FSMBlackboardAsset, BlackboardComp);
		Blackboard = BlackboardComp;
	}
}

void AGameAIController::RunFiniteStateMachine()
{
	UFSMComponent* FSMComp = FindComponentByClass<UFSMComponent>();
	if (ensure(FSMComp))
	{
		FSMComp->StartLogic();
	}
}

void AGameAIController::OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus)
{
	if (!Actor)
	{
		return;
	}

	if (Actor == GetPawn())
	{
		return;
	}

	if (Stimulus.WasSuccessfullySensed())
	{
		CurrentTarget = Actor;
	}
	else
	{
		if (CurrentTarget == Actor)
		{
			CurrentTarget = nullptr;
		}
	}
}