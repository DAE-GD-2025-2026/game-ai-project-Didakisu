// Fill out your copyright notice in the Description page of Project Settings.


#include "GameAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
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
		SightConfig->SightRadius = SightRadius;

		//at what distance does the guard loses sight
		SightConfig->LoseSightRadius = LoseSightRadius;

		//FOV
		SightConfig->PeripheralVisionAngleDegrees = 90.f;

		SightConfig->SetMaxAge(5.0f);
		//SightConfig->AutoSuccessRangeFromLastSeenLocation = 50.f;

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
	
	//bind perception
	if (AIPerceptionComponent)
	{
		AIPerceptionComponent->OnTargetPerceptionUpdated.AddDynamic(this, &AGameAIController::OnTargetPerceptionUpdated);
	}
}

void AGameAIController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);
	InitFiniteStateMachine();
}

// Called every frame
void AGameAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!GetPawn())
	{
		return;
	}

	const FVector Center = GetPawn()->GetActorLocation();
	FVector yAxis = FVector(1, 0, 0);
	FVector zAxis = FVector(0, 1, 0);

	DrawDebugCircle(GetWorld(), Center, SightRadius ,32 ,FColor::Green ,false , -1.0f, 0 ,0.f, yAxis, zAxis);
	DrawDebugCircle(GetWorld(), Center, LoseSightRadius, 32, FColor::Red, false, -1.0f , 0, 0.f, yAxis, zAxis);
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
	if (!Actor) return;
	if (!Actor->ActorHasTag(FName("Thief"))) return;
	if (GetPawn() && Actor == GetPawn()) return;

	if (Stimulus.WasSuccessfullySensed())
	{
		CurrentTarget = Actor;
		if (Blackboard)
		{
			Blackboard->SetValueAsBool("CanSeeTarget", true); 
			Blackboard->SetValueAsVector("LastKnownPos", Actor->GetActorLocation());
		}
	}
	else
	{
		if (CurrentTarget == Actor)
		{
			if (Blackboard)
			{
				Blackboard->SetValueAsVector("LastKnownPos", Actor->GetActorLocation());
				Blackboard->SetValueAsBool("CanSeeTarget", false);
			}
			CurrentTarget = nullptr;
		}
	}
}

FORCEINLINE bool AGameAIController::CanSeeTarget() const
{
	return Blackboard && Blackboard->GetValueAsBool("CanSeeTarget");
}