#include "ThiefAIController.h"
#include "Perception/AIPerceptionStimuliSourceComponent.h"

AThiefAIController::AThiefAIController()
{
	AIPerceptionComponent = CreateDefaultSubobject<UAIPerceptionComponent>(TEXT("AIPerception"));
	UAISenseConfig_Sight* SightConfig = CreateDefaultSubobject<UAISenseConfig_Sight>(TEXT("SightConfig"));

	if (SightConfig)
	{
		SightConfig->SightRadius = 1000.f;
		SightConfig->LoseSightRadius = 1200.f;

		//FOV
		SightConfig->PeripheralVisionAngleDegrees = 90.f;
		SightConfig->SetMaxAge(2.0f);

		SightConfig->DetectionByAffiliation.bDetectEnemies = true;
		SightConfig->DetectionByAffiliation.bDetectFriendlies = true;
		SightConfig->DetectionByAffiliation.bDetectNeutrals = true;


		AIPerceptionComponent->ConfigureSense(*SightConfig);

		AIPerceptionComponent->SetDominantSense(SightConfig->GetSenseImplementation());
	}
}

void AThiefAIController::BeginPlay()
{
	Super::BeginPlay();

	GetWorldTimerManager().SetTimer(MoveTimer, this, &AThiefAIController::MoveRandomly, 1.5f, true);

	//bind perception
	if (AIPerceptionComponent)
	{
		AIPerceptionComponent->OnTargetPerceptionUpdated.AddDynamic(this, &AThiefAIController::OnTargetPerceptionUpdated);
	}
}

void AThiefAIController::MoveRandomly()
{
	APawn* ControlledPawn = GetPawn();
	if (!ControlledPawn)
	{
		return;
	}

	FVector Origin = ControlledPawn->GetActorLocation();

	FVector RandomOffset = FVector(FMath::RandRange(-800.f, 800.f), FMath::RandRange(-800.f, 800.f), 0.f);

	FVector Destination = Origin + RandomOffset;
	MoveToLocation(Destination);
}

void AThiefAIController::OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus)
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
		bIsGuardDetected = true;
	}
	else
	{
		bIsGuardDetected = false;
	}
}
