#include "ThiefAIController.h"

AThiefAIController::AThiefAIController()
{
	
}

void AThiefAIController::BeginPlay()
{
	Super::BeginPlay();

	GetWorldTimerManager().SetTimer(MoveTimer, this, &AThiefAIController::MoveRandomly, 1.5f, true);
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