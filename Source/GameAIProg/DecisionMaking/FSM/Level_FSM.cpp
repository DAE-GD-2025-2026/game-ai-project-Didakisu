// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "States/PatrolState.h"

#include "InputCoreTypes.h"

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
	
	//TODO
	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
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

			FSM->AddState(std::make_unique<GameAI::FSM::PatrolState>(Agent, PatrolPath));
			AIController->RunFiniteStateMachine();
		}
	}


	//spawn thief
	Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
	FVector{ -500, -500, 90 }, FRotator::ZeroRotator);

	if (Thief)
	{
		Thief->SetDebugRenderingEnabled(false);
	}
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Thief) return;

	if (APlayerController* playerController = GetWorld()->GetFirstPlayerController())
	{
		if (playerController->WasInputKeyJustPressed(EKeys::LeftMouseButton))
		{
			FHitResult Hit;
			playerController->GetHitResultUnderCursor(ECC_Visibility, false, Hit);
			if (Hit.bBlockingHit && Thief)
			{
				if (AAIController* ThiefAI = Cast<AAIController>(Thief->GetController()))
				{
					ThiefAI->MoveToLocation(Hit.Location);
				}
			}
		}
	}
}

