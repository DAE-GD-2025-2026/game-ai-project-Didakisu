// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISense_Sight.h"
#include "Perception/AISenseConfig_Sight.h"

#include "ThiefAIController.generated.h"


UCLASS()
class GAMEAIPROG_API AThiefAIController : public AAIController
{
    GENERATED_BODY()

public:
    AThiefAIController();

    virtual void BeginPlay() override;
    UFUNCTION()
    void OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus);

	FORCEINLINE bool IsGuardDetected() const { return bIsGuardDetected; }
private:
    void MoveRandomly();
    FTimerHandle MoveTimer;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "AI", meta = (AllowPrivateAccess = "true"))
    UAIPerceptionComponent* AIPerceptionComponent;

    bool bIsGuardDetected = false;
};