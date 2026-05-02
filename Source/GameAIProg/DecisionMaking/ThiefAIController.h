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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AI|FSM")
    TObjectPtr<UBlackboardData> FSMBlackboardAsset;

    AThiefAIController();

    UFUNCTION()
    void OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus);
    void RunFiniteStateMachine();
	FORCEINLINE bool IsGuardDetected() const { return CurrentTarget != nullptr; }
protected:
    virtual void BeginPlay() override;
    void InitFiniteStateMachine();
private:
    //void MoveRandomly();
    FTimerHandle MoveTimer;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "AI", meta = (AllowPrivateAccess = "true"))
    UAIPerceptionComponent* AIPerceptionComponent;

    AActor* CurrentTarget;
};