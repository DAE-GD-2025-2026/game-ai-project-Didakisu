// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "ThiefAIController.generated.h"


UCLASS()
class GAMEAIPROG_API AThiefAIController : public AAIController
{
    GENERATED_BODY()

public:
    AThiefAIController();

    virtual void BeginPlay() override;
private:
    void MoveRandomly();
    FTimerHandle MoveTimer;
};