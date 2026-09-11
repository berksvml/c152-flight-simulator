// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "C152AircraftPawn.generated.h"

class UCameraComponent;
class UInputComponent;
class USceneComponent;
class USpringArmComponent;

UCLASS()
class C152FLIGHTSIM_API AC152AircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AC152AircraftPawn();

	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(
		UInputComponent* PlayerInputComponent) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	UPROPERTY(
		VisibleAnywhere,
		BlueprintReadOnly,
		Category = "Aircraft|Components",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USceneComponent> AircraftRoot;

	UPROPERTY(
		VisibleAnywhere,
		BlueprintReadOnly,
		Category = "Aircraft|Components",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USceneComponent> VisualRoot;

	UPROPERTY(
		VisibleAnywhere,
		BlueprintReadOnly,
		Category = "Aircraft|Camera",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USpringArmComponent> CameraBoom;

	UPROPERTY(
		VisibleAnywhere,
		BlueprintReadOnly,
		Category = "Aircraft|Camera",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UCameraComponent> ExternalCamera;
};
