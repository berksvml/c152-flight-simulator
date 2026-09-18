#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "FlightDynamics/C152Simulation.h"
#include "C152AircraftPawn.generated.h"

class UCameraComponent;
class UInputAction;
class UInputComponent;
class UInputMappingContext;
class USceneComponent;
class USpringArmComponent;
struct FInputActionValue;

USTRUCT(BlueprintType)
struct C152FLIGHTSIM_API FAircraftControlInput
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Input")
	float PitchCommand = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Input")
	float RollCommand = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Input")
	float YawCommand = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Input")
	float ThrottleCommand = 0.0f;
};

UCLASS()
class C152FLIGHTSIM_API AC152AircraftPawn : public APawn
{
	GENERATED_BODY()

public:
	AC152AircraftPawn();

	virtual void Tick(float DeltaTime) override;

	virtual void SetupPlayerInputComponent(
		UInputComponent* PlayerInputComponent) override;

	virtual void PawnClientRestart() override;

protected:
	virtual void BeginPlay() override;

private:
	void HandlePitchInput(const FInputActionValue& Value);
	void HandleRollInput(const FInputActionValue& Value);
	void HandleYawInput(const FInputActionValue& Value);
	void HandleThrottleRateInput(const FInputActionValue& Value);

	void ResetPitchInput(const FInputActionValue& Value);
	void ResetRollInput(const FInputActionValue& Value);
	void ResetYawInput(const FInputActionValue& Value);
	void ResetThrottleRateInput(const FInputActionValue& Value);

	void InitializeSimulationFromActorTransform();
	void ApplyAircraftStateToActorTransform();

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

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "Aircraft|Input",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UInputMappingContext> AircraftMappingContext;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "Aircraft|Input",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UInputAction> PitchAction;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "Aircraft|Input",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UInputAction> RollAction;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "Aircraft|Input",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UInputAction> YawAction;

	UPROPERTY(
		EditDefaultsOnly,
		BlueprintReadOnly,
		Category = "Aircraft|Input",
		meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UInputAction> ThrottleRateAction;

	UPROPERTY(
		VisibleAnywhere,
		BlueprintReadOnly,
		Category = "Aircraft|Input",
		meta = (AllowPrivateAccess = "true"))
	FAircraftControlInput ControlInput;

	UPROPERTY(
		EditDefaultsOnly,
		Category = "Aircraft|Input",
		meta = (ClampMin = "0.0"))
	float ThrottleChangeRate = 0.25f;

	UPROPERTY(
		EditDefaultsOnly,
		Category = "Aircraft|Debug")
	bool bShowControlInputDebug = true;

	float ThrottleRateCommand = 0.0f;

	C152::FlightDynamics::FC152Simulation Simulation;
};