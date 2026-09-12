
#include "C152AircraftPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/LocalPlayer.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputActionValue.h"

AC152AircraftPawn::AC152AircraftPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	AircraftRoot =
		CreateDefaultSubobject<USceneComponent>(TEXT("AircraftRoot"));
	SetRootComponent(AircraftRoot);

	VisualRoot =
		CreateDefaultSubobject<USceneComponent>(TEXT("VisualRoot"));
	VisualRoot->SetupAttachment(AircraftRoot);

	CameraBoom =
		CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(AircraftRoot);
	CameraBoom->TargetArmLength = 800.0f;
	CameraBoom->SetRelativeRotation(FRotator(-15.0f, 0.0f, 0.0f));
	CameraBoom->bDoCollisionTest = false;
	CameraBoom->bUsePawnControlRotation = false;

	ExternalCamera =
		CreateDefaultSubobject<UCameraComponent>(TEXT("ExternalCamera"));
	ExternalCamera->SetupAttachment(
		CameraBoom,
		USpringArmComponent::SocketName);
	ExternalCamera->bUsePawnControlRotation = false;

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AC152AircraftPawn::BeginPlay()
{
	Super::BeginPlay();

	UE_LOG(LogTemp, Log, TEXT("C152AircraftPawn initialized."));
}

void AC152AircraftPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	ControlInput.ThrottleCommand = FMath::Clamp(
		ControlInput.ThrottleCommand
		+ ThrottleRateCommand * ThrottleChangeRate * DeltaTime,
		0.0f,
		1.0f);

#if !UE_BUILD_SHIPPING
	if (bShowControlInputDebug && GEngine)
	{
		const FString DebugText = FString::Printf(
			TEXT("Pitch: %.2f | Roll: %.2f | Yaw: %.2f | Throttle: %.2f"),
			ControlInput.PitchCommand,
			ControlInput.RollCommand,
			ControlInput.YawCommand,
			ControlInput.ThrottleCommand);

		GEngine->AddOnScreenDebugMessage(
			static_cast<uint64>(GetUniqueID()),
			0.1f,
			FColor::Cyan,
			DebugText);
	}
#endif
}

void AC152AircraftPawn::PawnClientRestart()
{
	Super::PawnClientRestart();

	APlayerController* PlayerController =
		Cast<APlayerController>(GetController());

	if (!PlayerController || !AircraftMappingContext)
	{
		return;
	}

	ULocalPlayer* LocalPlayer = PlayerController->GetLocalPlayer();

	if (!LocalPlayer)
	{
		return;
	}

	UEnhancedInputLocalPlayerSubsystem* InputSubsystem =
		LocalPlayer->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>();

	if (InputSubsystem)
	{
		InputSubsystem->RemoveMappingContext(AircraftMappingContext);
		InputSubsystem->AddMappingContext(AircraftMappingContext, 0);
	}
}

void AC152AircraftPawn::SetupPlayerInputComponent(
	UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	UEnhancedInputComponent* EnhancedInputComponent =
		Cast<UEnhancedInputComponent>(PlayerInputComponent);

	if (!EnhancedInputComponent)
	{
		UE_LOG(
			LogTemp,
			Error,
			TEXT("Enhanced Input Component is not available."));
		return;
	}

	if (PitchAction)
	{
		EnhancedInputComponent->BindAction(
			PitchAction,
			ETriggerEvent::Triggered,
			this,
			&AC152AircraftPawn::HandlePitchInput);

		EnhancedInputComponent->BindAction(
			PitchAction,
			ETriggerEvent::Completed,
			this,
			&AC152AircraftPawn::ResetPitchInput);

		EnhancedInputComponent->BindAction(
			PitchAction,
			ETriggerEvent::Canceled,
			this,
			&AC152AircraftPawn::ResetPitchInput);
	}

	if (RollAction)
	{
		EnhancedInputComponent->BindAction(
			RollAction,
			ETriggerEvent::Triggered,
			this,
			&AC152AircraftPawn::HandleRollInput);

		EnhancedInputComponent->BindAction(
			RollAction,
			ETriggerEvent::Completed,
			this,
			&AC152AircraftPawn::ResetRollInput);

		EnhancedInputComponent->BindAction(
			RollAction,
			ETriggerEvent::Canceled,
			this,
			&AC152AircraftPawn::ResetRollInput);
	}

	if (YawAction)
	{
		EnhancedInputComponent->BindAction(
			YawAction,
			ETriggerEvent::Triggered,
			this,
			&AC152AircraftPawn::HandleYawInput);

		EnhancedInputComponent->BindAction(
			YawAction,
			ETriggerEvent::Completed,
			this,
			&AC152AircraftPawn::ResetYawInput);

		EnhancedInputComponent->BindAction(
			YawAction,
			ETriggerEvent::Canceled,
			this,
			&AC152AircraftPawn::ResetYawInput);
	}

	if (ThrottleRateAction)
	{
		EnhancedInputComponent->BindAction(
			ThrottleRateAction,
			ETriggerEvent::Triggered,
			this,
			&AC152AircraftPawn::HandleThrottleRateInput);

		EnhancedInputComponent->BindAction(
			ThrottleRateAction,
			ETriggerEvent::Completed,
			this,
			&AC152AircraftPawn::ResetThrottleRateInput);

		EnhancedInputComponent->BindAction(
			ThrottleRateAction,
			ETriggerEvent::Canceled,
			this,
			&AC152AircraftPawn::ResetThrottleRateInput);
	}
}

void AC152AircraftPawn::HandlePitchInput(
	const FInputActionValue& Value)
{
	ControlInput.PitchCommand =
		FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void AC152AircraftPawn::HandleRollInput(
	const FInputActionValue& Value)
{
	ControlInput.RollCommand =
		FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void AC152AircraftPawn::HandleYawInput(
	const FInputActionValue& Value)
{
	ControlInput.YawCommand =
		FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void AC152AircraftPawn::HandleThrottleRateInput(
	const FInputActionValue& Value)
{
	ThrottleRateCommand =
		FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
}

void AC152AircraftPawn::ResetPitchInput(
	const FInputActionValue& Value)
{
	ControlInput.PitchCommand = 0.0f;
}

void AC152AircraftPawn::ResetRollInput(
	const FInputActionValue& Value)
{
	ControlInput.RollCommand = 0.0f;
}

void AC152AircraftPawn::ResetYawInput(
	const FInputActionValue& Value)
{
	ControlInput.YawCommand = 0.0f;
}

void AC152AircraftPawn::ResetThrottleRateInput(
	const FInputActionValue& Value)
{
	ThrottleRateCommand = 0.0f;
}
