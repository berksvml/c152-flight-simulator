
#include "C152AircraftPawn.h"

#include "Integration/UnrealAircraftStateAdapter.h"

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

	InitializeSimulationFromActorTransform();

	UE_LOG(LogTemp, Log, TEXT("C152AircraftPawn initialized."));
}

void AC152AircraftPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	ControlInput.ThrottleCommand = FMath::Clamp(
		ControlInput.ThrottleCommand
		+ ThrottleRateCommand
		* ThrottleChangeRate
		* DeltaTime,
		0.0f,
		1.0f);

	C152::FlightDynamics::FControlCommand Command{};

	Command.Pitch =
		static_cast<double>(ControlInput.PitchCommand);

	Command.Roll =
		static_cast<double>(ControlInput.RollCommand);

	Command.Yaw =
		static_cast<double>(ControlInput.YawCommand);

	Command.Throttle =
		static_cast<double>(ControlInput.ThrottleCommand);

	const std::uint32_t SimulationStepCount =
		Simulation.Advance(
			Command,
			static_cast<double>(DeltaTime));

	const bool bDynamicsStepSuccessful =
		Simulation.WasLastDynamicsStepSuccessful();

	if (!bDynamicsStepSuccessful
		&& !bDynamicsFailureReported)
	{
		UE_LOG(
			LogTemp,
			Error,
			TEXT("C152 rigid-body propagation failed."));

		bDynamicsFailureReported = true;
	}
	else if (bDynamicsStepSuccessful)
	{
		bDynamicsFailureReported = false;
	}

	ApplyAircraftStateToActorTransform();

	const double FixedDeltaSeconds =
		Simulation.GetFixedDeltaSeconds();

	const C152::FlightDynamics::FControlSurfaceState&
		SurfaceState =
		Simulation.GetControlSurfaceState();

	const C152::FlightDynamics::FAircraftState&
		AircraftState =
		Simulation.GetAircraftState();

#if !UE_BUILD_SHIPPING
	if (bShowControlInputDebug && GEngine)
	{
		const TCHAR* DynamicsMode =
			Simulation.IsDynamicsConfigured()
			? TEXT("Enabled")
			: TEXT("Disabled");

		const TCHAR* DynamicsStatus =
			bDynamicsStepSuccessful
			? TEXT("OK")
			: TEXT("FAILED");

		const FString DebugText = FString::Printf(
			TEXT(
				"Simulation Steps: %u | Fixed dt: %.3f ms\n"
				"Dynamics: %s | Status: %s\n"
				"Input  P: %.2f | R: %.2f | Y: %.2f | T: %.2f\n"
				"Surface  Elevator: %.1f deg | Aileron: %.1f deg | "
				"Rudder: %.1f deg\n"
				"Position NED  N: %.2f m | E: %.2f m | D: %.2f m\n"
				"Velocity Body  U: %.2f m/s | V: %.2f m/s | "
				"W: %.2f m/s"),
			static_cast<unsigned int>(SimulationStepCount),
			FixedDeltaSeconds * 1000.0,
			DynamicsMode,
			DynamicsStatus,
			ControlInput.PitchCommand,
			ControlInput.RollCommand,
			ControlInput.YawCommand,
			ControlInput.ThrottleCommand,
			FMath::RadiansToDegrees(SurfaceState.ElevatorRad),
			FMath::RadiansToDegrees(SurfaceState.AileronRad),
			FMath::RadiansToDegrees(SurfaceState.RudderRad),
			AircraftState.PositionNedMeters.X,
			AircraftState.PositionNedMeters.Y,
			AircraftState.PositionNedMeters.Z,
			AircraftState.VelocityBodyMetersPerSecond.X,
			AircraftState.VelocityBodyMetersPerSecond.Y,
			AircraftState.VelocityBodyMetersPerSecond.Z);

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

void AC152AircraftPawn::
InitializeSimulationFromActorTransform()
{
	using namespace C152::FlightDynamics;

	FAircraftState InitialAircraftState{};

	C152::UnrealIntegration::FUnrealAircraftStateAdapter::
		UpdateCorePoseFromUnrealTransform(
			GetActorTransform(),
			InitialAircraftState);

	if (bEnablePhaseOneDynamicsTest)
	{
		InitialAircraftState
			.VelocityBodyMetersPerSecond.X =
			static_cast<double>(
				FMath::Max(
					PhaseOneInitialForwardSpeedMetersPerSecond,
					0.0f));
	}

	Simulation.ClearDynamicsConfiguration();
	Simulation.Reset(InitialAircraftState);

	if (bEnablePhaseOneDynamicsTest)
	{
		const bool bConfigurationSucceeded =
			ConfigurePhaseOneDynamicsTest();

		if (!bConfigurationSucceeded)
		{
			UE_LOG(
				LogTemp,
				Error,
				TEXT(
					"Failed to configure the Phase 1 "
					"dynamics test."));
		}
	}

	bDynamicsFailureReported = false;
}

void AC152AircraftPawn::ApplyAircraftStateToActorTransform()
{
	const FTransform TargetTransform =
		C152::UnrealIntegration::FUnrealAircraftStateAdapter::
		ToUnrealTransform(
			Simulation.GetAircraftState());

	SetActorLocationAndRotation(
		TargetTransform.GetLocation(),
		TargetTransform.GetRotation(),
		false,
		nullptr,
		ETeleportType::TeleportPhysics);
}

bool AC152AircraftPawn::
ConfigurePhaseOneDynamicsTest()
{
	using namespace C152::FlightDynamics;

	FC152SimulationConfiguration Configuration;

	// Synthetic values used only for the Phase 1 runtime test.
	// Validated C152 data will be introduced separately.
	Configuration.MassProperties.MassKilograms = 1.0;

	Configuration.MassProperties
		.InertiaXxKilogramMetersSquared = 1.0;

	Configuration.MassProperties
		.InertiaYyKilogramMetersSquared = 1.0;

	Configuration.MassProperties
		.InertiaZzKilogramMetersSquared = 1.0;

	Configuration.MassProperties
		.ProductOfInertiaXzKilogramMetersSquared = 0.0;

	// Gravity remains disabled until lift and ground-contact
	// models are available.
	Configuration
		.GravityAccelerationNedMetersPerSecondSquared = {
			0.0,
			0.0,
			0.0
	};

	return Simulation.SetDynamicsConfiguration(
		Configuration);
}