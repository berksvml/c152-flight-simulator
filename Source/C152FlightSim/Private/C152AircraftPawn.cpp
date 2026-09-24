
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
#include "FlightDynamics/C152AircraftConfiguration.h"
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

	const bool bBrakeCommandAccepted =
		Simulation.SetBrakeCommand(
			static_cast<double>(
				ControlInput.BrakeCommand));

	if (!bBrakeCommandAccepted)
	{
		UE_LOG(
			LogTemp,
			Error,
			TEXT("Invalid C152 brake command."));
	}

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
			TEXT("C152 integrated simulation step failed."));

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

		FString DebugText = FString::Printf(
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

		C152::FlightDynamics::FAtmosphereState AtmosphereSample{};
		C152::FlightDynamics::FAirData AirDataSample{};

		if (Simulation.TryGetEnvironmentSample(
			AtmosphereSample,
			AirDataSample))
		{
			const double AltitudeMeters =
				WorldOriginGeopotentialAltitudeMeters
				- AircraftState.PositionNedMeters.Z;

			DebugText += FString::Printf(
				TEXT(
					"\nAtmosphere  Alt: %.1f m | Density: %.3f kg/m^3"
					"\nAir data  TAS: %.2f m/s | Alpha: %.1f deg | "
					"Beta: %.1f deg | q: %.1f Pa | Mach: %.3f"),
				AltitudeMeters,
				AtmosphereSample.DensityKilogramsPerCubicMeter,
				AirDataSample.TrueAirspeedMetersPerSecond,
				FMath::RadiansToDegrees(
					AirDataSample.AngleOfAttackRadians),
				FMath::RadiansToDegrees(
					AirDataSample.SideslipAngleRadians),
				AirDataSample.DynamicPressurePascals,
				AirDataSample.MachNumber);

			const C152::FlightDynamics::FVector3& CurrentWind =
				Simulation.GetWindVelocityNedMetersPerSecond();

			const C152::FlightDynamics::FVector3& CurrentTurbulence =
				Simulation.GetTurbulenceVelocityNedMetersPerSecond();

			DebugText += FString::Printf(
				TEXT(
					"\nWind NED  N: %.2f | E: %.2f | D: %.2f m/s"
					"\nTurbulence NED  N: %.2f | E: %.2f | D: %.2f m/s"),
				CurrentWind.X,
				CurrentWind.Y,
				CurrentWind.Z,
				CurrentTurbulence.X,
				CurrentTurbulence.Y,
				CurrentTurbulence.Z);
		}
		else
		{
			DebugText += TEXT(
				"\nAir data unavailable: check origin altitude "
				"and aircraft position.");
		}

		const C152::FlightDynamics::FAircraftMassProperties&
			CurrentMassProperties =
			Simulation.GetCurrentMassProperties();

		DebugText += FString::Printf(
			TEXT(
				"\nAircraft mass: %.3f kg | Brake: %.2f"),
			CurrentMassProperties.MassKilograms,
			Simulation.GetBrakeCommand());

		const C152::FlightDynamics::FC152AircraftModelStepOutput&
			ModelOutput =
			Simulation.GetLastAircraftModelStepOutput();

		if (ModelOutput.bValid)
		{
			const TCHAR* GroundStatus =
				ModelOutput.GroundReaction.bOnGround
				? TEXT("ON GROUND")
				: TEXT("AIRBORNE");

			DebugText += FString::Printf(
				TEXT(
					"\nFuel: %.3f kg | Flow: %.6f kg/s | "
					"Power: %s"
					"\nGround: %s | Contacts: %d | Normal: %.1f N"
					"\nLoads X  Aero: %.1f N | Engine: %.1f N"
					"\nLoads Z  Aero: %.1f N | Ground: %.1f N"),
				ModelOutput.Powerplant.Fuel
				.RemainingUsableFuelKilograms,
				ModelOutput.Powerplant.Fuel
				.FuelFlowKilogramsPerSecond,
				ModelOutput.Powerplant.bProducingPower
				? TEXT("ON")
				: TEXT("OFF"),
				GroundStatus,
				ModelOutput.GroundReaction.ActiveContactCount,
				ModelOutput.GroundReaction.TotalNormalForceNewtons,
				ModelOutput.AerodynamicLoads
				.ForceBodyNewtons.X,
				ModelOutput.PowerplantLoads
				.ForceBodyNewtons.X,
				ModelOutput.AerodynamicLoads
				.ForceBodyNewtons.Z,
				ModelOutput.GroundReactionLoads
				.ForceBodyNewtons.Z);
		}

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

	if (BrakeAction)
	{
		EnhancedInputComponent->BindAction(
			BrakeAction,
			ETriggerEvent::Triggered,
			this,
			&AC152AircraftPawn::HandleBrakeInput);

		EnhancedInputComponent->BindAction(
			BrakeAction,
			ETriggerEvent::Completed,
			this,
			&AC152AircraftPawn::ResetBrakeInput);

		EnhancedInputComponent->BindAction(
			BrakeAction,
			ETriggerEvent::Canceled,
			this,
			&AC152AircraftPawn::ResetBrakeInput);
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

void AC152AircraftPawn::HandleBrakeInput(
	const FInputActionValue& Value)
{
	ControlInput.BrakeCommand =
		FMath::Clamp(
			Value.Get<float>(),
			0.0f,
			1.0f);
}

void AC152AircraftPawn::ResetBrakeInput(
	const FInputActionValue& Value)
{
	ControlInput.BrakeCommand = 0.0f;
}

void AC152AircraftPawn::
InitializeSimulationFromActorTransform()
{
	using namespace C152::FlightDynamics;

	FTransform InitialTransform =
		GetActorTransform();

	if (bStartOnRunway)
	{
		FRotator LevelRotation =
			InitialTransform.Rotator();

		LevelRotation.Pitch = 0.0;
		LevelRotation.Roll = 0.0;

		InitialTransform.SetRotation(
			LevelRotation.Quaternion());
	}

	FAircraftState InitialAircraftState{};

	C152::UnrealIntegration::FUnrealAircraftStateAdapter::
		UpdateCorePoseFromUnrealTransform(
			InitialTransform,
			InitialAircraftState);

	Simulation.ClearAircraftModelConfiguration();
	Simulation.ClearDynamicsConfiguration();
	Simulation.ClearEnvironmentConfiguration();

	bool bConfigurationSucceeded = true;

	if (bEnableIntegratedAircraftSimulation)
	{
		bConfigurationSucceeded =
			ConfigureIntegratedAircraftSimulation(
				InitialAircraftState);
	}
	else
	{
		InitialAircraftState
			.VelocityBodyMetersPerSecond.X =
			FMath::Max(
				InitialForwardSpeedMetersPerSecond,
				0.0);

		Simulation.Reset(InitialAircraftState);
	}

	if (!bConfigurationSucceeded)
	{
		UE_LOG(
			LogTemp,
			Error,
			TEXT(
				"Failed to configure the integrated "
				"C152 aircraft simulation."));
	}
	else
	{
		ApplyAircraftStateToActorTransform();

		UE_LOG(
			LogTemp,
			Log,
			TEXT(
				"Integrated C152 aircraft simulation "
				"configured successfully."));
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
ConfigureIntegratedAircraftSimulation(
	C152::FlightDynamics::FAircraftState
	InitialAircraftState)
{
	using namespace C152::FlightDynamics;

	const FC152AircraftConfiguration
		AircraftConfiguration =
		FC152AircraftConfiguration::
		Create1979Model152();

	if (!AircraftConfiguration.IsValid())
	{
		return false;
	}

	FPowerplantModelConfiguration
		PowerplantConfiguration{};

	if (!AircraftConfiguration
		.TryGetDevelopmentPowerplantConfiguration(
			PowerplantConfiguration))
	{
		return false;
	}

	FC152SimulationConfiguration
		DynamicsConfiguration{};

	DynamicsConfiguration.MassProperties =
		AircraftConfiguration
		.DevelopmentMassPropertiesEstimate;

	DynamicsConfiguration
		.GravityAccelerationNedMetersPerSecondSquared =
		FVector3{
			0.0,
			0.0,
			9.80665
	};

	FEnvironmentConfiguration
		EnvironmentConfiguration{};

	EnvironmentConfiguration
		.OriginGeopotentialAltitudeMeters =
		WorldOriginGeopotentialAltitudeMeters;

	EnvironmentConfiguration
		.WindVelocityNedMetersPerSecond =
		FVector3{
			WindVelocityNedMetersPerSecond.X,
			WindVelocityNedMetersPerSecond.Y,
			WindVelocityNedMetersPerSecond.Z
	};

	EnvironmentConfiguration
		.TurbulenceStandardDeviationNedMetersPerSecond =
		FVector3{
			TurbulenceStandardDeviationNedMetersPerSecond.X,
			TurbulenceStandardDeviationNedMetersPerSecond.Y,
			TurbulenceStandardDeviationNedMetersPerSecond.Z
	};

	EnvironmentConfiguration
		.TurbulenceCorrelationTimeSeconds =
		TurbulenceCorrelationTimeSeconds;

	EnvironmentConfiguration.TurbulenceRandomSeed =
		static_cast<std::uint32_t>(
			FMath::Max(
				TurbulenceRandomSeed,
				0));

	FC152AircraftModelConfiguration
		AircraftModelConfiguration{};

	AircraftModelConfiguration.Aerodynamics =
		AircraftConfiguration
		.DevelopmentAerodynamicEstimate;

	AircraftModelConfiguration.Powerplant =
		PowerplantConfiguration;

	AircraftModelConfiguration.GroundReaction =
		AircraftConfiguration
		.DevelopmentGroundReactionEstimate;

	const double ClampedFuelFraction =
		FMath::Clamp(
			InitialUsableFuelFraction,
			0.0,
			1.0);

	AircraftModelConfiguration
		.InitialUsableFuelKilograms =
		ClampedFuelFraction
		* AircraftConfiguration
		.DevelopmentFuelEstimate
		.UsableFuelCapacityKilograms;

	// Unreal uses Z-up centimeters.
	// Core runway position uses NED Down meters.
	AircraftModelConfiguration
		.GroundPlaneDownMeters =
		-RunwayWorldZCentimeters / 100.0;

	if (bStartOnRunway)
	{
		const FGroundContactPointConfiguration&
			NoseGear =
			AircraftModelConfiguration
			.GroundReaction.ContactPoints[0];

		const double CompressionMeters =
			FMath::Clamp(
				InitialLandingGearCompressionMeters,
				0.0,
				0.10);

		// Position the aircraft so the landing-gear contacts
		// begin with a small spring compression.
		InitialAircraftState.PositionNedMeters.Z =
			AircraftModelConfiguration
			.GroundPlaneDownMeters
			- NoseGear.PositionBodyMeters.Z
			+ CompressionMeters;

		InitialAircraftState
			.VelocityBodyMetersPerSecond =
			FVector3{};

		InitialAircraftState
			.AngularRateBodyRadiansPerSecond =
			FVector3{};
	}
	else
	{
		InitialAircraftState
			.VelocityBodyMetersPerSecond.X =
			FMath::Max(
				InitialForwardSpeedMetersPerSecond,
				0.0);
	}

	const bool bDynamicsConfigured =
		Simulation.SetDynamicsConfiguration(
			DynamicsConfiguration);

	const bool bEnvironmentConfigured =
		Simulation.SetEnvironmentConfiguration(
			EnvironmentConfiguration);

	const bool bAircraftModelsConfigured =
		Simulation.SetAircraftModelConfiguration(
			AircraftModelConfiguration);

	if (!bDynamicsConfigured
		|| !bEnvironmentConfigured
		|| !bAircraftModelsConfigured)
	{
		Simulation.ClearAircraftModelConfiguration();
		Simulation.ClearDynamicsConfiguration();
		Simulation.ClearEnvironmentConfiguration();

		return false;
	}

	Simulation.Reset(InitialAircraftState);

	return Simulation.IsDynamicsConfigured()
		&& Simulation.IsEnvironmentConfigured()
		&& Simulation.IsAircraftModelConfigured()
		&& Simulation.WasLastDynamicsStepSuccessful();
}
