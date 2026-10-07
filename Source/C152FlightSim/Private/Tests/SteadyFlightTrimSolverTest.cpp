#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152AircraftConfiguration.h"
#include "FlightDynamics/SteadyFlightTrimSolver.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace SteadyFlightTrimSolverTestData
{
	bool IsSteadyFlightTrimValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-6)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	bool TryCreateBaselineInputs(
		C152::FlightDynamics::FC152AircraftConfiguration&
		OutAircraftConfiguration,
		C152::FlightDynamics::FPropulsionModelConfiguration&
		OutPropulsionConfiguration,
		C152::FlightDynamics::FAtmosphereState& OutAtmosphere)
	{
		using namespace C152::FlightDynamics;

		const FC152AircraftConfiguration AircraftConfiguration =
			FC152AircraftConfiguration::Create1979Model152();

		FPowerplantModelConfiguration PowerplantConfiguration{};

		if (!AircraftConfiguration
			.TryGetDevelopmentPowerplantConfiguration(
				PowerplantConfiguration))
		{
			return false;
		}

		FAtmosphereState Atmosphere{};

		if (!FStandardAtmosphere::TryEvaluate(
			0.0,
			Atmosphere))
		{
			return false;
		}

		OutAircraftConfiguration = AircraftConfiguration;
		OutPropulsionConfiguration =
			PowerplantConfiguration.Propulsion;

		OutAtmosphere = Atmosphere;
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FSteadyFlightTrimValidationTest,
	"C152FlightSim.FlightDynamics.Trim.Validation",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FSteadyFlightTrimValidationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FSteadyFlightTrimRequest Request{};

	TestFalse(
		TEXT("Default request is invalid without target airspeed"),
		Request.IsValid());

	Request.TrueAirspeedMetersPerSecond = 30.0;

	TestTrue(
		TEXT("Positive flight-speed request is valid"),
		Request.IsValid());

	FSteadyFlightTrimSolverSettings Settings{};

	TestTrue(
		TEXT("Default solver settings are valid"),
		Settings.IsValid());

	Settings.MaximumIterations = 0;

	TestFalse(
		TEXT("Zero solver iterations are rejected"),
		Settings.IsValid());

	FC152AircraftConfiguration AircraftConfiguration{};
	FPropulsionModelConfiguration PropulsionConfiguration{};
	FAtmosphereState Atmosphere{};

	if (!SteadyFlightTrimSolverTestData::
		TryCreateBaselineInputs(
			AircraftConfiguration,
			PropulsionConfiguration,
			Atmosphere))
	{
		AddError(TEXT("Failed to create baseline trim inputs."));
		return false;
	}

	Request.TrueAirspeedMetersPerSecond = 0.1;

	FSteadyFlightTrimResult PreservedResult{};
	PreservedResult.Throttle = 0.75;

	TestFalse(
		TEXT("Invalid trim request is rejected"),
		FSteadyFlightTrimSolver::TrySolve(
			AircraftConfiguration.DevelopmentAerodynamicEstimate,
			PropulsionConfiguration,
			AircraftConfiguration.DevelopmentMassPropertiesEstimate,
			Atmosphere,
			Request,
			PreservedResult));

	TestTrue(
		TEXT("Rejected trim request preserves previous output"),
		PreservedResult.Throttle == 0.75);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FSteadyFlightLevelTrimTest,
	"C152FlightSim.FlightDynamics.Trim.LevelFlight",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FSteadyFlightLevelTrimTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152AircraftConfiguration AircraftConfiguration{};
	FPropulsionModelConfiguration PropulsionConfiguration{};
	FAtmosphereState Atmosphere{};

	if (!SteadyFlightTrimSolverTestData::
		TryCreateBaselineInputs(
			AircraftConfiguration,
			PropulsionConfiguration,
			Atmosphere))
	{
		AddError(TEXT("Failed to create baseline trim inputs."));
		return false;
	}

	FSteadyFlightTrimRequest Request{};
	Request.TrueAirspeedMetersPerSecond = 30.0;

	FSteadyFlightTrimResult Result{};

	const bool bSolved =
		FSteadyFlightTrimSolver::TrySolve(
			AircraftConfiguration.DevelopmentAerodynamicEstimate,
			PropulsionConfiguration,
			AircraftConfiguration.DevelopmentMassPropertiesEstimate,
			Atmosphere,
			Request,
			Result);

	TestTrue(
		TEXT("Baseline C152 level-flight trim converges"),
		bSolved);

	if (!bSolved)
	{
		return false;
	}

	TestTrue(
		TEXT("Level-flight trim residual meets tolerance"),
		Result.NormalizedResidual <= 1.0e-8);

	TestTrue(
		TEXT("Level-flight pitch attitude equals angle of attack"),
		SteadyFlightTrimSolverTestData::
		IsSteadyFlightTrimValueNear(
			Result.PitchAttitudeRadians,
			Result.AngleOfAttackRadians));

	TestTrue(
		TEXT("Baseline angle of attack is deterministic"),
		SteadyFlightTrimSolverTestData::
		IsSteadyFlightTrimValueNear(
			Result.AngleOfAttackRadians,
			0.1108373201,
			1.0e-6));

	TestTrue(
		TEXT("Baseline elevator trim is deterministic"),
		SteadyFlightTrimSolverTestData::
		IsSteadyFlightTrimValueNear(
			Result.ElevatorRadians,
			0.0351544146,
			1.0e-6));

	TestTrue(
		TEXT("Baseline throttle trim is deterministic"),
		SteadyFlightTrimSolverTestData::
		IsSteadyFlightTrimValueNear(
			Result.Throttle,
			0.3323451575,
			1.0e-6));

	TestTrue(
		TEXT("Trim result retains valid flow angles"),
		Result.AirData.bFlowAnglesValid);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FSteadyFlightClimbTrimTest,
	"C152FlightSim.FlightDynamics.Trim.Climb",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FSteadyFlightClimbTrimTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152AircraftConfiguration AircraftConfiguration{};
	FPropulsionModelConfiguration PropulsionConfiguration{};
	FAtmosphereState Atmosphere{};

	if (!SteadyFlightTrimSolverTestData::
		TryCreateBaselineInputs(
			AircraftConfiguration,
			PropulsionConfiguration,
			Atmosphere))
	{
		AddError(TEXT("Failed to create baseline trim inputs."));
		return false;
	}

	FSteadyFlightTrimRequest Request{};
	Request.TrueAirspeedMetersPerSecond = 30.0;
	Request.FlightPathAngleRadians =
		DegreesToRadians(5.0);

	FSteadyFlightTrimResult Result{};

	const bool bSolved =
		FSteadyFlightTrimSolver::TrySolve(
			AircraftConfiguration.DevelopmentAerodynamicEstimate,
			PropulsionConfiguration,
			AircraftConfiguration.DevelopmentMassPropertiesEstimate,
			Atmosphere,
			Request,
			Result);

	TestTrue(
		TEXT("Baseline C152 climb trim converges"),
		bSolved);

	if (!bSolved)
	{
		return false;
	}

	TestTrue(
		TEXT("Climb trim residual meets tolerance"),
		Result.NormalizedResidual <= 1.0e-8);

	TestTrue(
		TEXT("Pitch attitude includes target flight-path angle"),
		SteadyFlightTrimSolverTestData::
		IsSteadyFlightTrimValueNear(
			Result.PitchAttitudeRadians,
			Result.AngleOfAttackRadians
				+ Request.FlightPathAngleRadians));

	const FVector3 VelocityNed =
		Result.AircraftState.AttitudeBodyToNed.RotateVector(
			Result.AircraftState.VelocityBodyMetersPerSecond);

	TestTrue(
		TEXT("Positive flight-path angle produces negative NED down velocity"),
		SteadyFlightTrimSolverTestData::
		IsSteadyFlightTrimValueNear(
			VelocityNed.Z,
			-Request.TrueAirspeedMetersPerSecond
				* std::sin(Request.FlightPathAngleRadians)));

	return true;
}

#endif
