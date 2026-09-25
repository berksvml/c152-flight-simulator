#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/PropulsionModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace PropulsionModelTestData
{
	bool IsPropulsionModelValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-6)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	C152::FlightDynamics::FPropulsionModelConfiguration
		MakeValidConfiguration()
	{
		using namespace C152::FlightDynamics;

		FPropulsionModelConfiguration Configuration{};

		Configuration.RatedPowerWatts =
			82026.98587404973;

		Configuration.PropellerDiameterMeters =
			1.73355;

		Configuration.PropellerProfileEfficiency =
			0.80;

		Configuration
			.SeaLevelDensityKilogramsPerCubicMeter =
			1.225;

		return Configuration;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPropulsionConfigurationTest,
	"C152FlightSim.FlightDynamics."
	"Propulsion.Configuration",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FPropulsionConfigurationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FPropulsionModel Model;

	TestFalse(
		TEXT("New propulsion model is not configured"),
		Model.IsConfigured());

	const FPropulsionModelConfiguration Configuration =
		PropulsionModelTestData::
		MakeValidConfiguration();

	TestTrue(
		TEXT("Valid propulsion configuration is accepted"),
		Model.SetConfiguration(Configuration));

	TestTrue(
		TEXT("Model reports configured state"),
		Model.IsConfigured());

	FPropulsionModelConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.PropellerProfileEfficiency =
		1.01;

	TestFalse(
		TEXT("Efficiency above one is rejected"),
		Model.SetConfiguration(InvalidConfiguration));

	Model.ClearConfiguration();

	TestFalse(
		TEXT("Clearing removes propulsion configuration"),
		Model.IsConfigured());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPropulsionStaticAndForwardThrustTest,
	"C152FlightSim.FlightDynamics."
	"Propulsion.StaticAndForwardThrust",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FPropulsionStaticAndForwardThrustTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FPropulsionModel Model;

	const FPropulsionModelConfiguration Configuration =
		PropulsionModelTestData::
		MakeValidConfiguration();

	if (!Model.SetConfiguration(Configuration))
	{
		AddError(TEXT(
			"Failed to configure propulsion model."));

		return false;
	}

	FBodyForcesAndMoments StaticLoads{};
	FPropulsionOutput StaticOutput{};

	const bool bStaticSucceeded =
		Model.TryEvaluate(
			1.0,
			1.225,
			0.0,
			StaticLoads,
			StaticOutput);

	TestTrue(
		TEXT("Static thrust evaluation succeeds"),
		bStaticSucceeded);

	TestTrue(
		TEXT("Full throttle produces positive static thrust"),
		StaticOutput.ThrustNewtons > 0.0);

	TestTrue(
		TEXT("Propulsion thrust acts along body-forward X"),
		PropulsionModelTestData::
		IsPropulsionModelValueNear(
			StaticLoads.ForceBodyNewtons.X,
			StaticOutput.ThrustNewtons)
		&& StaticLoads.ForceBodyNewtons.Y == 0.0
		&& StaticLoads.ForceBodyNewtons.Z == 0.0);

	TestTrue(
		TEXT("Static propulsive efficiency is zero"),
		StaticOutput.OverallPropulsiveEfficiency == 0.0);

	FBodyForcesAndMoments ForwardLoads{};
	FPropulsionOutput ForwardOutput{};

	const bool bForwardSucceeded =
		Model.TryEvaluate(
			1.0,
			1.225,
			30.0,
			ForwardLoads,
			ForwardOutput);

	TestTrue(
		TEXT("Forward-flight thrust evaluation succeeds"),
		bForwardSucceeded);

	TestTrue(
		TEXT("Forward-flight thrust is below static thrust"),
		ForwardOutput.ThrustNewtons
		< StaticOutput.ThrustNewtons);

	TestTrue(
		TEXT("Forward flight has positive propulsive efficiency"),
		ForwardOutput.OverallPropulsiveEfficiency > 0.0
		&& ForwardOutput.OverallPropulsiveEfficiency < 1.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPropulsionDensityAndInputValidationTest,
	"C152FlightSim.FlightDynamics."
	"Propulsion.DensityAndInputValidation",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FPropulsionDensityAndInputValidationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FPropulsionModel Model;

	if (!Model.SetConfiguration(
		PropulsionModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure propulsion model."));

		return false;
	}

	FBodyForcesAndMoments SeaLevelLoads{};
	FPropulsionOutput SeaLevelOutput{};

	const bool bSeaLevelSucceeded =
		Model.TryEvaluate(
			1.0,
			1.225,
			30.0,
			SeaLevelLoads,
			SeaLevelOutput);

	FBodyForcesAndMoments ReducedDensityLoads{};
	FPropulsionOutput ReducedDensityOutput{};

	const bool bReducedDensitySucceeded =
		Model.TryEvaluate(
			1.0,
			0.6125,
			30.0,
			ReducedDensityLoads,
			ReducedDensityOutput);

	TestTrue(
		TEXT("Both density evaluations succeed"),
		bSeaLevelSucceeded
		&& bReducedDensitySucceeded);

	TestTrue(
		TEXT("Reduced density decreases available brake power"),
		ReducedDensityOutput.BrakePowerWatts
		< SeaLevelOutput.BrakePowerWatts);

	TestTrue(
		TEXT("Reduced density decreases thrust"),
		ReducedDensityOutput.ThrustNewtons
		< SeaLevelOutput.ThrustNewtons);

	FBodyForcesAndMoments UnchangedLoads{};

	UnchangedLoads.ForceBodyNewtons =
	{ 1.0, 2.0, 3.0 };

	FPropulsionOutput UnchangedOutput{};

	UnchangedOutput.ThrustNewtons =
		123.0;

	const bool bInvalidThrottleSucceeded =
		Model.TryEvaluate(
			1.1,
			1.225,
			30.0,
			UnchangedLoads,
			UnchangedOutput);

	TestFalse(
		TEXT("Throttle above one is rejected"),
		bInvalidThrottleSucceeded);

	TestTrue(
		TEXT("Rejected input leaves outputs unchanged"),
		UnchangedLoads.ForceBodyNewtons.X == 1.0
		&& UnchangedLoads.ForceBodyNewtons.Y == 2.0
		&& UnchangedLoads.ForceBodyNewtons.Z == 3.0
		&& UnchangedOutput.ThrustNewtons == 123.0);

	return true;
}

#endif