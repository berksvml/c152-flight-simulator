#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/PowerplantModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace PowerplantModelTestData
{
	bool IsPowerplantValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	C152::FlightDynamics::FPowerplantModelConfiguration
		MakeValidConfiguration()
	{
		using namespace C152::FlightDynamics;

		FPowerplantModelConfiguration Configuration{};

		Configuration.Propulsion.RatedPowerWatts =
			100000.0;

		Configuration.Propulsion.PropellerDiameterMeters =
			2.0;

		Configuration.Propulsion
			.PropellerProfileEfficiency =
			0.8;

		Configuration.Propulsion
			.SeaLevelDensityKilogramsPerCubicMeter =
			1.225;

		Configuration.Fuel
			.UsableFuelCapacityKilograms =
			10.0;

		Configuration.Fuel
			.FuelFlowAtRatedPowerKilogramsPerSecond =
			0.01;

		return Configuration;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPowerplantConfigurationTest,
	"C152FlightSim.FlightDynamics."
	"Powerplant.Configuration",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FPowerplantConfigurationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FPowerplantModel Model;

	TestFalse(
		TEXT("New powerplant model is not configured"),
		Model.IsConfigured());

	const FPowerplantModelConfiguration Configuration =
		PowerplantModelTestData::
		MakeValidConfiguration();

	TestTrue(
		TEXT("Valid powerplant configuration is accepted"),
		Model.SetConfiguration(Configuration));

	TestTrue(
		TEXT("Powerplant reports configured state"),
		Model.IsConfigured());

	TestTrue(
		TEXT("Configured powerplant starts with full fuel"),
		PowerplantModelTestData::
		IsPowerplantValueNear(
			Model.GetFuelState()
			.RemainingUsableFuelKilograms,
			10.0));

	FPowerplantModelConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.Fuel
		.UsableFuelCapacityKilograms =
		0.0;

	TestFalse(
		TEXT("Invalid fuel configuration is rejected"),
		Model.SetConfiguration(
			InvalidConfiguration));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPowerplantCoupledOperationTest,
	"C152FlightSim.FlightDynamics."
	"Powerplant.CoupledOperation",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FPowerplantCoupledOperationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FPowerplantModel Model;

	if (!Model.SetConfiguration(
		PowerplantModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure powerplant model."));

		return false;
	}

	FBodyForcesAndMoments Loads{};
	FPowerplantOutput Output{};

	const bool bPoweredStepSucceeded =
		Model.TryAdvance(
			0.5,
			1.225,
			30.0,
			10.0,
			Loads,
			Output);

	TestTrue(
		TEXT("Powered powerplant step succeeds"),
		bPoweredStepSucceeded);

	TestTrue(
		TEXT("Half throttle produces half rated brake power"),
		PowerplantModelTestData::
		IsPowerplantValueNear(
			Output.Propulsion.BrakePowerWatts,
			50000.0,
			1.0e-6));

	TestTrue(
		TEXT("Powered step produces forward thrust"),
		Loads.ForceBodyNewtons.X > 0.0
		&& Loads.ForceBodyNewtons.Y == 0.0
		&& Loads.ForceBodyNewtons.Z == 0.0);

	TestTrue(
		TEXT("Power fraction determines fuel flow"),
		PowerplantModelTestData::
		IsPowerplantValueNear(
			Output.Fuel
			.FuelFlowKilogramsPerSecond,
			0.005));

	TestTrue(
		TEXT("Powered step consumes expected fuel mass"),
		PowerplantModelTestData::
		IsPowerplantValueNear(
			Output.Fuel
			.TotalFuelConsumedKilograms,
			0.05));

	const double ConsumedFuelBeforeIdle =
		Output.Fuel.TotalFuelConsumedKilograms;

	const bool bIdleStepSucceeded =
		Model.TryAdvance(
			0.0,
			1.225,
			30.0,
			10.0,
			Loads,
			Output);

	TestTrue(
		TEXT("Zero-throttle step succeeds"),
		bIdleStepSucceeded);

	TestTrue(
		TEXT("Zero throttle produces no power or thrust"),
		Output.Propulsion.BrakePowerWatts == 0.0
		&& Output.Propulsion.ThrustNewtons == 0.0
		&& Loads.ForceBodyNewtons.X == 0.0);

	TestTrue(
		TEXT("Zero throttle consumes no additional fuel"),
		Output.Fuel.TotalFuelConsumedKilograms
		== ConsumedFuelBeforeIdle);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPowerplantFuelCutoffTest,
	"C152FlightSim.FlightDynamics."
	"Powerplant.FuelCutoff",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FPowerplantFuelCutoffTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FPowerplantModelConfiguration Configuration =
		PowerplantModelTestData::
		MakeValidConfiguration();

	Configuration.Fuel
		.UsableFuelCapacityKilograms =
		0.001;

	FPowerplantModel Model;

	if (!Model.SetConfiguration(Configuration))
	{
		AddError(TEXT(
			"Failed to configure powerplant model."));

		return false;
	}

	FBodyForcesAndMoments Loads{};
	FPowerplantOutput Output{};

	const bool bDepletionStepSucceeded =
		Model.TryAdvance(
			1.0,
			1.225,
			0.0,
			1.0,
			Loads,
			Output);

	TestTrue(
		TEXT("Fuel depletion step succeeds"),
		bDepletionStepSucceeded);

	TestTrue(
		TEXT("Fuel is depleted during the step"),
		Output.Fuel
		.RemainingUsableFuelKilograms
		== 0.0
		&& !Output.bFuelAvailable);

	TestTrue(
		TEXT("Power is limited by available fuel"),
		PowerplantModelTestData::
		IsPowerplantValueNear(
			Output.Propulsion.BrakePowerWatts,
			10000.0,
			1.0e-6));

	TestTrue(
		TEXT("Partial-step fuel supply still produces thrust"),
		Output.Propulsion.ThrustNewtons > 0.0);

	const bool bEmptyTankStepSucceeded =
		Model.TryAdvance(
			1.0,
			1.225,
			0.0,
			1.0,
			Loads,
			Output);

	TestTrue(
		TEXT("Empty-tank step remains valid"),
		bEmptyTankStepSucceeded);

	TestTrue(
		TEXT("Empty tank prevents power and thrust"),
		Output.Propulsion.BrakePowerWatts == 0.0
		&& Output.Propulsion.ThrustNewtons == 0.0
		&& Loads.ForceBodyNewtons.X == 0.0
		&& !Output.bProducingPower);

	return true;
}

#endif