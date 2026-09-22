#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/FuelModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace FuelModelTestData
{
	bool IsFuelModelValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	C152::FlightDynamics::FFuelModelConfiguration
		MakeValidConfiguration()
	{
		using namespace C152::FlightDynamics;

		FFuelModelConfiguration Configuration{};

		Configuration.UsableFuelCapacityKilograms =
			10.0;

		Configuration
			.FuelFlowAtRatedPowerKilogramsPerSecond =
			0.01;

		return Configuration;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFuelConfigurationAndInitializationTest,
	"C152FlightSim.FlightDynamics."
	"Fuel.ConfigurationAndInitialization",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FFuelConfigurationAndInitializationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FFuelModel Model;

	TestFalse(
		TEXT("New fuel model is not configured"),
		Model.IsConfigured());

	TestTrue(
		TEXT("Valid fuel configuration is accepted"),
		Model.SetConfiguration(
			FuelModelTestData::
			MakeValidConfiguration()));

	TestTrue(
		TEXT("Configured fuel model starts full"),
		FuelModelTestData::IsFuelModelValueNear(
			Model.GetState()
			.RemainingUsableFuelKilograms,
			10.0));

	TestTrue(
		TEXT("Configured fuel model has usable fuel"),
		Model.HasUsableFuel());

	TestTrue(
		TEXT("Initial fuel quantity can be selected"),
		Model.TrySetInitialUsableFuelKilograms(
			4.0));

	TestTrue(
		TEXT("Selected initial fuel quantity is stored"),
		FuelModelTestData::IsFuelModelValueNear(
			Model.GetState()
			.RemainingUsableFuelKilograms,
			4.0));

	const double UnchangedFuel =
		Model.GetState()
		.RemainingUsableFuelKilograms;

	TestFalse(
		TEXT("Fuel quantity above capacity is rejected"),
		Model.TrySetInitialUsableFuelKilograms(
			10.1));

	TestTrue(
		TEXT("Rejected fuel quantity leaves state unchanged"),
		Model.GetState()
		.RemainingUsableFuelKilograms
		== UnchangedFuel);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFuelConsumptionAndDeterminismTest,
	"C152FlightSim.FlightDynamics."
	"Fuel.ConsumptionAndDeterminism",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FFuelConsumptionAndDeterminismTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FFuelModelConfiguration Configuration =
		FuelModelTestData::
		MakeValidConfiguration();

	FFuelModel SingleStepModel;
	FFuelModel SplitStepModel;

	if (!SingleStepModel.SetConfiguration(Configuration)
		|| !SplitStepModel.SetConfiguration(Configuration))
	{
		AddError(TEXT(
			"Failed to configure fuel models."));

		return false;
	}

	const bool bSingleStepSucceeded =
		SingleStepModel.TryAdvance(
			0.5,
			100.0);

	const bool bFirstSplitStepSucceeded =
		SplitStepModel.TryAdvance(
			0.5,
			50.0);

	const bool bSecondSplitStepSucceeded =
		SplitStepModel.TryAdvance(
			0.5,
			50.0);

	TestTrue(
		TEXT("Fuel integration steps succeed"),
		bSingleStepSucceeded
		&& bFirstSplitStepSucceeded
		&& bSecondSplitStepSucceeded);

	const FFuelState& SingleStepState =
		SingleStepModel.GetState();

	const FFuelState& SplitStepState =
		SplitStepModel.GetState();

	TestTrue(
		TEXT("Half power produces half rated fuel flow"),
		FuelModelTestData::IsFuelModelValueNear(
			SingleStepState
			.FuelFlowKilogramsPerSecond,
			0.005));

	TestTrue(
		TEXT("Expected fuel mass is consumed"),
		FuelModelTestData::IsFuelModelValueNear(
			SingleStepState
			.TotalFuelConsumedKilograms,
			0.5));

	TestTrue(
		TEXT("Expected fuel mass remains"),
		FuelModelTestData::IsFuelModelValueNear(
			SingleStepState
			.RemainingUsableFuelKilograms,
			9.5));

	TestTrue(
		TEXT("Split and single integration are deterministic"),
		FuelModelTestData::IsFuelModelValueNear(
			SingleStepState
			.RemainingUsableFuelKilograms,
			SplitStepState
			.RemainingUsableFuelKilograms)
		&& FuelModelTestData::IsFuelModelValueNear(
			SingleStepState
			.TotalFuelConsumedKilograms,
			SplitStepState
			.TotalFuelConsumedKilograms));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFuelDepletionAndValidationTest,
	"C152FlightSim.FlightDynamics."
	"Fuel.DepletionAndValidation",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FFuelDepletionAndValidationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FFuelModelConfiguration Configuration =
		FuelModelTestData::
		MakeValidConfiguration();

	Configuration.UsableFuelCapacityKilograms =
		1.0;

	Configuration
		.FuelFlowAtRatedPowerKilogramsPerSecond =
		0.1;

	FFuelModel Model;

	if (!Model.SetConfiguration(Configuration))
	{
		AddError(TEXT(
			"Failed to configure fuel model."));

		return false;
	}

	const bool bDepletionSucceeded =
		Model.TryAdvance(
			1.0,
			20.0);

	TestTrue(
		TEXT("Fuel depletion step succeeds"),
		bDepletionSucceeded);

	TestTrue(
		TEXT("Fuel mass is clamped at zero"),
		Model.GetState()
		.RemainingUsableFuelKilograms
		== 0.0);

	TestTrue(
		TEXT("Consumed fuel cannot exceed available fuel"),
		FuelModelTestData::IsFuelModelValueNear(
			Model.GetState()
			.TotalFuelConsumedKilograms,
			1.0));

	TestTrue(
		TEXT("Depletion step reports actual average flow"),
		FuelModelTestData::IsFuelModelValueNear(
			Model.GetState()
			.FuelFlowKilogramsPerSecond,
			0.05));

	TestFalse(
		TEXT("Depleted model has no usable fuel"),
		Model.HasUsableFuel());

	const FFuelState StateBeforeInvalidInput =
		Model.GetState();

	const bool bInvalidInputSucceeded =
		Model.TryAdvance(
			1.1,
			1.0);

	TestFalse(
		TEXT("Power fraction above one is rejected"),
		bInvalidInputSucceeded);

	TestTrue(
		TEXT("Rejected input leaves fuel state unchanged"),
		Model.GetState()
		.RemainingUsableFuelKilograms
		== StateBeforeInvalidInput
		.RemainingUsableFuelKilograms
		&& Model.GetState()
		.TotalFuelConsumedKilograms
		== StateBeforeInvalidInput
		.TotalFuelConsumedKilograms
		&& Model.GetState()
		.FuelFlowKilogramsPerSecond
		== StateBeforeInvalidInput
		.FuelFlowKilogramsPerSecond);

	TestTrue(
		TEXT("Advancing with an empty tank remains valid"),
		Model.TryAdvance(
			1.0,
			1.0));

	TestTrue(
		TEXT("Empty tank supplies zero fuel flow"),
		Model.GetState()
		.FuelFlowKilogramsPerSecond
		== 0.0);

	return true;
}

#endif