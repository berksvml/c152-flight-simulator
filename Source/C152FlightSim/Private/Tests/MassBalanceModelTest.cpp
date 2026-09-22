#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152AircraftConfiguration.h"
#include "FlightDynamics/MassBalanceModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <limits>

namespace
{
	bool IsMassBalanceValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMassBalanceWeightedAverageTest,
	"C152FlightSim.FlightDynamics.MassBalance.WeightedAverage",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FMassBalanceWeightedAverageTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FMassBalanceLoad Loads[]{
		{ 500.0, 0.8 },
		{ 80.0, 1.0 },
		{ 50.0, 1.1 }
	};

	FMassBalanceResult Result{};

	const bool bSucceeded =
		FMassBalanceModel::TryEvaluate(
			Loads,
			3U,
			Result);

	TestTrue(
		TEXT("Valid loading configuration is evaluated"),
		bSucceeded);

	TestTrue(
		TEXT("Mass-balance result is valid"),
		Result.IsValid());

	TestTrue(
		TEXT("Total mass is the sum of all loads"),
		IsMassBalanceValueNear(
			Result.TotalMassKilograms,
			630.0));

	TestTrue(
		TEXT("Total moment is the sum of load moments"),
		IsMassBalanceValueNear(
			Result.TotalMomentKilogramMeters,
			535.0));

	TestTrue(
		TEXT("C.G. is total moment divided by total mass"),
		IsMassBalanceValueNear(
			Result.CenterOfGravityMetersAftOfDatum,
			0.8492063492063492));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMassBalanceEnvelopeIntegrationTest,
	"C152FlightSim.FlightDynamics.MassBalance.EnvelopeIntegration",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FMassBalanceEnvelopeIntegrationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FMassBalanceLoad ValidLoads[]{
		{ 500.0, 0.8 },
		{ 80.0, 1.0 },
		{ 50.0, 1.1 }
	};

	FMassBalanceResult ValidResult{};

	const bool bValidEvaluationSucceeded =
		FMassBalanceModel::TryEvaluate(
			ValidLoads,
			3U,
			ValidResult);

	TestTrue(
		TEXT("Valid loading evaluation succeeds"),
		bValidEvaluationSucceeded);

	TestTrue(
		TEXT("Calculated loading is inside the C152 C.G. envelope"),
		Configuration.MassAndBalance
		.IsWithinCenterOfGravityEnvelope(
			ValidResult.TotalMassKilograms,
			ValidResult
			.CenterOfGravityMetersAftOfDatum));

	const FMassBalanceLoad AftLoading[]{
		{ 630.0, 1.0 }
	};

	FMassBalanceResult AftResult{};

	const bool bAftEvaluationSucceeded =
		FMassBalanceModel::TryEvaluate(
			AftLoading,
			1U,
			AftResult);

	TestTrue(
		TEXT("Aft loading is mathematically valid"),
		bAftEvaluationSucceeded);

	TestFalse(
		TEXT("Aft loading is outside the C152 C.G. envelope"),
		Configuration.MassAndBalance
		.IsWithinCenterOfGravityEnvelope(
			AftResult.TotalMassKilograms,
			AftResult
			.CenterOfGravityMetersAftOfDatum));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMassBalanceInvalidInputTest,
	"C152FlightSim.FlightDynamics.MassBalance.InvalidInput",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FMassBalanceInvalidInputTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FMassBalanceResult Result{
		1.0,
		2.0,
		3.0
	};

	TestFalse(
		TEXT("Null load array is rejected"),
		FMassBalanceModel::TryEvaluate(
			nullptr,
			1U,
			Result));

	TestFalse(
		TEXT("Empty load array is rejected"),
		FMassBalanceModel::TryEvaluate(
			nullptr,
			0U,
			Result));

	const FMassBalanceLoad NegativeMassLoad[]{
		{ -1.0, 0.8 }
	};

	TestFalse(
		TEXT("Negative mass is rejected"),
		FMassBalanceModel::TryEvaluate(
			NegativeMassLoad,
			1U,
			Result));

	const FMassBalanceLoad NonFiniteStationLoad[]{
		{
			10.0,
			std::numeric_limits<double>::quiet_NaN()
		}
	};

	TestFalse(
		TEXT("Non-finite station is rejected"),
		FMassBalanceModel::TryEvaluate(
			NonFiniteStationLoad,
			1U,
			Result));

	const FMassBalanceLoad ZeroMassLoads[]{
		{ 0.0, 0.0 },
		{ 0.0, 1.0 }
	};

	TestFalse(
		TEXT("Zero total mass is rejected"),
		FMassBalanceModel::TryEvaluate(
			ZeroMassLoads,
			2U,
			Result));

	TestTrue(
		TEXT("Rejected inputs leave the output unchanged"),
		Result.TotalMassKilograms == 1.0
		&& Result.TotalMomentKilogramMeters == 2.0
		&& Result.CenterOfGravityMetersAftOfDatum == 3.0);

	return true;
}

#endif