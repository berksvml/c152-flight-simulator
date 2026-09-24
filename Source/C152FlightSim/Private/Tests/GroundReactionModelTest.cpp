#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/GroundReactionModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace GroundReactionModelTestData
{
	bool IsGroundReactionValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-6)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	C152::FlightDynamics::
		FGroundReactionModelConfiguration
		MakeValidConfiguration()
	{
		using namespace C152::FlightDynamics;

		FGroundReactionModelConfiguration Configuration{};

		Configuration
			.FrictionTransitionSpeedMetersPerSecond =
			1.0;

		Configuration.ContactPoints[0]
			.PositionBodyMeters =
		{ 1.0, 0.0, 1.0 };

		Configuration.ContactPoints[1]
			.PositionBodyMeters =
		{ -1.0, -1.0, 1.0 };

		Configuration.ContactPoints[2]
			.PositionBodyMeters =
		{ -1.0, 1.0, 1.0 };

		for (FGroundContactPointConfiguration&
			ContactPoint : Configuration.ContactPoints)
		{
			ContactPoint
				.SpringStiffnessNewtonsPerMeter =
				100000.0;

			ContactPoint
				.DampingCoefficientNewtonSecondsPerMeter =
				5000.0;

			ContactPoint
				.RollingResistanceCoefficient =
				0.02;

			ContactPoint
				.MaximumBrakingFrictionCoefficient =
				0.50;
		}

		Configuration.ContactPoints[0]
			.BrakingAuthority =
			0.0;

		Configuration.ContactPoints[1]
			.BrakingAuthority =
			1.0;

		Configuration.ContactPoints[2]
			.BrakingAuthority =
			1.0;

		return Configuration;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FGroundReactionConfigurationTest,
	"C152FlightSim.FlightDynamics."
	"GroundReaction.Configuration",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FGroundReactionConfigurationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FGroundReactionModel Model;

	TestFalse(
		TEXT("New ground-reaction model is not configured"),
		Model.IsConfigured());

	const FGroundReactionModelConfiguration Configuration =
		GroundReactionModelTestData::
		MakeValidConfiguration();

	TestTrue(
		TEXT("Valid ground configuration is accepted"),
		Model.SetConfiguration(Configuration));

	TestTrue(
		TEXT("Ground-reaction model reports configured state"),
		Model.IsConfigured());

	FGroundReactionModelConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.ContactPoints[0]
		.BrakingAuthority =
		1.1;

	TestFalse(
		TEXT("Braking authority above one is rejected"),
		Model.SetConfiguration(
			InvalidConfiguration));

	Model.ClearConfiguration();

	TestFalse(
		TEXT("Clearing removes ground configuration"),
		Model.IsConfigured());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FGroundReactionNormalForceTest,
	"C152FlightSim.FlightDynamics."
	"GroundReaction.NormalForce",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FGroundReactionNormalForceTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FGroundReactionModel Model;

	if (!Model.SetConfiguration(
		GroundReactionModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure ground-reaction model."));

		return false;
	}

	FAircraftState AircraftState{};

	AircraftState.PositionNedMeters =
	{ 0.0, 0.0, -0.95 };

	FBodyForcesAndMoments Loads{};
	FGroundReactionResult Result{};

	const bool bEvaluationSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			0.0,
			Loads,
			Result);

	TestTrue(
		TEXT("Ground-contact evaluation succeeds"),
		bEvaluationSucceeded);

	TestTrue(
		TEXT("All three wheels contact the runway"),
		Result.bOnGround
		&& Result.ActiveContactCount == 3);

	TestTrue(
		TEXT("Spring penetration produces expected normal force"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			Result.TotalNormalForceNewtons,
			15000.0));

	TestTrue(
		TEXT("Normal force acts upward in level body axes"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			Loads.ForceBodyNewtons.Z,
			-15000.0)
		&& GroundReactionModelTestData::
		IsGroundReactionValueNear(
			Loads.ForceBodyNewtons.X,
			0.0)
		&& GroundReactionModelTestData::
		IsGroundReactionValueNear(
			Loads.ForceBodyNewtons.Y,
			0.0));

	TestTrue(
		TEXT("Contact locations produce expected pitch moment"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			Loads.MomentBodyNewtonMeters.Y,
			-5000.0));

	AircraftState.PositionNedMeters.Z =
		-2.0;

	const bool bAirborneEvaluationSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			0.0,
			Loads,
			Result);

	TestTrue(
		TEXT("Airborne evaluation succeeds"),
		bAirborneEvaluationSucceeded);

	TestTrue(
		TEXT("Airborne aircraft has no ground loads"),
		!Result.bOnGround
		&& Result.ActiveContactCount == 0
		&& Loads.ForceBodyNewtons.NormSquared() == 0.0
		&& Loads.MomentBodyNewtonMeters.NormSquared() == 0.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FGroundReactionRollingAndBrakingTest,
	"C152FlightSim.FlightDynamics."
	"GroundReaction.RollingAndBraking",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FGroundReactionRollingAndBrakingTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FGroundReactionModel Model;

	if (!Model.SetConfiguration(
		GroundReactionModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure ground-reaction model."));

		return false;
	}

	FAircraftState AircraftState{};

	AircraftState.PositionNedMeters =
	{ 0.0, 0.0, -0.95 };

	AircraftState.VelocityBodyMetersPerSecond =
	{ 10.0, 0.0, 0.0 };

	FBodyForcesAndMoments RollingLoads{};
	FGroundReactionResult RollingResult{};

	const bool bRollingSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			0.0,
			RollingLoads,
			RollingResult);

	TestTrue(
		TEXT("Rolling-resistance evaluation succeeds"),
		bRollingSucceeded);

	TestTrue(
		TEXT("Rolling resistance opposes forward motion"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			RollingLoads.ForceBodyNewtons.X,
			-300.0));

	FBodyForcesAndMoments BrakingLoads{};
	FGroundReactionResult BrakingResult{};

	const bool bBrakingSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			1.0,
			BrakingLoads,
			BrakingResult);

	TestTrue(
		TEXT("Braking evaluation succeeds"),
		bBrakingSucceeded);

	TestTrue(
		TEXT("Main-wheel brakes increase opposing force"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			BrakingLoads.ForceBodyNewtons.X,
			-5300.0));

	TestTrue(
		TEXT("Braking does not change normal force"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			BrakingResult.TotalNormalForceNewtons,
			RollingResult.TotalNormalForceNewtons));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FGroundReactionStaticBrakeHoldTest,
	"C152FlightSim.FlightDynamics."
	"GroundReaction.StaticBrakeHold",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FGroundReactionStaticBrakeHoldTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FGroundReactionModel Model;

	if (!Model.SetConfiguration(
		GroundReactionModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure ground-reaction model."));

		return false;
	}

	FAircraftState AircraftState{};

	AircraftState.PositionNedMeters =
	{ 0.0, 0.0, -0.95 };

	FBodyForcesAndMoments AppliedLoadsWithoutGround{};

	AppliedLoadsWithoutGround.ForceBodyNewtons =
	{ 2000.0, 0.0, 0.0 };

	FBodyForcesAndMoments GroundLoads{};
	FGroundReactionResult Result{};

	const bool bEvaluationSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			1.0,
			AppliedLoadsWithoutGround,
			GroundLoads,
			Result);

	TestTrue(
		TEXT("Static-brake evaluation succeeds"),
		bEvaluationSucceeded);

	TestTrue(
		TEXT("Aircraft remains on all three contacts"),
		Result.bOnGround
		&& Result.ActiveContactCount == 3);

	TestTrue(
		TEXT("Static brake cancels available forward force"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			GroundLoads.ForceBodyNewtons.X,
			-2000.0));

	TestTrue(
		TEXT("Combined longitudinal force is zero"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			AppliedLoadsWithoutGround
			.ForceBodyNewtons.X
			+ GroundLoads.ForceBodyNewtons.X,
			0.0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FGroundReactionBrakeReleaseTest,
	"C152FlightSim.FlightDynamics."
	"GroundReaction.BrakeRelease",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FGroundReactionBrakeReleaseTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FGroundReactionModel Model;

	if (!Model.SetConfiguration(
		GroundReactionModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure ground-reaction model."));

		return false;
	}

	FAircraftState AircraftState{};

	AircraftState.PositionNedMeters =
	{ 0.0, 0.0, -0.95 };

	FBodyForcesAndMoments AppliedLoadsWithoutGround{};

	AppliedLoadsWithoutGround.ForceBodyNewtons =
	{ 2000.0, 0.0, 0.0 };

	FBodyForcesAndMoments GroundLoads{};
	FGroundReactionResult Result{};

	const bool bEvaluationSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			0.0,
			AppliedLoadsWithoutGround,
			GroundLoads,
			Result);

	TestTrue(
		TEXT("Released-brake evaluation succeeds"),
		bEvaluationSucceeded);

	TestTrue(
		TEXT("Released brake does not cancel force at zero speed"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			GroundLoads.ForceBodyNewtons.X,
			0.0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FGroundReactionBrakeCapacityExceededTest,
	"C152FlightSim.FlightDynamics."
	"GroundReaction.BrakeCapacityExceeded",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FGroundReactionBrakeCapacityExceededTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FGroundReactionModel Model;

	if (!Model.SetConfiguration(
		GroundReactionModelTestData::
		MakeValidConfiguration()))
	{
		AddError(TEXT(
			"Failed to configure ground-reaction model."));

		return false;
	}

	FAircraftState AircraftState{};

	AircraftState.PositionNedMeters =
	{ 0.0, 0.0, -0.95 };

	FBodyForcesAndMoments AppliedLoadsWithoutGround{};

	AppliedLoadsWithoutGround.ForceBodyNewtons =
	{ 6000.0, 0.0, 0.0 };

	FBodyForcesAndMoments GroundLoads{};
	FGroundReactionResult Result{};

	const bool bEvaluationSucceeded =
		Model.TryEvaluate(
			AircraftState,
			0.0,
			1.0,
			AppliedLoadsWithoutGround,
			GroundLoads,
			Result);

	TestTrue(
		TEXT("Brake-capacity evaluation succeeds"),
		bEvaluationSucceeded);

	TestTrue(
		TEXT("Static braking is limited by available capacity"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			GroundLoads.ForceBodyNewtons.X,
			-5000.0));

	TestTrue(
		TEXT("Excess applied force remains after braking"),
		GroundReactionModelTestData::
		IsGroundReactionValueNear(
			AppliedLoadsWithoutGround
			.ForceBodyNewtons.X
			+ GroundLoads.ForceBodyNewtons.X,
			1000.0));

	return true;
}

#endif