#if WITH_DEV_AUTOMATION_TESTS

#include "Integration/UnrealAircraftStateAdapter.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
	bool IsAircraftStateValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected) <= Tolerance;
	}

	bool AreEquivalentOrientations(
		const C152::FlightDynamics::FQuaternion& First,
		const C152::FlightDynamics::FQuaternion& Second)
	{
		const double DotProduct =
			First.W * Second.W
			+ First.X * Second.X
			+ First.Y * Second.Y
			+ First.Z * Second.Z;

		// q and -q represent the same physical attitude.
		return IsAircraftStateValueNear(std::abs(DotProduct), 1.0);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftStateToUnrealTransformTest,
	"C152FlightSim.Integration.AircraftState.ToUnrealTransform",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FAircraftStateToUnrealTransformTest::RunTest(
		const FString& Parameters)
{
	using namespace C152;

	(void)Parameters;

	const double YawHalfAngle =
		FMath::DegreesToRadians(40.0) * 0.5;

	FlightDynamics::FAircraftState AircraftState{};

	AircraftState.PositionNedMeters =
		FlightDynamics::FVector3{
			1.0,
			2.0,
			3.0
	};

	AircraftState.AttitudeBodyToNed =
		FlightDynamics::FQuaternion{
			std::cos(YawHalfAngle),
			0.0,
			0.0,
			std::sin(YawHalfAngle)
	};

	const FTransform UnrealTransform =
		UnrealIntegration::FUnrealAircraftStateAdapter::
		ToUnrealTransform(AircraftState);

	TestTrue(
		TEXT("Aircraft NED position maps to Unreal location"),
		UnrealTransform.GetLocation().Equals(
			FVector(100.0, 200.0, -300.0),
			0.001));

	TestTrue(
		TEXT("Aircraft attitude maps to Unreal rotation"),
		FMath::IsNearlyEqual(
			UnrealTransform.Rotator().Yaw,
			40.0,
			0.001));

	TestTrue(
		TEXT("Unreal transform scale remains one"),
		UnrealTransform.GetScale3D().Equals(
			FVector::OneVector));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftStatePoseRoundTripTest,
	"C152FlightSim.Integration.AircraftState.PoseRoundTrip",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FAircraftStatePoseRoundTripTest::RunTest(
		const FString& Parameters)
{
	using namespace C152;

	(void)Parameters;

	const double RollHalfAngle =
		FMath::DegreesToRadians(25.0) * 0.5;

	FlightDynamics::FAircraftState OriginalState{};

	OriginalState.PositionNedMeters =
		FlightDynamics::FVector3{
			12.0,
			-4.0,
			150.0
	};

	OriginalState.AttitudeBodyToNed =
		FlightDynamics::FQuaternion{
			std::cos(RollHalfAngle),
			std::sin(RollHalfAngle),
			0.0,
			0.0
	};

	const FTransform UnrealTransform =
		UnrealIntegration::FUnrealAircraftStateAdapter::
		ToUnrealTransform(OriginalState);

	FlightDynamics::FAircraftState RoundTripState{};

	// Pose conversion must not modify these values.
	RoundTripState.VelocityBodyMetersPerSecond =
		FlightDynamics::FVector3{
			10.0,
			20.0,
			30.0
	};

	RoundTripState.AngularRateBodyRadiansPerSecond =
		FlightDynamics::FVector3{
			0.1,
			0.2,
			0.3
	};

	UnrealIntegration::FUnrealAircraftStateAdapter::
		UpdateCorePoseFromUnrealTransform(
			UnrealTransform,
			RoundTripState);

	TestTrue(
		TEXT("Aircraft position survives pose round trip"),
		IsAircraftStateValueNear(
			RoundTripState.PositionNedMeters.X,
			OriginalState.PositionNedMeters.X)
		&& IsAircraftStateValueNear(
			RoundTripState.PositionNedMeters.Y,
			OriginalState.PositionNedMeters.Y)
		&& IsAircraftStateValueNear(
			RoundTripState.PositionNedMeters.Z,
			OriginalState.PositionNedMeters.Z));

	TestTrue(
		TEXT("Aircraft attitude survives pose round trip"),
		AreEquivalentOrientations(
			RoundTripState.AttitudeBodyToNed,
			OriginalState.AttitudeBodyToNed));

	TestTrue(
		TEXT("Pose conversion does not overwrite body velocity"),
		IsAircraftStateValueNear(
			RoundTripState.VelocityBodyMetersPerSecond.X,
			10.0)
		&& IsAircraftStateValueNear(
			RoundTripState.VelocityBodyMetersPerSecond.Y,
			20.0)
		&& IsAircraftStateValueNear(
			RoundTripState.VelocityBodyMetersPerSecond.Z,
			30.0));

	TestTrue(
		TEXT("Pose conversion does not overwrite angular rate"),
		IsAircraftStateValueNear(
			RoundTripState.AngularRateBodyRadiansPerSecond.X,
			0.1)
		&& IsAircraftStateValueNear(
			RoundTripState.AngularRateBodyRadiansPerSecond.Y,
			0.2)
		&& IsAircraftStateValueNear(
			RoundTripState.AngularRateBodyRadiansPerSecond.Z,
			0.3));

	return true;
}

#endif