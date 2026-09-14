#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/FlightDynamicsTypes.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftStateDefaultInitializationTest,
	"C152FlightSim.FlightDynamics.State.DefaultInitialization",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FAircraftStateDefaultInitializationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FAircraftState State;

	TestTrue(
		TEXT("NED position starts at zero"),
		State.PositionNedMeters.X == 0.0
		&& State.PositionNedMeters.Y == 0.0
		&& State.PositionNedMeters.Z == 0.0);

	TestTrue(
		TEXT("Body velocity starts at zero"),
		State.VelocityBodyMetersPerSecond.X == 0.0
		&& State.VelocityBodyMetersPerSecond.Y == 0.0
		&& State.VelocityBodyMetersPerSecond.Z == 0.0);

	TestTrue(
		TEXT("Body angular rates start at zero"),
		State.AngularRateBodyRadiansPerSecond.X == 0.0
		&& State.AngularRateBodyRadiansPerSecond.Y == 0.0
		&& State.AngularRateBodyRadiansPerSecond.Z == 0.0);

	TestTrue(
		TEXT("Initial attitude is the identity quaternion"),
		State.AttitudeBodyToNed.W == 1.0
		&& State.AttitudeBodyToNed.X == 0.0
		&& State.AttitudeBodyToNed.Y == 0.0
		&& State.AttitudeBodyToNed.Z == 0.0);

	TestTrue(
		TEXT("Initial attitude is normalized"),
		State.AttitudeBodyToNed.IsNormalized());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FQuaternionNormalizationTest,
	"C152FlightSim.FlightDynamics.State.QuaternionNormalization",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FQuaternionNormalizationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FQuaternion Quaternion;
	Quaternion.W = 2.0;
	Quaternion.X = 2.0;

	const bool bValidNormalization =
		Quaternion.Normalize();

	TestTrue(
		TEXT("A valid quaternion can be normalized"),
		bValidNormalization);

	TestTrue(
		TEXT("The resulting quaternion has unit norm"),
		Quaternion.IsNormalized());

	FQuaternion InvalidQuaternion;
	InvalidQuaternion.W = 0.0;

	const bool bInvalidNormalization =
		InvalidQuaternion.Normalize();

	TestFalse(
		TEXT("A zero quaternion cannot be normalized"),
		bInvalidNormalization);

	TestTrue(
		TEXT("Invalid quaternion falls back to identity"),
		InvalidQuaternion.W == 1.0
		&& InvalidQuaternion.X == 0.0
		&& InvalidQuaternion.Y == 0.0
		&& InvalidQuaternion.Z == 0.0);

	return true;
}

#endif