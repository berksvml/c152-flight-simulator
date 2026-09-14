#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/FixedStepClock.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFixedStepClockStepCountTest,
	"C152FlightSim.FlightDynamics.FixedStep.StepCount",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FFixedStepClockStepCountTest::RunTest(
		const FString& Parameters)
{
	using C152::FlightDynamics::FFixedStepClock;

	(void)Parameters;

	FFixedStepClock Clock;

	TestTrue(
		TEXT("60 FPS produces two simulation steps"),
		Clock.Advance(1.0 / 60.0) == 2U);

	Clock.Reset();

	TestTrue(
		TEXT("30 FPS produces four simulation steps"),
		Clock.Advance(1.0 / 30.0) == 4U);

	Clock.Reset();

	TestTrue(
		TEXT("Half a fixed step initially produces no step"),
		Clock.Advance(1.0 / 240.0) == 0U);

	TestTrue(
		TEXT("Two half steps accumulate into one step"),
		Clock.Advance(1.0 / 240.0) == 1U);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFixedStepClockHitchProtectionTest,
	"C152FlightSim.FlightDynamics.FixedStep.HitchProtection",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FFixedStepClockHitchProtectionTest::RunTest(
		const FString& Parameters)
{
	using C152::FlightDynamics::FFixedStepClock;

	(void)Parameters;

	FFixedStepClock Clock(
		1.0 / 120.0,
		8U);

	TestTrue(
		TEXT("A long frame is limited to eight substeps"),
		Clock.Advance(1.0) == 8U);

	TestTrue(
		TEXT("Negative frame time produces no steps"),
		Clock.Advance(-1.0) == 0U);

	return true;
}

#endif