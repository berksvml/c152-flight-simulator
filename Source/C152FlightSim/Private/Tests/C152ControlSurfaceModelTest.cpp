#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152ControlSurfaceModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
	constexpr double TestToleranceRad = 1.0e-9;

	bool IsControlSurfaceValueNear(
		const double Actual,
		const double Expected)
	{
		return std::abs(Actual - Expected) <= TestToleranceRad;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152ControlSurfaceRateLimitTest,
	"C152FlightSim.FlightDynamics.ControlSurface.RateLimit",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152ControlSurfaceRateLimitTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152ControlSurfaceModel Model;

	FControlSurfaceLimits Limits;
	Limits.SurfaceRateRadPerSecond = DegreesToRadians(10.0);
	Model.SetLimits(Limits);

	FControlCommand Command;
	Command.Pitch = 1.0;

	Model.Update(Command, 0.5);

	const FControlSurfaceState& State = Model.GetState();

	TestTrue(
		TEXT("Elevator moves five degrees in half a second"),
		IsControlSurfaceValueNear(
			State.ElevatorRad,
			DegreesToRadians(5.0)));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152ControlSurfaceSaturationTest,
	"C152FlightSim.FlightDynamics.ControlSurface.Saturation",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152ControlSurfaceSaturationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152ControlSurfaceModel Model;

	FControlCommand Command;
	Command.Pitch = 2.0;
	Command.Roll = -2.0;
	Command.Yaw = 2.0;

	Model.Update(Command, 10.0);

	const FControlSurfaceState& PositiveState =
		Model.GetState();

	TestTrue(
		TEXT("Elevator is limited to positive 25 degrees"),
		IsControlSurfaceValueNear(
			PositiveState.ElevatorRad,
			DegreesToRadians(25.0)));

	TestTrue(
		TEXT("Aileron is limited to negative 20 degrees"),
		IsControlSurfaceValueNear(
			PositiveState.AileronRad,
			DegreesToRadians(-20.0)));

	TestTrue(
		TEXT("Rudder is limited to positive 23 degrees"),
		IsControlSurfaceValueNear(
			PositiveState.RudderRad,
			DegreesToRadians(23.0)));

	Model.Reset();

	Command.Pitch = -2.0;
	Command.Roll = 0.0;
	Command.Yaw = -2.0;

	Model.Update(Command, 10.0);

	const FControlSurfaceState& NegativeState =
		Model.GetState();

	TestTrue(
		TEXT("Elevator is limited to negative 18 degrees"),
		IsControlSurfaceValueNear(
			NegativeState.ElevatorRad,
			DegreesToRadians(-18.0)));

	TestTrue(
		TEXT("Rudder is limited to negative 23 degrees"),
		IsControlSurfaceValueNear(
			NegativeState.RudderRad,
			DegreesToRadians(-23.0)));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152ControlSurfaceNeutralReturnTest,
	"C152FlightSim.FlightDynamics.ControlSurface.NeutralReturn",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152ControlSurfaceNeutralReturnTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152ControlSurfaceModel Model;

	FControlSurfaceLimits FastLimits;
	FastLimits.SurfaceRateRadPerSecond =
		DegreesToRadians(100.0);
	Model.SetLimits(FastLimits);

	FControlCommand Command;
	Command.Pitch = 1.0;

	Model.Update(Command, 1.0);

	FControlSurfaceLimits SlowLimits = FastLimits;
	SlowLimits.SurfaceRateRadPerSecond =
		DegreesToRadians(10.0);
	Model.SetLimits(SlowLimits);

	Command.Pitch = 0.0;

	Model.Update(Command, 1.0);

	const FControlSurfaceState& State = Model.GetState();

	TestTrue(
		TEXT("Elevator returns from 25 to 15 degrees"),
		IsControlSurfaceValueNear(
			State.ElevatorRad,
			DegreesToRadians(15.0)));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152ControlSurfaceThrottleClampTest,
	"C152FlightSim.FlightDynamics.ControlSurface.ThrottleClamp",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152ControlSurfaceThrottleClampTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152ControlSurfaceModel Model;
	FControlCommand Command;

	Command.Throttle = 2.0;
	Model.Update(Command, 0.01);

	TestTrue(
		TEXT("Throttle is limited to one"),
		IsControlSurfaceValueNear(Model.GetState().Throttle, 1.0));

	Command.Throttle = -1.0;
	Model.Update(Command, 0.01);

	TestTrue(
		TEXT("Throttle is limited to zero"),
		IsControlSurfaceValueNear(Model.GetState().Throttle, 0.0));

	return true;
}

#endif