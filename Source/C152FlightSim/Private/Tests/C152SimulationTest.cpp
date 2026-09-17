#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152Simulation.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
	bool IsNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected) <= Tolerance;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152SimulationAdvanceTest,
	"C152FlightSim.FlightDynamics.Simulation.Advance",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152SimulationAdvanceTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152Simulation Simulation;

	FAircraftState InitialState{};
	InitialState.PositionNedMeters =
		FVector3{ 10.0, 20.0, 30.0 };

	Simulation.Reset(InitialState);

	FControlCommand Command{};
	Command.Pitch = 1.0;
	Command.Throttle = 0.5;

	const std::uint32_t StepCount =
		Simulation.Advance(
			Command,
			1.0 / 60.0);

	TestTrue(
		TEXT("A 60 Hz frame produces two 120 Hz simulation steps"),
		StepCount == 2U);

	TestTrue(
		TEXT("Simulation advances the control-surface model"),
		std::abs(
			Simulation
			.GetControlSurfaceState()
			.ElevatorRad) > 0.0);

	const FAircraftState& State =
		Simulation.GetAircraftState();

	TestTrue(
		TEXT("State remains unchanged before 6DOF propagation"),
		IsNear(State.PositionNedMeters.X, 10.0)
		&& IsNear(State.PositionNedMeters.Y, 20.0)
		&& IsNear(State.PositionNedMeters.Z, 30.0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152SimulationResetTest,
	"C152FlightSim.FlightDynamics.Simulation.Reset",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152SimulationResetTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FC152Simulation Simulation;

	FControlCommand Command{};
	Command.Roll = 1.0;

	const std::uint32_t StepCountBeforeReset =
		Simulation.Advance(
			Command,
			1.0 / 60.0);

	TestTrue(
		TEXT("Simulation advances before reset"),
		StepCountBeforeReset == 2U);

	FAircraftState ResetState{};
	ResetState.PositionNedMeters =
		FVector3{ 4.0, 5.0, 6.0 };

	ResetState.AttitudeBodyToNed =
		FQuaternion{ 2.0, 0.0, 0.0, 0.0 };

	Simulation.Reset(ResetState);

	const FControlSurfaceState& SurfaceState =
		Simulation.GetControlSurfaceState();

	const FAircraftState& AircraftState =
		Simulation.GetAircraftState();

	TestTrue(
		TEXT("Reset returns control surfaces to neutral"),
		IsNear(SurfaceState.ElevatorRad, 0.0)
		&& IsNear(SurfaceState.AileronRad, 0.0)
		&& IsNear(SurfaceState.RudderRad, 0.0));

	TestTrue(
		TEXT("Reset stores the supplied aircraft position"),
		IsNear(AircraftState.PositionNedMeters.X, 4.0)
		&& IsNear(AircraftState.PositionNedMeters.Y, 5.0)
		&& IsNear(AircraftState.PositionNedMeters.Z, 6.0));

	TestTrue(
		TEXT("Reset normalizes the aircraft attitude"),
		AircraftState.AttitudeBodyToNed.IsNormalized());

	return true;
}

#endif