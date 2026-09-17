#include "FlightDynamics/C152Simulation.h"

namespace C152::FlightDynamics
{
	FC152Simulation::FC152Simulation()
	{
		Reset();
	}

	void FC152Simulation::Reset()
	{
		Reset(FAircraftState{});
	}

	void FC152Simulation::Reset(
		const FAircraftState& InitialAircraftState)
	{
		AircraftState = InitialAircraftState;
		AircraftState.AttitudeBodyToNed.Normalize();

		ControlSurfaceModel.Reset();
		SimulationClock.Reset();
	}

	std::uint32_t FC152Simulation::Advance(
		const FControlCommand& Command,
		const double FrameDeltaSeconds)
	{
		const std::uint32_t StepCount =
			SimulationClock.Advance(FrameDeltaSeconds);

		const double FixedDeltaSeconds =
			SimulationClock.GetFixedDeltaSeconds();

		for (
			std::uint32_t StepIndex = 0U;
			StepIndex < StepCount;
			++StepIndex)
		{
			Step(Command, FixedDeltaSeconds);
		}

		return StepCount;
	}

	const FAircraftState&
		FC152Simulation::GetAircraftState() const
	{
		return AircraftState;
	}

	const FControlSurfaceState&
		FC152Simulation::GetControlSurfaceState() const
	{
		return ControlSurfaceModel.GetState();
	}

	double FC152Simulation::GetFixedDeltaSeconds() const
	{
		return SimulationClock.GetFixedDeltaSeconds();
	}

	void FC152Simulation::Step(
		const FControlCommand& Command,
		const double FixedDeltaSeconds)
	{
		ControlSurfaceModel.Update(
			Command,
			FixedDeltaSeconds);

		// Rigid-body state propagation will be added here.
	}
}