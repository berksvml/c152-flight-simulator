#pragma once

#include "FlightDynamics/C152ControlSurfaceModel.h"
#include "FlightDynamics/FixedStepClock.h"
#include "FlightDynamics/FlightDynamicsTypes.h"

#include <cstdint>

namespace C152::FlightDynamics 
{
	class FC152Simulation final
	{
	public:
		FC152Simulation();

		void Reset();
		
		void Reset(
			const FAircraftState& InitialAircraftState);

		[[nodiscard]]
		std::uint32_t Advance(
			const FControlCommand& Command,
			double FrameDeltaSeconds);

		[[nodiscard]]
		const FAircraftState& GetAircraftState() const;

		[[nodiscard]]
		const FControlSurfaceState& GetControlSurfaceState() const;

		[[nodiscard]]
		double GetFixedDeltaSeconds() const;

	private:
		void Step(
			const FControlCommand& Command,
			double FixedDeltaSeconds);

		FAircraftState AircraftState{};
		FC152ControlSurfaceModel ControlSurfaceModel;
		FFixedStepClock SimulationClock;
	};
}