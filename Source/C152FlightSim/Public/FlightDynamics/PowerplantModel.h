#pragma once

#include "FlightDynamics/FuelModel.h"
#include "FlightDynamics/PropulsionModel.h"

namespace C152::FlightDynamics
{
	struct FPowerplantModelConfiguration
	{
		FPropulsionModelConfiguration Propulsion{};
		FFuelModelConfiguration Fuel{};

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FPowerplantOutput
	{
		FPropulsionOutput Propulsion{};
		FFuelState Fuel{};

		bool bFuelAvailable = false;
		bool bProducingPower = false;
	};

	class FPowerplantModel final
	{
	public:
		[[nodiscard]]
		bool SetConfiguration(
			const FPowerplantModelConfiguration&
			NewConfiguration);

		void ClearConfiguration();

		[[nodiscard]]
		bool IsConfigured() const;

		[[nodiscard]]
		bool TrySetInitialUsableFuelKilograms(
			double InitialUsableFuelKilograms);

		// On failure, internal state and output arguments remain unchanged.
		[[nodiscard]]
		bool TryAdvance(
			double ThrottlePosition,
			double AirDensityKilogramsPerCubicMeter,
			double AxialAirspeedMetersPerSecond,
			double DeltaSeconds,
			FBodyForcesAndMoments& OutLoads,
			FPowerplantOutput& OutPowerplant);

		[[nodiscard]]
		const FFuelState& GetFuelState() const;

		[[nodiscard]]
		bool HasUsableFuel() const;

	private:
		FPowerplantModelConfiguration Configuration{};
		FPropulsionModel PropulsionModel{};
		FFuelModel FuelModel{};

		bool bIsConfigured = false;
	};
}