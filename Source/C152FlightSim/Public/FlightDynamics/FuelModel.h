#pragma once

namespace C152::FlightDynamics
{
	struct FFuelModelConfiguration
	{
		double UsableFuelCapacityKilograms = 0.0;

		// Fuel mass flow when the engine produces rated power.
		double FuelFlowAtRatedPowerKilogramsPerSecond = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FFuelState
	{
		double RemainingUsableFuelKilograms = 0.0;
		double FuelFlowKilogramsPerSecond = 0.0;
		double TotalFuelConsumedKilograms = 0.0;
	};

	class FFuelModel final
	{
	public:
		// A successful configuration starts with a full usable tank.
		[[nodiscard]]
		bool SetConfiguration(
			const FFuelModelConfiguration& NewConfiguration);

		void ClearConfiguration();

		[[nodiscard]]
		bool IsConfigured() const;

		// Resets the simulation fuel state and consumption counter.
		[[nodiscard]]
		bool TrySetInitialUsableFuelKilograms(
			double InitialUsableFuelKilograms);

		// RatedPowerFraction must be in [0, 1].
		// On failure, the fuel state is not modified.
		[[nodiscard]]
		bool TryAdvance(
			double RatedPowerFraction,
			double DeltaSeconds);

		[[nodiscard]]
		const FFuelState& GetState() const;

		[[nodiscard]]
		bool HasUsableFuel() const;

	private:
		FFuelModelConfiguration Configuration{};
		FFuelState State{};
		bool bIsConfigured = false;
	};
}