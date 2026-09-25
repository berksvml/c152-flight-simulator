#include "FlightDynamics/FuelModel.h"

#include <algorithm>
#include <cmath>

namespace FuelModelConstants
{
	constexpr double MinimumPositiveValue =
		1.0e-12;
}

namespace C152::FlightDynamics
{
	bool FFuelModelConfiguration::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(UsableFuelCapacityKilograms)
			&& std::isfinite(
				FuelFlowAtRatedPowerKilogramsPerSecond);

		if (!bValuesAreFinite)
		{
			return false;
		}

		return UsableFuelCapacityKilograms
				> FuelModelConstants::MinimumPositiveValue
			&& FuelFlowAtRatedPowerKilogramsPerSecond
				> FuelModelConstants::MinimumPositiveValue;
	}

	bool FFuelModel::SetConfiguration(
		const FFuelModelConfiguration& NewConfiguration)
	{
		if (!NewConfiguration.IsValid())
		{
			return false;
		}

		Configuration = NewConfiguration;

		State = {};
		State.RemainingUsableFuelKilograms =
			Configuration.UsableFuelCapacityKilograms;

		bIsConfigured = true;

		return true;
	}

	void FFuelModel::ClearConfiguration()
	{
		Configuration = {};
		State = {};
		bIsConfigured = false;
	}

	bool FFuelModel::IsConfigured() const
	{
		return bIsConfigured;
	}

	bool FFuelModel::TrySetInitialUsableFuelKilograms(
		const double InitialUsableFuelKilograms)
	{
		if (!bIsConfigured
			|| !std::isfinite(
				InitialUsableFuelKilograms)
			|| InitialUsableFuelKilograms < 0.0
			|| InitialUsableFuelKilograms
			> Configuration
			.UsableFuelCapacityKilograms)
		{
			return false;
		}

		FFuelState NewState{};

		NewState.RemainingUsableFuelKilograms =
			InitialUsableFuelKilograms;

		State = NewState;

		return true;
	}

	bool FFuelModel::TryAdvance(
		const double RatedPowerFraction,
		const double DeltaSeconds)
	{
		const bool bInputsAreFinite =
			std::isfinite(RatedPowerFraction)
			&& std::isfinite(DeltaSeconds);

		if (!bIsConfigured
			|| !bInputsAreFinite
			|| RatedPowerFraction < 0.0
			|| RatedPowerFraction > 1.0
			|| DeltaSeconds
			<= FuelModelConstants::MinimumPositiveValue)
		{
			return false;
		}

		FFuelState NewState = State;

		if (RatedPowerFraction
			<= FuelModelConstants::MinimumPositiveValue
			|| State.RemainingUsableFuelKilograms
			<= FuelModelConstants::MinimumPositiveValue)
		{
			NewState.RemainingUsableFuelKilograms =
				std::max(
					0.0,
					State.RemainingUsableFuelKilograms);

			NewState.FuelFlowKilogramsPerSecond =
				0.0;

			State = NewState;

			return true;
		}

		const double RequestedFuelFlowKilogramsPerSecond =
			Configuration
			.FuelFlowAtRatedPowerKilogramsPerSecond
			* RatedPowerFraction;

		const double MaximumAvailableFuelFlow =
			State.RemainingUsableFuelKilograms
			/ DeltaSeconds;

		const double ActualFuelFlowKilogramsPerSecond =
			std::min(
				RequestedFuelFlowKilogramsPerSecond,
				MaximumAvailableFuelFlow);

		const double ConsumedFuelKilograms =
			ActualFuelFlowKilogramsPerSecond
			* DeltaSeconds;

		NewState.RemainingUsableFuelKilograms =
			std::max(
				0.0,
				State.RemainingUsableFuelKilograms
				- ConsumedFuelKilograms);

		NewState.FuelFlowKilogramsPerSecond =
			ActualFuelFlowKilogramsPerSecond;

		NewState.TotalFuelConsumedKilograms =
			State.TotalFuelConsumedKilograms
			+ ConsumedFuelKilograms;

		const bool bStateIsFinite =
			std::isfinite(
				NewState.RemainingUsableFuelKilograms)
			&& std::isfinite(
				NewState.FuelFlowKilogramsPerSecond)
			&& std::isfinite(
				NewState.TotalFuelConsumedKilograms);

		if (!bStateIsFinite)
		{
			return false;
		}

		State = NewState;

		return true;
	}

	const FFuelState& FFuelModel::GetState() const
	{
		return State;
	}

	bool FFuelModel::HasUsableFuel() const
	{
		return bIsConfigured
			&& State.RemainingUsableFuelKilograms
				> FuelModelConstants::MinimumPositiveValue;
	}
}