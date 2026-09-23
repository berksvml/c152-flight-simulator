#include "FlightDynamics/PowerplantModel.h"

#include <algorithm>
#include <cmath>

namespace PowerplantModelConstants
{
	constexpr double MinimumPositiveValue =
		1.0e-12;
}

namespace C152::FlightDynamics
{
	bool FPowerplantModelConfiguration::IsValid() const
	{
		return Propulsion.IsValid()
			&& Fuel.IsValid();
	}

	bool FPowerplantModel::SetConfiguration(
		const FPowerplantModelConfiguration&
		NewConfiguration)
	{
		if (!NewConfiguration.IsValid())
		{
			return false;
		}

		FPropulsionModel NewPropulsionModel;
		FFuelModel NewFuelModel;

		if (!NewPropulsionModel.SetConfiguration(
			NewConfiguration.Propulsion)
			|| !NewFuelModel.SetConfiguration(
				NewConfiguration.Fuel))
		{
			return false;
		}

		Configuration = NewConfiguration;
		PropulsionModel = NewPropulsionModel;
		FuelModel = NewFuelModel;
		bIsConfigured = true;

		return true;
	}

	void FPowerplantModel::ClearConfiguration()
	{
		Configuration = {};

		PropulsionModel.ClearConfiguration();
		FuelModel.ClearConfiguration();

		bIsConfigured = false;
	}

	bool FPowerplantModel::IsConfigured() const
	{
		return bIsConfigured;
	}

	bool FPowerplantModel::
		TrySetInitialUsableFuelKilograms(
			const double InitialUsableFuelKilograms)
	{
		if (!bIsConfigured)
		{
			return false;
		}

		return FuelModel
			.TrySetInitialUsableFuelKilograms(
				InitialUsableFuelKilograms);
	}

	bool FPowerplantModel::TryAdvance(
		const double ThrottlePosition,
		const double AirDensityKilogramsPerCubicMeter,
		const double AxialAirspeedMetersPerSecond,
		const double DeltaSeconds,
		FBodyForcesAndMoments& OutLoads,
		FPowerplantOutput& OutPowerplant)
	{
		const bool bInputsAreFinite =
			std::isfinite(ThrottlePosition)
			&& std::isfinite(
				AirDensityKilogramsPerCubicMeter)
			&& std::isfinite(
				AxialAirspeedMetersPerSecond)
			&& std::isfinite(DeltaSeconds);

		if (!bIsConfigured
			|| !bInputsAreFinite
			|| ThrottlePosition < 0.0
			|| ThrottlePosition > 1.0
			|| AirDensityKilogramsPerCubicMeter
			<= PowerplantModelConstants::
			MinimumPositiveValue
			|| AxialAirspeedMetersPerSecond < 0.0
			|| DeltaSeconds
			<= PowerplantModelConstants::
			MinimumPositiveValue)
		{
			return false;
		}

		// Work on a copy so a failure cannot partially consume fuel.
		FFuelModel CandidateFuelModel =
			FuelModel;

		double EffectiveThrottlePosition =
			CandidateFuelModel.HasUsableFuel()
			? ThrottlePosition
			: 0.0;

		FBodyForcesAndMoments Loads{};
		FPropulsionOutput Propulsion{};

		if (!PropulsionModel.TryEvaluate(
			EffectiveThrottlePosition,
			AirDensityKilogramsPerCubicMeter,
			AxialAirspeedMetersPerSecond,
			Loads,
			Propulsion))
		{
			return false;
		}

		const double RequestedRatedPowerFraction =
			Propulsion.BrakePowerWatts
			/ Configuration.Propulsion.RatedPowerWatts;

		if (!CandidateFuelModel.TryAdvance(
			RequestedRatedPowerFraction,
			DeltaSeconds))
		{
			return false;
		}

		const double RequestedFuelFlowKilogramsPerSecond =
			Configuration.Fuel
			.FuelFlowAtRatedPowerKilogramsPerSecond
			* RequestedRatedPowerFraction;

		const double ActualFuelFlowKilogramsPerSecond =
			CandidateFuelModel.GetState()
			.FuelFlowKilogramsPerSecond;

		// If the tank emptied during this step, scale the average
		// power to the fuel that was actually available.
		if (RequestedFuelFlowKilogramsPerSecond
			> PowerplantModelConstants::
			MinimumPositiveValue)
		{
			const double FuelSupplyRatio =
				std::clamp(
					ActualFuelFlowKilogramsPerSecond
					/ RequestedFuelFlowKilogramsPerSecond,
					0.0,
					1.0);

			if (FuelSupplyRatio < 1.0)
			{
				EffectiveThrottlePosition *=
					FuelSupplyRatio;

				if (!PropulsionModel.TryEvaluate(
					EffectiveThrottlePosition,
					AirDensityKilogramsPerCubicMeter,
					AxialAirspeedMetersPerSecond,
					Loads,
					Propulsion))
				{
					return false;
				}
			}
		}

		FPowerplantOutput Powerplant{};

		Powerplant.Propulsion =
			Propulsion;

		Powerplant.Fuel =
			CandidateFuelModel.GetState();

		Powerplant.bFuelAvailable =
			CandidateFuelModel.HasUsableFuel();

		Powerplant.bProducingPower =
			Propulsion.BrakePowerWatts
			> PowerplantModelConstants::
			MinimumPositiveValue;

		FuelModel = CandidateFuelModel;

		OutLoads = Loads;
		OutPowerplant = Powerplant;

		return true;
	}

	const FFuelState&
		FPowerplantModel::GetFuelState() const
	{
		return FuelModel.GetState();
	}

	bool FPowerplantModel::HasUsableFuel() const
	{
		return bIsConfigured
			&& FuelModel.HasUsableFuel();
	}
}