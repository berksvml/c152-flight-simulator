#include "FlightDynamics/PropulsionModel.h"

#include <algorithm>
#include <cmath>

namespace PropulsionModelConstants
{
	constexpr double MinimumPositiveValue =
		1.0e-12;

	constexpr double PropulsionPi =
		3.14159265358979323846;

	constexpr int ThrustSolverIterationCount =
		80;

	double EvaluateThrustPolynomial(
		const double ThrustNewtons,
		const double AirDensityKilogramsPerCubicMeter,
		const double PropellerDiskAreaSquareMeters,
		const double AxialAirspeedMetersPerSecond,
		const double PowerDeliveredToAirWatts)
	{
		return ThrustNewtons
			* ThrustNewtons
			* ThrustNewtons
			+ 2.0
			* AirDensityKilogramsPerCubicMeter
			* PropellerDiskAreaSquareMeters
			* AxialAirspeedMetersPerSecond
			* PowerDeliveredToAirWatts
			* ThrustNewtons
			- 2.0
			* AirDensityKilogramsPerCubicMeter
			* PropellerDiskAreaSquareMeters
			* PowerDeliveredToAirWatts
			* PowerDeliveredToAirWatts;
	}
}

namespace C152::FlightDynamics
{
	bool FPropulsionModelConfiguration::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(RatedPowerWatts)
			&& std::isfinite(PropellerDiameterMeters)
			&& std::isfinite(
				PropellerProfileEfficiency)
			&& std::isfinite(
				SeaLevelDensityKilogramsPerCubicMeter);

		if (!bValuesAreFinite)
		{
			return false;
		}

		return RatedPowerWatts
				> PropulsionModelConstants::
			MinimumPositiveValue
			&& PropellerDiameterMeters
				> PropulsionModelConstants::
			MinimumPositiveValue
			&& PropellerProfileEfficiency
				> PropulsionModelConstants::
			MinimumPositiveValue
			&& PropellerProfileEfficiency <= 1.0
			&& SeaLevelDensityKilogramsPerCubicMeter
				> PropulsionModelConstants::
			MinimumPositiveValue;
	}

	bool FPropulsionModel::SetConfiguration(
		const FPropulsionModelConfiguration&
		NewConfiguration)
	{
		if (!NewConfiguration.IsValid())
		{
			return false;
		}

		Configuration = NewConfiguration;
		bIsConfigured = true;

		return true;
	}

	void FPropulsionModel::ClearConfiguration()
	{
		Configuration = {};
		bIsConfigured = false;
	}

	bool FPropulsionModel::IsConfigured() const
	{
		return bIsConfigured;
	}

	bool FPropulsionModel::TryEvaluate(
		const double ThrottlePosition,
		const double AirDensityKilogramsPerCubicMeter,
		const double AxialAirspeedMetersPerSecond,
		FBodyForcesAndMoments& OutLoads,
		FPropulsionOutput& OutPropulsion) const
	{
		const bool bInputsAreFinite =
			std::isfinite(ThrottlePosition)
			&& std::isfinite(
				AirDensityKilogramsPerCubicMeter)
			&& std::isfinite(
				AxialAirspeedMetersPerSecond);

		if (!bIsConfigured
			|| !bInputsAreFinite
			|| ThrottlePosition < 0.0
			|| ThrottlePosition > 1.0
			|| AirDensityKilogramsPerCubicMeter
			<= PropulsionModelConstants::
			MinimumPositiveValue
			|| AxialAirspeedMetersPerSecond < 0.0)
		{
			return false;
		}

		FBodyForcesAndMoments Loads{};
		FPropulsionOutput Propulsion{};

		if (ThrottlePosition
			<= PropulsionModelConstants::
			MinimumPositiveValue)
		{
			OutLoads = Loads;
			OutPropulsion = Propulsion;

			return true;
		}

		const double DensityPowerScale =
			std::clamp(
				AirDensityKilogramsPerCubicMeter
				/ Configuration
				.SeaLevelDensityKilogramsPerCubicMeter,
				0.0,
				1.0);

		Propulsion.BrakePowerWatts =
			Configuration.RatedPowerWatts
			* ThrottlePosition
			* DensityPowerScale;

		Propulsion.PowerDeliveredToAirWatts =
			Propulsion.BrakePowerWatts
			* Configuration.PropellerProfileEfficiency;

		const double PropellerRadiusMeters =
			0.5
			* Configuration.PropellerDiameterMeters;

		const double PropellerDiskAreaSquareMeters =
			PropulsionModelConstants::PropulsionPi
			* PropellerRadiusMeters
			* PropellerRadiusMeters;

		const double StaticThrustUpperBoundNewtons =
			std::cbrt(
				2.0
				* AirDensityKilogramsPerCubicMeter
				* PropellerDiskAreaSquareMeters
				* Propulsion.PowerDeliveredToAirWatts
				* Propulsion.PowerDeliveredToAirWatts);

		if (!std::isfinite(StaticThrustUpperBoundNewtons))
		{
			return false;
		}

		double LowerThrustNewtons = 0.0;
		double UpperThrustNewtons =
			StaticThrustUpperBoundNewtons;

		for (int Iteration = 0;
			Iteration
			< PropulsionModelConstants::
			ThrustSolverIterationCount;
			++Iteration)
		{
			const double CandidateThrustNewtons =
				0.5
				* (LowerThrustNewtons
					+ UpperThrustNewtons);

			const double PolynomialValue =
				PropulsionModelConstants::
				EvaluateThrustPolynomial(
					CandidateThrustNewtons,
					AirDensityKilogramsPerCubicMeter,
					PropellerDiskAreaSquareMeters,
					AxialAirspeedMetersPerSecond,
					Propulsion.PowerDeliveredToAirWatts);

			if (PolynomialValue > 0.0)
			{
				UpperThrustNewtons =
					CandidateThrustNewtons;
			}
			else
			{
				LowerThrustNewtons =
					CandidateThrustNewtons;
			}
		}

		Propulsion.ThrustNewtons =
			0.5
			* (LowerThrustNewtons
				+ UpperThrustNewtons);

		if (Propulsion.BrakePowerWatts
			> PropulsionModelConstants::
			MinimumPositiveValue)
		{
			Propulsion.OverallPropulsiveEfficiency =
				Propulsion.ThrustNewtons
				* AxialAirspeedMetersPerSecond
				/ Propulsion.BrakePowerWatts;
		}

		// Body axes are FRD: propulsion acts along +X.
		Loads.ForceBodyNewtons.X =
			Propulsion.ThrustNewtons;

		const bool bOutputsAreFinite =
			std::isfinite(Propulsion.BrakePowerWatts)
			&& std::isfinite(
				Propulsion.PowerDeliveredToAirWatts)
			&& std::isfinite(Propulsion.ThrustNewtons)
			&& std::isfinite(
				Propulsion.OverallPropulsiveEfficiency)
			&& Loads.ForceBodyNewtons.IsFinite()
			&& Loads.MomentBodyNewtonMeters.IsFinite();

		if (!bOutputsAreFinite)
		{
			return false;
		}

		OutLoads = Loads;
		OutPropulsion = Propulsion;

		return true;
	}
}