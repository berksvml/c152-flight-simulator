#pragma once

#include "FlightDynamics/FlightDynamicsTypes.h"

namespace C152::FlightDynamics
{
	struct FPropulsionModelConfiguration
	{
		double RatedPowerWatts = 0.0;
		double PropellerDiameterMeters = 0.0;

		// Fraction of engine brake power delivered to the airflow.
		// This is an initial development parameter.
		double PropellerProfileEfficiency = 0.0;

		double SeaLevelDensityKilogramsPerCubicMeter =
			1.225;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FPropulsionOutput
	{
		double BrakePowerWatts = 0.0;
		double PowerDeliveredToAirWatts = 0.0;
		double ThrustNewtons = 0.0;
		double OverallPropulsiveEfficiency = 0.0;
	};

	class FPropulsionModel final
	{
	public:
		[[nodiscard]]
		bool SetConfiguration(
			const FPropulsionModelConfiguration&
			NewConfiguration);

		void ClearConfiguration();

		[[nodiscard]]
		bool IsConfigured() const;

		// Axial airspeed is positive along body-forward X.
		// On failure, neither output is modified.
		[[nodiscard]]
		bool TryEvaluate(
			double ThrottlePosition,
			double AirDensityKilogramsPerCubicMeter,
			double AxialAirspeedMetersPerSecond,
			FBodyForcesAndMoments& OutLoads,
			FPropulsionOutput& OutPropulsion) const;

	private:
		FPropulsionModelConfiguration Configuration{};
		bool bIsConfigured = false;
	};
}