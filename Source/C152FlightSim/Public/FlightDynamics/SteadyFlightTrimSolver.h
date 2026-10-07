#pragma once

#include "FlightDynamics/AerodynamicModel.h"
#include "FlightDynamics/AircraftMassProperties.h"
#include "FlightDynamics/PropulsionModel.h"
#include "FlightDynamics/StandardAtmosphere.h"

namespace C152::FlightDynamics
{
	struct FSteadyFlightTrimRequest
	{
		// Target true airspeed and inertial flight-path angle.
		// Positive flight-path angle represents a climb [rad].
		double TrueAirspeedMetersPerSecond = 0.0;
		double FlightPathAngleRadians = 0.0;

		double GravityAccelerationMetersPerSecondSquared =
			9.80665;

		// Search bounds remain inside the current linear aerodynamic
		// model's intended pre-stall development range.
		double MinimumAngleOfAttackRadians =
			DegreesToRadians(-10.0);

		double MaximumAngleOfAttackRadians =
			DegreesToRadians(18.0);

		double MinimumElevatorRadians =
			DegreesToRadians(-18.0);

		double MaximumElevatorRadians =
			DegreesToRadians(25.0);

		// Deterministic initial iterate.
		double InitialAngleOfAttackRadians =
			DegreesToRadians(4.0);

		double InitialElevatorRadians = 0.0;
		double InitialThrottle = 0.5;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FSteadyFlightTrimSolverSettings
	{
		int MaximumIterations = 40;
		double NormalizedResidualTolerance = 1.0e-8;
		double FiniteDifferenceAngleStepRadians = 1.0e-5;
		double FiniteDifferenceThrottleStep = 1.0e-4;
		double MinimumLineSearchScale = 1.0 / 1024.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FSteadyFlightTrimResult
	{
		double AngleOfAttackRadians = 0.0;
		double PitchAttitudeRadians = 0.0;
		double ElevatorRadians = 0.0;
		double Throttle = 0.0;

		FAircraftState AircraftState{};
		FControlSurfaceState ControlSurfaceState{};
		FAirData AirData{};
		FAerodynamicCoefficients AerodynamicCoefficients{};
		FPropulsionOutput Propulsion{};

		FBodyForcesAndMoments AerodynamicLoads{};
		FBodyForcesAndMoments PropulsionLoads{};

		// Residuals include gravity and should approach zero at trim.
		FVector3 ResidualForceBodyNewtons{};
		double ResidualPitchMomentNewtonMeters = 0.0;
		double NormalizedResidual = 0.0;

		int IterationCount = 0;
	};

	class FSteadyFlightTrimSolver final
	{
	public:
		// Solves symmetric steady flight for angle of attack, elevator,
		// and throttle. On failure, OutResult is not modified.
		[[nodiscard]]
		static bool TrySolve(
			const FAerodynamicModelConfiguration&
			AerodynamicConfiguration,
			const FPropulsionModelConfiguration&
			PropulsionConfiguration,
			const FAircraftMassProperties& MassProperties,
			const FAtmosphereState& Atmosphere,
			const FSteadyFlightTrimRequest& Request,
			FSteadyFlightTrimResult& OutResult);

		[[nodiscard]]
		static bool TrySolve(
			const FAerodynamicModelConfiguration&
			AerodynamicConfiguration,
			const FPropulsionModelConfiguration&
			PropulsionConfiguration,
			const FAircraftMassProperties& MassProperties,
			const FAtmosphereState& Atmosphere,
			const FSteadyFlightTrimRequest& Request,
			const FSteadyFlightTrimSolverSettings& Settings,
			FSteadyFlightTrimResult& OutResult);
	};
}
