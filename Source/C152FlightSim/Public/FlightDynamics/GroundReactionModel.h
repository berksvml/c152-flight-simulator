#pragma once

#include "FlightDynamics/FlightDynamicsTypes.h"

#include <array>

namespace C152::FlightDynamics
{
	struct FGroundContactPointConfiguration
	{
		// Position relative to aircraft center of gravity in Body/FRD.
		FVector3 PositionBodyMeters{};

		double SpringStiffnessNewtonsPerMeter = 0.0;
		double DampingCoefficientNewtonSecondsPerMeter = 0.0;

		double RollingResistanceCoefficient = 0.0;
		double MaximumBrakingFrictionCoefficient = 0.0;

		// Zero for an unbraked wheel and one for full brake authority.
		double BrakingAuthority = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FGroundReactionModelConfiguration
	{
		// Nose gear, left main gear, right main gear.
		std::array<FGroundContactPointConfiguration, 3>
			ContactPoints{};

		// Smooths friction around zero ground speed.
		double FrictionTransitionSpeedMetersPerSecond = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FGroundReactionResult
	{
		double TotalNormalForceNewtons = 0.0;
		int ActiveContactCount = 0;
		bool bOnGround = false;
	};

	class FGroundReactionModel final
	{
	public:
		[[nodiscard]]
		bool SetConfiguration(
			const FGroundReactionModelConfiguration&
			NewConfiguration);

		void ClearConfiguration();

		[[nodiscard]]
		bool IsConfigured() const;

		// Convenience overload when no non-ground loads are available.
		[[nodiscard]]
		bool TryEvaluate(
			const FAircraftState& AircraftState,
			double GroundPlaneDownMeters,
			double BrakeCommand,
			FBodyForcesAndMoments& OutLoads,
			FGroundReactionResult& OutResult) const;

		// This overload uses aerodynamic, propulsion and other
		// non-ground loads to calculate low-speed static braking.
		[[nodiscard]]
		bool TryEvaluate(
			const FAircraftState& AircraftState,
			double GroundPlaneDownMeters,
			double BrakeCommand,
			const FBodyForcesAndMoments& AppliedLoadsWithoutGround,
			FBodyForcesAndMoments& OutLoads,
			FGroundReactionResult& OutResult) const;

	private:
		FGroundReactionModelConfiguration Configuration{};
		bool bIsConfigured = false;
	};
}