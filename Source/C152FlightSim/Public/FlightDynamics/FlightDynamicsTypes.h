#pragma once 

namespace C152::FlightDynamics
{
	struct FVector3
	{
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;
	};

	struct FQuaternion
	{
		double W = 1.0;
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;

		[[nodiscard]]
		double NormSquared() const;

		[[nodiscard]]
		bool IsNormalized(
			double Tolerance = 1.0e-9) const;

		bool Normalize();
	};

	struct FAircraftState
	{
		// NED position relative to the simulation origin [m].
		FVector3 PositionNedMeters;

		// Body-axis velocity: u, v, w [m/s].
		FVector3 VelocityBodyMetersPerSecond;

		// Body-axis angular rates: p, q, r [rad/s].
		FVector3 AngularRateBodyRadiansPerSecond;

		// Active rotation from Body/FRD to Navigation/NED.
		FQuaternion AttitudeBodyToNed;
	};

	struct FBodyForcesAndMoments
	{
		// Body-axis forces: Fx, Fy, Fz [N].
		FVector3 ForceBodyNewtons;

		// Body-axis moments: L, M, N [N m].
		FVector3 MomentBodyNewtonMeters;
	};
}