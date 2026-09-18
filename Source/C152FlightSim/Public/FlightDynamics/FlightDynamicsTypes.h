#pragma once

namespace C152::FlightDynamics
{
	struct FVector3
	{
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;

		[[nodiscard]]
		double NormSquared() const;

		[[nodiscard]]
		double Norm() const;

		[[nodiscard]]
		bool IsFinite() const;

		bool Normalize();

		[[nodiscard]]
		FVector3 operator+(
			const FVector3& Other) const;

		[[nodiscard]]
		FVector3 operator-(
			const FVector3& Other) const;

		[[nodiscard]]
		FVector3 operator-() const;

		[[nodiscard]]
		FVector3 operator*(
			double Scalar) const;

		[[nodiscard]]
		FVector3 operator/(
			double Scalar) const;

		FVector3& operator+=(
			const FVector3& Other);

		FVector3& operator-=(
			const FVector3& Other);

		FVector3& operator*=(
			double Scalar);

		FVector3& operator/=(
			double Scalar);
	};

	[[nodiscard]]
	FVector3 operator*(
		double Scalar,
		const FVector3& Vector);

	[[nodiscard]]
	double Dot(
		const FVector3& First,
		const FVector3& Second);

	[[nodiscard]]
	FVector3 Cross(
		const FVector3& First,
		const FVector3& Second);

	struct FQuaternion
	{
		double W = 1.0;
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;

		[[nodiscard]]
		double NormSquared() const;

		[[nodiscard]]
		double Norm() const;

		[[nodiscard]]
		bool IsNormalized(
			double Tolerance = 1.0e-9) const;

		bool Normalize();

		[[nodiscard]]
		FQuaternion Conjugate() const;

		[[nodiscard]]
		FQuaternion operator*(
			const FQuaternion& Other) const;

		[[nodiscard]]
		FVector3 RotateVector(
			const FVector3& Vector) const;

		[[nodiscard]]
		FVector3 InverseRotateVector(
			const FVector3& Vector) const;
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