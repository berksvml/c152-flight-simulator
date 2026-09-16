#include "Integration/UnrealCoordinateAdapter.h"

namespace
{
	constexpr double CentimetersPerMeter = 100.0;
	constexpr double MetersPerCentimeter = 0.01;
}

namespace C152::UnrealIntegration
{
	FVector
		FUnrealCoordinateAdapter::NedPositionMetersToUnrealCentimeters(
			const FlightDynamics::FVector3& PositionNedMeters)
	{
		return FVector(
			PositionNedMeters.X * CentimetersPerMeter,
			PositionNedMeters.Y * CentimetersPerMeter,
			-PositionNedMeters.Z * CentimetersPerMeter);
	}

	FlightDynamics::FVector3
		FUnrealCoordinateAdapter::UnrealPositionCentimetersToNedMeters(
			const FVector& PositionUnrealCentimeters)
	{
		return FlightDynamics::FVector3{
			PositionUnrealCentimeters.X * MetersPerCentimeter,
			PositionUnrealCentimeters.Y * MetersPerCentimeter,
			-PositionUnrealCentimeters.Z * MetersPerCentimeter
		};
	}

	FVector
		FUnrealCoordinateAdapter::BodyVelocityMetersPerSecondToUnreal(
			const FlightDynamics::FVector3&
			VelocityBodyMetersPerSecond)
	{
		return FVector(
			VelocityBodyMetersPerSecond.X * CentimetersPerMeter,
			VelocityBodyMetersPerSecond.Y * CentimetersPerMeter,
			-VelocityBodyMetersPerSecond.Z * CentimetersPerMeter);
	}

	FlightDynamics::FVector3
		FUnrealCoordinateAdapter::UnrealVelocityToBodyMetersPerSecond(
			const FVector& VelocityUnrealCentimetersPerSecond)
	{
		return FlightDynamics::FVector3{
			VelocityUnrealCentimetersPerSecond.X
				* MetersPerCentimeter,
			VelocityUnrealCentimetersPerSecond.Y
				* MetersPerCentimeter,
			-VelocityUnrealCentimetersPerSecond.Z
				* MetersPerCentimeter
		};
	}

	FVector
		FUnrealCoordinateAdapter::BodyFrdDirectionToUnrealLocal(
			const FlightDynamics::FVector3& BodyDirection)
	{
		return FVector(
			BodyDirection.X,
			BodyDirection.Y,
			-BodyDirection.Z);
	}

	FlightDynamics::FVector3
		FUnrealCoordinateAdapter::UnrealLocalDirectionToBodyFrd(
			const FVector& UnrealDirection)
	{
		return FlightDynamics::FVector3{
			UnrealDirection.X,
			UnrealDirection.Y,
			-UnrealDirection.Z
		};
	}

	FQuat
		FUnrealCoordinateAdapter::BodyToNedAttitudeToUnrealRotation(
			const FlightDynamics::FQuaternion& AttitudeBodyToNed)
	{
		FlightDynamics::FQuaternion NormalizedAttitude =
			AttitudeBodyToNed;

		NormalizedAttitude.Normalize();

		FQuat UnrealRotation(
			-NormalizedAttitude.X,
			-NormalizedAttitude.Y,
			NormalizedAttitude.Z,
			NormalizedAttitude.W);

		UnrealRotation.Normalize();

		return UnrealRotation;
	}

	FlightDynamics::FQuaternion
		FUnrealCoordinateAdapter::UnrealRotationToBodyToNedAttitude(
			const FQuat& UnrealRotation)
	{
		FQuat NormalizedUnrealRotation = UnrealRotation;
		NormalizedUnrealRotation.Normalize();

		FlightDynamics::FQuaternion AttitudeBodyToNed{
			NormalizedUnrealRotation.W,
			-NormalizedUnrealRotation.X,
			-NormalizedUnrealRotation.Y,
			NormalizedUnrealRotation.Z
		};

		AttitudeBodyToNed.Normalize();

		return AttitudeBodyToNed;
	}
}