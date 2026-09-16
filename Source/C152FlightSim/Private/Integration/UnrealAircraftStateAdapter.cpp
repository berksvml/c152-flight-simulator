#include "Integration/UnrealAircraftStateAdapter.h"

#include "Integration/UnrealCoordinateAdapter.h"

namespace C152::UnrealIntegration
{
	FTransform FUnrealAircraftStateAdapter::ToUnrealTransform(
		const FlightDynamics::FAircraftState& AircraftState)
	{
		const FVector UnrealLocation =
			FUnrealCoordinateAdapter::
			NedPositionMetersToUnrealCentimeters(
				AircraftState.PositionNedMeters);

		FQuat UnrealRotation =
			FUnrealCoordinateAdapter::
			BodyToNedAttitudeToUnrealRotation(
				AircraftState.AttitudeBodyToNed);

		UnrealRotation.Normalize();

		return FTransform(
			UnrealRotation,
			UnrealLocation,
			FVector::OneVector);
	}

	void FUnrealAircraftStateAdapter::
		UpdateCorePoseFromUnrealTransform(
			const FTransform& UnrealTransform,
			FlightDynamics::FAircraftState& InOutAircraftState)
	{
		InOutAircraftState.PositionNedMeters =
			FUnrealCoordinateAdapter::
			UnrealPositionCentimetersToNedMeters(
				UnrealTransform.GetLocation());

		InOutAircraftState.AttitudeBodyToNed =
			FUnrealCoordinateAdapter::
			UnrealRotationToBodyToNedAttitude(
				UnrealTransform.GetRotation());
	}
}