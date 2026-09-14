#pragma once

#include "CoreMinimal.h"
#include "FlightDynamics/FlightDynamicsTypes.h"

namespace C152::UnrealIntegration
{
	class C152FLIGHTSIM_API FUnrealCoordinateAdapter
	{
	public:
		[[nodiscard]]
		static FVector NedPositionMetersToUnrealCentimeters(
			const FlightDynamics::FVector3& PositionNedMeters);

		[[nodiscard]]
		static FlightDynamics::FVector3
			UnrealPositionCentimetersToNedMeters(
				const FVector& PositionUnrealCentimeters);

		[[nodiscard]]
		static FVector BodyVelocityMetersPerSecondToUnreal(
			const FlightDynamics::FVector3&
			VelocityBodyMetersPerSecond);

		[[nodiscard]]
		static FlightDynamics::FVector3
			UnrealVelocityToBodyMetersPerSecond(
				const FVector&
				VelocityUnrealCentimetersPerSecond);

		[[nodiscard]]
		static FVector BodyFrdDirectionToUnrealLocal(
			const FlightDynamics::FVector3& BodyDirection);

		[[nodiscard]]
		static FlightDynamics::FVector3
			UnrealLocalDirectionToBodyFrd(
				const FVector& UnrealDirection);
	};
}