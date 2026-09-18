#pragma once

#include "FlightDynamics/FlightDynamicsTypes.h"

namespace C152::FlightDynamics
{
	struct FAircraftMassProperties
	{
		double MassKilograms = 0.0;

		double InertiaXxKilogramMetersSquared = 0.0;
		double InertiaYyKilogramMetersSquared = 0.0;
		double InertiaZzKilogramMetersSquared = 0.0;

		// The inertia tensor uses -Ixz in its off-diagonal terms.
		double ProductOfInertiaXzKilogramMetersSquared = 0.0;

		[[nodiscard]]
		bool IsValid() const;

		[[nodiscard]]
		FVector3 ApplyInertia(
			const FVector3& Vector) const;

		[[nodiscard]]
		bool TryApplyInverseInertia(
			const FVector3& Vector,
			FVector3& OutResult) const;
	};
}