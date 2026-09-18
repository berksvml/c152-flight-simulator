#include "FlightDynamics/AircraftMassProperties.h"

#include <cmath>

namespace C152::FlightDynamics
{
	bool FAircraftMassProperties::IsValid() const
	{
		constexpr double MinimumPositiveValue = 1.0e-12;

		const bool bValuesAreFinite =
			std::isfinite(MassKilograms)
			&& std::isfinite(
				InertiaXxKilogramMetersSquared)
			&& std::isfinite(
				InertiaYyKilogramMetersSquared)
			&& std::isfinite(
				InertiaZzKilogramMetersSquared)
			&& std::isfinite(
				ProductOfInertiaXzKilogramMetersSquared);

		if (!bValuesAreFinite)
		{
			return false;
		}

		if (MassKilograms <= MinimumPositiveValue
			|| InertiaXxKilogramMetersSquared
			<= MinimumPositiveValue
			|| InertiaYyKilogramMetersSquared
			<= MinimumPositiveValue
			|| InertiaZzKilogramMetersSquared
			<= MinimumPositiveValue)
		{
			return false;
		}

		const double XzBlockDeterminant =
			InertiaXxKilogramMetersSquared
			* InertiaZzKilogramMetersSquared
			- ProductOfInertiaXzKilogramMetersSquared
			* ProductOfInertiaXzKilogramMetersSquared;

		return std::isfinite(XzBlockDeterminant)
			&& XzBlockDeterminant
				> MinimumPositiveValue;
	}

	FVector3 FAircraftMassProperties::ApplyInertia(
		const FVector3& Vector) const
	{
		const double Ixx =
			InertiaXxKilogramMetersSquared;

		const double Iyy =
			InertiaYyKilogramMetersSquared;

		const double Izz =
			InertiaZzKilogramMetersSquared;

		const double Ixz =
			ProductOfInertiaXzKilogramMetersSquared;

		return FVector3{
			Ixx * Vector.X - Ixz * Vector.Z,
			Iyy * Vector.Y,
			-Ixz * Vector.X + Izz * Vector.Z
		};
	}

	bool FAircraftMassProperties::
		TryApplyInverseInertia(
			const FVector3& Vector,
			FVector3& OutResult) const
	{
		if (!IsValid() || !Vector.IsFinite())
		{
			OutResult = FVector3{};
			return false;
		}

		const double Ixx =
			InertiaXxKilogramMetersSquared;

		const double Iyy =
			InertiaYyKilogramMetersSquared;

		const double Izz =
			InertiaZzKilogramMetersSquared;

		const double Ixz =
			ProductOfInertiaXzKilogramMetersSquared;

		const double XzBlockDeterminant =
			Ixx * Izz - Ixz * Ixz;

		OutResult = FVector3{
			(Izz * Vector.X + Ixz * Vector.Z)
				/ XzBlockDeterminant,

			Vector.Y / Iyy,

			(Ixz * Vector.X + Ixx * Vector.Z)
				/ XzBlockDeterminant
		};

		if (!OutResult.IsFinite())
		{
			OutResult = FVector3{};
			return false;
		}

		return true;
	}
}