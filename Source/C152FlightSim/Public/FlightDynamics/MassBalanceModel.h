#pragma once

#include <cstddef>

namespace C152::FlightDynamics
{
	struct FMassBalanceLoad
	{
		double MassKilograms = 0.0;

		// Longitudinal station measured aft of the aircraft datum [m].
		// Negative stations are allowed for items ahead of the datum.
		double StationMetersAftOfDatum = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FMassBalanceResult
	{
		double TotalMassKilograms = 0.0;
		double TotalMomentKilogramMeters = 0.0;
		double CenterOfGravityMetersAftOfDatum = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	class FMassBalanceModel final
	{
	public:
		// On failure, OutResult is not modified.
		[[nodiscard]]
		static bool TryEvaluate(
			const FMassBalanceLoad* Loads,
			std::size_t LoadCount,
			FMassBalanceResult& OutResult);
	};
}