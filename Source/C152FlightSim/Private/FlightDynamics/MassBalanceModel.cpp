#include "FlightDynamics/MassBalanceModel.h"

#include <cmath>

namespace
{
	constexpr double MinimumPositiveMassKilograms =
		1.0e-12;
}

namespace C152::FlightDynamics
{
	bool FMassBalanceLoad::IsValid() const
	{
		return std::isfinite(MassKilograms)
			&& std::isfinite(StationMetersAftOfDatum)
			&& MassKilograms >= 0.0;
	}

	bool FMassBalanceResult::IsValid() const
	{
		return std::isfinite(TotalMassKilograms)
			&& std::isfinite(TotalMomentKilogramMeters)
			&& std::isfinite(
				CenterOfGravityMetersAftOfDatum)
			&& TotalMassKilograms
				> MinimumPositiveMassKilograms;
	}

	bool FMassBalanceModel::TryEvaluate(
		const FMassBalanceLoad* Loads,
		const std::size_t LoadCount,
		FMassBalanceResult& OutResult)
	{
		if (Loads == nullptr || LoadCount == 0U)
		{
			return false;
		}

		double TotalMassKilograms = 0.0;
		double TotalMomentKilogramMeters = 0.0;

		for (std::size_t Index = 0U;
			Index < LoadCount;
			++Index)
		{
			const FMassBalanceLoad& Load =
				Loads[Index];

			if (!Load.IsValid())
			{
				return false;
			}

			const double LoadMomentKilogramMeters =
				Load.MassKilograms
				* Load.StationMetersAftOfDatum;

			if (!std::isfinite(
				LoadMomentKilogramMeters))
			{
				return false;
			}

			TotalMassKilograms +=
				Load.MassKilograms;

			TotalMomentKilogramMeters +=
				LoadMomentKilogramMeters;

			if (!std::isfinite(TotalMassKilograms)
				|| !std::isfinite(
					TotalMomentKilogramMeters))
			{
				return false;
			}
		}

		if (TotalMassKilograms
			<= MinimumPositiveMassKilograms)
		{
			return false;
		}

		FMassBalanceResult CandidateResult{};

		CandidateResult.TotalMassKilograms =
			TotalMassKilograms;

		CandidateResult.TotalMomentKilogramMeters =
			TotalMomentKilogramMeters;

		CandidateResult
			.CenterOfGravityMetersAftOfDatum =
			TotalMomentKilogramMeters
			/ TotalMassKilograms;

		if (!CandidateResult.IsValid())
		{
			return false;
		}

		OutResult = CandidateResult;
		return true;
	}
}