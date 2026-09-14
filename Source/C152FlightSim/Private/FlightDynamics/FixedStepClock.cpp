#include "FlightDynamics/FixedStepClock.h"

#include <algorithm>
#include <cmath>

namespace
{
	constexpr double DefaultFixedDeltaSeconds = 1.0 / 120.0;
}

namespace C152::FlightDynamics
{
	FFixedStepClock::FFixedStepClock(
		const double InFixedDeltaSeconds,
		const std::uint32_t InMaximumSubsteps)
		: FixedDeltaSeconds(
			std::isfinite(InFixedDeltaSeconds)
			&& InFixedDeltaSeconds > 0.0
			? InFixedDeltaSeconds
			: DefaultFixedDeltaSeconds)
		, MaximumSubsteps(
			std::max<std::uint32_t>(
				1U,
				InMaximumSubsteps))
	{
	}

	std::uint32_t FFixedStepClock::Advance(
		const double FrameDeltaSeconds)
	{
		if (!std::isfinite(FrameDeltaSeconds)
			|| FrameDeltaSeconds <= 0.0)
		{
			return 0U;
		}

		const double MaximumAcceptedFrameTime =
			FixedDeltaSeconds
			* static_cast<double>(MaximumSubsteps);

		AccumulatorSeconds += std::min(
			FrameDeltaSeconds,
			MaximumAcceptedFrameTime);

		const double StepEpsilon =
			FixedDeltaSeconds * 1.0e-9;

		const auto AvailableSteps =
			static_cast<std::uint32_t>(
				std::floor(
					(AccumulatorSeconds + StepEpsilon)
					/ FixedDeltaSeconds));

		const std::uint32_t StepCount =
			std::min(
				AvailableSteps,
				MaximumSubsteps);

		AccumulatorSeconds -=
			static_cast<double>(StepCount)
			* FixedDeltaSeconds;

		if (AccumulatorSeconds < 0.0)
		{
			AccumulatorSeconds = 0.0;
		}

		if (AccumulatorSeconds >= FixedDeltaSeconds)
		{
			AccumulatorSeconds = std::fmod(
				AccumulatorSeconds,
				FixedDeltaSeconds);
		}

		return StepCount;
	}

	void FFixedStepClock::Reset()
	{
		AccumulatorSeconds = 0.0;
	}

	double FFixedStepClock::GetFixedDeltaSeconds() const
	{
		return FixedDeltaSeconds;
	}

	std::uint32_t
		FFixedStepClock::GetMaximumSubsteps() const
	{
		return MaximumSubsteps;
	}

	double FFixedStepClock::GetInterpolationAlpha() const
	{
		return std::clamp(
			AccumulatorSeconds / FixedDeltaSeconds,
			0.0,
			1.0);
	}
}