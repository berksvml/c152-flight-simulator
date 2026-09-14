#pragma once

#include <cstdint>

namespace C152::FlightDynamics
{
	class FFixedStepClock
	{
	public:
		explicit FFixedStepClock(
			double InFixedDeltaSeconds = 1.0 / 120.0,
			std::uint32_t InMaximumSubsteps = 8U);

		std::uint32_t Advance(double FrameDeltaSeconds);

		void Reset();

		[[nodiscard]]
		double GetFixedDeltaSeconds() const;

		[[nodiscard]]
		std::uint32_t GetMaximumSubsteps() const;

		[[nodiscard]]
		double GetInterpolationAlpha() const;

	private:
		double FixedDeltaSeconds;
		std::uint32_t MaximumSubsteps;
		double AccumulatorSeconds = 0.0;
	};
}