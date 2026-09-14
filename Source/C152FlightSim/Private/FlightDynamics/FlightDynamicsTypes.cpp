#include "FlightDynamics/FlightDynamicsTypes.h"

#include <algorithm>
#include <cmath>

namespace C152::FlightDynamics
{
	double FQuaternion::NormSquared() const
	{
		return W * W + X * X + Y * Y + Z * Z;
	}

	bool FQuaternion::IsNormalized(
		const double Tolerance) const
	{
		const double SafeTolerance =
			std::max(0.0, Tolerance);

		const double CurrentNormSquared =
			NormSquared();

		return std::isfinite(CurrentNormSquared)
			&& std::abs(CurrentNormSquared - 1.0)
			<= SafeTolerance;
	}

	bool FQuaternion::Normalize()
	{
		constexpr double MinimumNormSquared = 1.0e-24;

		const double CurrentNormSquared =
			NormSquared();

		if (!std::isfinite(CurrentNormSquared)
			|| CurrentNormSquared <= MinimumNormSquared)
		{
			W = 1.0;
			X = 0.0;
			Y = 0.0;
			Z = 0.0;

			return false;
		}

		const double InverseNorm =
			1.0 / std::sqrt(CurrentNormSquared);

		W *= InverseNorm;
		X *= InverseNorm;
		Y *= InverseNorm;
		Z *= InverseNorm;

		return true;
	}
}