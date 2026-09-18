#include "FlightDynamics/FlightDynamicsTypes.h"

#include <algorithm>
#include <cmath>

namespace C152::FlightDynamics
{
	double FVector3::NormSquared() const
	{
		return X * X + Y * Y + Z * Z;
	}

	double FVector3::Norm() const
	{
		return std::sqrt(NormSquared());
	}

	bool FVector3::IsFinite() const
	{
		return std::isfinite(X)
			&& std::isfinite(Y)
			&& std::isfinite(Z);
	}

	bool FVector3::Normalize()
	{
		constexpr double MinimumNormSquared = 1.0e-24;

		const double CurrentNormSquared =
			NormSquared();

		if (!std::isfinite(CurrentNormSquared)
			|| CurrentNormSquared <= MinimumNormSquared)
		{
			X = 0.0;
			Y = 0.0;
			Z = 0.0;

			return false;
		}

		const double InverseNorm =
			1.0 / std::sqrt(CurrentNormSquared);

		X *= InverseNorm;
		Y *= InverseNorm;
		Z *= InverseNorm;

		return true;
	}

	FVector3 FVector3::operator+(
		const FVector3& Other) const
	{
		return FVector3{
			X + Other.X,
			Y + Other.Y,
			Z + Other.Z
		};
	}

	FVector3 FVector3::operator-(
		const FVector3& Other) const
	{
		return FVector3{
			X - Other.X,
			Y - Other.Y,
			Z - Other.Z
		};
	}

	FVector3 FVector3::operator-() const
	{
		return FVector3{
			-X,
			-Y,
			-Z
		};
	}

	FVector3 FVector3::operator*(
		const double Scalar) const
	{
		return FVector3{
			X * Scalar,
			Y * Scalar,
			Z * Scalar
		};
	}

	FVector3 FVector3::operator/(
		const double Scalar) const
	{
		return FVector3{
			X / Scalar,
			Y / Scalar,
			Z / Scalar
		};
	}

	FVector3& FVector3::operator+=(
		const FVector3& Other)
	{
		X += Other.X;
		Y += Other.Y;
		Z += Other.Z;

		return *this;
	}

	FVector3& FVector3::operator-=(
		const FVector3& Other)
	{
		X -= Other.X;
		Y -= Other.Y;
		Z -= Other.Z;

		return *this;
	}

	FVector3& FVector3::operator*=(
		const double Scalar)
	{
		X *= Scalar;
		Y *= Scalar;
		Z *= Scalar;

		return *this;
	}

	FVector3& FVector3::operator/=(
		const double Scalar)
	{
		X /= Scalar;
		Y /= Scalar;
		Z /= Scalar;

		return *this;
	}

	FVector3 operator*(
		const double Scalar,
		const FVector3& Vector)
	{
		return Vector * Scalar;
	}

	double Dot(
		const FVector3& First,
		const FVector3& Second)
	{
		return First.X * Second.X
			+ First.Y * Second.Y
			+ First.Z * Second.Z;
	}

	FVector3 Cross(
		const FVector3& First,
		const FVector3& Second)
	{
		return FVector3{
			First.Y * Second.Z
				- First.Z * Second.Y,

			First.Z * Second.X
				- First.X * Second.Z,

			First.X * Second.Y
				- First.Y * Second.X
		};
	}

	double FQuaternion::NormSquared() const
	{
		return W * W + X * X + Y * Y + Z * Z;
	}

	double FQuaternion::Norm() const
	{
		return std::sqrt(NormSquared());
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

	FQuaternion FQuaternion::Conjugate() const
	{
		return FQuaternion{
			W,
			-X,
			-Y,
			-Z
		};
	}

	FQuaternion FQuaternion::operator*(
		const FQuaternion& Other) const
	{
		return FQuaternion{
			W * Other.W
				- X * Other.X
				- Y * Other.Y
				- Z * Other.Z,

			W * Other.X
				+ X * Other.W
				+ Y * Other.Z
				- Z * Other.Y,

			W * Other.Y
				- X * Other.Z
				+ Y * Other.W
				+ Z * Other.X,

			W * Other.Z
				+ X * Other.Y
				- Y * Other.X
				+ Z * Other.W
		};
	}

	FVector3 FQuaternion::RotateVector(
		const FVector3& Vector) const
	{
		FQuaternion UnitRotation = *this;

		if (!UnitRotation.IsNormalized())
		{
			UnitRotation.Normalize();
		}

		const FQuaternion VectorQuaternion{
			0.0,
			Vector.X,
			Vector.Y,
			Vector.Z
		};

		const FQuaternion RotatedQuaternion =
			UnitRotation
			* VectorQuaternion
			* UnitRotation.Conjugate();

		return FVector3{
			RotatedQuaternion.X,
			RotatedQuaternion.Y,
			RotatedQuaternion.Z
		};
	}

	FVector3 FQuaternion::InverseRotateVector(
		const FVector3& Vector) const
	{
		FQuaternion UnitRotation = *this;

		if (!UnitRotation.IsNormalized())
		{
			UnitRotation.Normalize();
		}

		const FQuaternion VectorQuaternion{
			0.0,
			Vector.X,
			Vector.Y,
			Vector.Z
		};

		const FQuaternion RotatedQuaternion =
			UnitRotation.Conjugate()
			* VectorQuaternion
			* UnitRotation;

		return FVector3{
			RotatedQuaternion.X,
			RotatedQuaternion.Y,
			RotatedQuaternion.Z
		};
	}
}