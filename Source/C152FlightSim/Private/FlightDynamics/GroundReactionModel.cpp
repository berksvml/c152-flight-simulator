#include "FlightDynamics/GroundReactionModel.h"

#include <algorithm>
#include <cmath>

namespace GroundReactionModelConstants
{
	constexpr double MinimumPositiveValue =
		1.0e-12;

	bool IsQuaternionFinite(
		const C152::FlightDynamics::FQuaternion& Quaternion)
	{
		return std::isfinite(Quaternion.W)
			&& std::isfinite(Quaternion.X)
			&& std::isfinite(Quaternion.Y)
			&& std::isfinite(Quaternion.Z);
	}
}

namespace C152::FlightDynamics
{
	bool FGroundContactPointConfiguration::IsValid() const
	{
		const bool bValuesAreFinite =
			PositionBodyMeters.IsFinite()
			&& std::isfinite(
				SpringStiffnessNewtonsPerMeter)
			&& std::isfinite(
				DampingCoefficientNewtonSecondsPerMeter)
			&& std::isfinite(
				RollingResistanceCoefficient)
			&& std::isfinite(
				MaximumBrakingFrictionCoefficient)
			&& std::isfinite(BrakingAuthority);

		if (!bValuesAreFinite)
		{
			return false;
		}

		return SpringStiffnessNewtonsPerMeter
				> GroundReactionModelConstants::
			MinimumPositiveValue
			&& DampingCoefficientNewtonSecondsPerMeter
			>= 0.0
			&& RollingResistanceCoefficient >= 0.0
			&& MaximumBrakingFrictionCoefficient >= 0.0
			&& BrakingAuthority >= 0.0
			&& BrakingAuthority <= 1.0;
	}

	bool FGroundReactionModelConfiguration::IsValid() const
	{
		if (!std::isfinite(
			FrictionTransitionSpeedMetersPerSecond)
			|| FrictionTransitionSpeedMetersPerSecond
			<= GroundReactionModelConstants::
			MinimumPositiveValue)
		{
			return false;
		}

		for (const FGroundContactPointConfiguration&
			ContactPoint : ContactPoints)
		{
			if (!ContactPoint.IsValid())
			{
				return false;
			}
		}

		return true;
	}

	bool FGroundReactionModel::SetConfiguration(
		const FGroundReactionModelConfiguration&
		NewConfiguration)
	{
		if (!NewConfiguration.IsValid())
		{
			return false;
		}

		Configuration = NewConfiguration;
		bIsConfigured = true;

		return true;
	}

	void FGroundReactionModel::ClearConfiguration()
	{
		Configuration = {};
		bIsConfigured = false;
	}

	bool FGroundReactionModel::IsConfigured() const
	{
		return bIsConfigured;
	}

	bool FGroundReactionModel::TryEvaluate(
		const FAircraftState& AircraftState,
		const double GroundPlaneDownMeters,
		const double BrakeCommand,
		FBodyForcesAndMoments& OutLoads,
		FGroundReactionResult& OutResult) const
	{
		const bool bInputsAreFinite =
			AircraftState.PositionNedMeters.IsFinite()
			&& AircraftState
			.VelocityBodyMetersPerSecond.IsFinite()
			&& AircraftState
			.AngularRateBodyRadiansPerSecond.IsFinite()
			&& GroundReactionModelConstants::
			IsQuaternionFinite(
				AircraftState.AttitudeBodyToNed)
			&& std::isfinite(GroundPlaneDownMeters)
			&& std::isfinite(BrakeCommand);

		if (!bIsConfigured
			|| !bInputsAreFinite
			|| BrakeCommand < 0.0
			|| BrakeCommand > 1.0
			|| AircraftState.AttitudeBodyToNed.NormSquared()
			<= GroundReactionModelConstants::
			MinimumPositiveValue)
		{
			return false;
		}

		FBodyForcesAndMoments Loads{};
		FGroundReactionResult Result{};

		for (const FGroundContactPointConfiguration&
			ContactPoint : Configuration.ContactPoints)
		{
			const FVector3 ContactPositionNedMeters =
				AircraftState.PositionNedMeters
				+ AircraftState.AttitudeBodyToNed
				.RotateVector(
					ContactPoint.PositionBodyMeters);

			const double PenetrationMeters =
				ContactPositionNedMeters.Z
				- GroundPlaneDownMeters;

			if (PenetrationMeters <= 0.0)
			{
				continue;
			}

			const FVector3 ContactVelocityBody =
				AircraftState
				.VelocityBodyMetersPerSecond
				+ Cross(
					AircraftState
					.AngularRateBodyRadiansPerSecond,
					ContactPoint.PositionBodyMeters);

			const FVector3 ContactVelocityNed =
				AircraftState.AttitudeBodyToNed
				.RotateVector(ContactVelocityBody);

			// Positive NED Z velocity means motion into the runway.
			const double NormalForceNewtons =
				std::max(
					0.0,
					ContactPoint
					.SpringStiffnessNewtonsPerMeter
					* PenetrationMeters
					+ ContactPoint
					.DampingCoefficientNewtonSecondsPerMeter
					* ContactVelocityNed.Z);

			if (NormalForceNewtons
				<= GroundReactionModelConstants::
				MinimumPositiveValue)
			{
				continue;
			}

			const double HorizontalSpeedMetersPerSecond =
				std::sqrt(
					ContactVelocityNed.X
					* ContactVelocityNed.X
					+ ContactVelocityNed.Y
					* ContactVelocityNed.Y);

			FVector3 ContactForceNed{
				0.0,
				0.0,
				-NormalForceNewtons
			};

			if (HorizontalSpeedMetersPerSecond
				> GroundReactionModelConstants::
				MinimumPositiveValue)
			{
				const double FrictionCoefficient =
					ContactPoint
					.RollingResistanceCoefficient
					+ BrakeCommand
					* ContactPoint.BrakingAuthority
					* ContactPoint
					.MaximumBrakingFrictionCoefficient;

				const double FrictionTransitionFactor =
					std::min(
						1.0,
						HorizontalSpeedMetersPerSecond
						/ Configuration
						.FrictionTransitionSpeedMetersPerSecond);

				const double FrictionForceNewtons =
					NormalForceNewtons
					* FrictionCoefficient
					* FrictionTransitionFactor;

				ContactForceNed.X =
					-FrictionForceNewtons
					* ContactVelocityNed.X
					/ HorizontalSpeedMetersPerSecond;

				ContactForceNed.Y =
					-FrictionForceNewtons
					* ContactVelocityNed.Y
					/ HorizontalSpeedMetersPerSecond;
			}

			const FVector3 ContactForceBody =
				AircraftState.AttitudeBodyToNed
				.InverseRotateVector(
					ContactForceNed);

			Loads.ForceBodyNewtons +=
				ContactForceBody;

			Loads.MomentBodyNewtonMeters +=
				Cross(
					ContactPoint.PositionBodyMeters,
					ContactForceBody);

			Result.TotalNormalForceNewtons +=
				NormalForceNewtons;

			++Result.ActiveContactCount;
		}

		Result.bOnGround =
			Result.ActiveContactCount > 0;

		if (!Loads.ForceBodyNewtons.IsFinite()
			|| !Loads.MomentBodyNewtonMeters.IsFinite()
			|| !std::isfinite(
				Result.TotalNormalForceNewtons))
		{
			return false;
		}

		OutLoads = Loads;
		OutResult = Result;

		return true;
	}
}