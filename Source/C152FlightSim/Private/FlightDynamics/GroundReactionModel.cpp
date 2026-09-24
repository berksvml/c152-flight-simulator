#include "FlightDynamics/GroundReactionModel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

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
			&& DampingCoefficientNewtonSecondsPerMeter >= 0.0
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
		return TryEvaluate(
			AircraftState,
			GroundPlaneDownMeters,
			BrakeCommand,
			FBodyForcesAndMoments{},
			OutLoads,
			OutResult);
	}

	bool FGroundReactionModel::TryEvaluate(
		const FAircraftState& AircraftState,
		const double GroundPlaneDownMeters,
		const double BrakeCommand,
		const FBodyForcesAndMoments&
		AppliedLoadsWithoutGround,
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
			&& std::isfinite(BrakeCommand)
			&& AppliedLoadsWithoutGround
			.ForceBodyNewtons.IsFinite()
			&& AppliedLoadsWithoutGround
			.MomentBodyNewtonMeters.IsFinite();

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

		constexpr std::size_t ContactPointCount = 3U;

		std::array<bool, ContactPointCount>
			bActiveContacts{};

		std::array<FVector3, ContactPointCount>
			ContactVelocitiesNed{};

		std::array<double, ContactPointCount>
			NormalForcesNewtons{};

		std::array<double, ContactPointCount>
			BrakingCapacitiesNewtons{};

		FGroundReactionResult Result{};

		double TotalBrakingCapacityNewtons = 0.0;
		double MaximumContactHorizontalSpeed = 0.0;

		// First pass: determine active contacts, normal forces
		// and available braking capacity.
		for (
			std::size_t ContactIndex = 0U;
			ContactIndex
			< Configuration.ContactPoints.size();
			++ContactIndex)
		{
			const FGroundContactPointConfiguration&
				ContactPoint =
				Configuration.ContactPoints[ContactIndex];

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

			// Positive NED Z velocity means motion
			// into the runway.
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

			const double BrakingCapacityNewtons =
				NormalForceNewtons
				* BrakeCommand
				* ContactPoint.BrakingAuthority
				* ContactPoint
				.MaximumBrakingFrictionCoefficient;

			bActiveContacts[ContactIndex] = true;

			ContactVelocitiesNed[ContactIndex] =
				ContactVelocityNed;

			NormalForcesNewtons[ContactIndex] =
				NormalForceNewtons;

			BrakingCapacitiesNewtons[ContactIndex] =
				BrakingCapacityNewtons;

			Result.TotalNormalForceNewtons +=
				NormalForceNewtons;

			++Result.ActiveContactCount;

			TotalBrakingCapacityNewtons +=
				BrakingCapacityNewtons;

			MaximumContactHorizontalSpeed =
				std::max(
					MaximumContactHorizontalSpeed,
					HorizontalSpeedMetersPerSecond);
		}

		Result.bOnGround =
			Result.ActiveContactCount > 0;

		const bool bUseStaticBraking =
			Result.bOnGround
			&& BrakeCommand
			> GroundReactionModelConstants::
			MinimumPositiveValue
			&& TotalBrakingCapacityNewtons
			> GroundReactionModelConstants::
			MinimumPositiveValue
			&& MaximumContactHorizontalSpeed
			< Configuration
			.FrictionTransitionSpeedMetersPerSecond;

		const FVector3 AppliedForceNed =
			AircraftState.AttitudeBodyToNed
			.RotateVector(
				AppliedLoadsWithoutGround
				.ForceBodyNewtons);

		FBodyForcesAndMoments Loads{};

		// Second pass: calculate normal, rolling-resistance
		// and brake forces for every active contact.
		for (
			std::size_t ContactIndex = 0U;
			ContactIndex
			< Configuration.ContactPoints.size();
			++ContactIndex)
		{
			if (!bActiveContacts[ContactIndex])
			{
				continue;
			}

			const FGroundContactPointConfiguration&
				ContactPoint =
				Configuration.ContactPoints[ContactIndex];

			const FVector3& ContactVelocityNed =
				ContactVelocitiesNed[ContactIndex];

			const double NormalForceNewtons =
				NormalForcesNewtons[ContactIndex];

			const double BrakingCapacityNewtons =
				BrakingCapacitiesNewtons[ContactIndex];

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
				const double FrictionTransitionFactor =
					std::min(
						1.0,
						HorizontalSpeedMetersPerSecond
						/ Configuration
						.FrictionTransitionSpeedMetersPerSecond);

				const double RollingResistanceForceNewtons =
					NormalForceNewtons
					* ContactPoint
					.RollingResistanceCoefficient
					* FrictionTransitionFactor;

				ContactForceNed.X -=
					RollingResistanceForceNewtons
					* ContactVelocityNed.X
					/ HorizontalSpeedMetersPerSecond;

				ContactForceNed.Y -=
					RollingResistanceForceNewtons
					* ContactVelocityNed.Y
					/ HorizontalSpeedMetersPerSecond;
			}

			if (bUseStaticBraking
				&& BrakingCapacityNewtons
			> GroundReactionModelConstants::
				MinimumPositiveValue)
			{
				const double CapacityFraction =
					BrakingCapacityNewtons
					/ TotalBrakingCapacityNewtons;

				// Cancel the corresponding share of external
				// horizontal force and damp residual wheel motion.
				double DesiredBrakeForceX =
					-AppliedForceNed.X
					* CapacityFraction
					- BrakingCapacityNewtons
					* ContactVelocityNed.X
					/ Configuration
					.FrictionTransitionSpeedMetersPerSecond;

				double DesiredBrakeForceY =
					-AppliedForceNed.Y
					* CapacityFraction
					- BrakingCapacityNewtons
					* ContactVelocityNed.Y
					/ Configuration
					.FrictionTransitionSpeedMetersPerSecond;

				const double DesiredBrakeForceMagnitude =
					std::sqrt(
						DesiredBrakeForceX
						* DesiredBrakeForceX
						+ DesiredBrakeForceY
						* DesiredBrakeForceY);

				if (DesiredBrakeForceMagnitude
					> BrakingCapacityNewtons)
				{
					const double Scale =
						BrakingCapacityNewtons
						/ DesiredBrakeForceMagnitude;

					DesiredBrakeForceX *= Scale;
					DesiredBrakeForceY *= Scale;
				}

				ContactForceNed.X +=
					DesiredBrakeForceX;

				ContactForceNed.Y +=
					DesiredBrakeForceY;
			}
			else if (
				HorizontalSpeedMetersPerSecond
				> GroundReactionModelConstants::
				MinimumPositiveValue
				&& BrakingCapacityNewtons
				> GroundReactionModelConstants::
				MinimumPositiveValue)
			{
				const double FrictionTransitionFactor =
					std::min(
						1.0,
						HorizontalSpeedMetersPerSecond
						/ Configuration
						.FrictionTransitionSpeedMetersPerSecond);

				const double BrakingForceNewtons =
					BrakingCapacityNewtons
					* FrictionTransitionFactor;

				ContactForceNed.X -=
					BrakingForceNewtons
					* ContactVelocityNed.X
					/ HorizontalSpeedMetersPerSecond;

				ContactForceNed.Y -=
					BrakingForceNewtons
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
		}

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