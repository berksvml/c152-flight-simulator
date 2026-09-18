#include "FlightDynamics/RigidBody6DofModel.h"

#include <cmath>

namespace
{
    constexpr double MinimumQuaternionNorm = 1.0e-12;

    bool IsFiniteQuaternion(
        const C152::FlightDynamics::FQuaternion& Quaternion)
    {
        return std::isfinite(Quaternion.W)
            && std::isfinite(Quaternion.X)
            && std::isfinite(Quaternion.Y)
            && std::isfinite(Quaternion.Z);
    }

    bool IsValidQuaternion(
        const C152::FlightDynamics::FQuaternion& Quaternion)
    {
        return IsFiniteQuaternion(Quaternion)
            && Quaternion.Norm() > MinimumQuaternionNorm;
    }

    C152::FlightDynamics::FQuaternion NormalizeQuaternion(
        const C152::FlightDynamics::FQuaternion& Quaternion)
    {
        const double Norm = Quaternion.Norm();

        return {
            Quaternion.W / Norm,
            Quaternion.X / Norm,
            Quaternion.Y / Norm,
            Quaternion.Z / Norm
        };
    }

    bool IsValidState(
        const C152::FlightDynamics::FAircraftState& State)
    {
        return State.PositionNedMeters.IsFinite()
            && State.VelocityBodyMetersPerSecond.IsFinite()
            && State.AngularRateBodyRadiansPerSecond.IsFinite()
            && IsValidQuaternion(State.AttitudeBodyToNed);
    }

    bool IsValidLoads(
        const C152::FlightDynamics::FBodyForcesAndMoments& Loads)
    {
        return Loads.ForceBodyNewtons.IsFinite()
            && Loads.MomentBodyNewtonMeters.IsFinite();
    }

    C152::FlightDynamics::FQuaternion MakeAttitudeIncrement(
        const C152::FlightDynamics::FVector3& AngularRateBodyRadiansPerSecond,
        const double DeltaTimeSeconds)
    {
        using namespace C152::FlightDynamics;

        const FVector3 RotationVector =
            AngularRateBodyRadiansPerSecond * DeltaTimeSeconds;

        const double RotationAngle = RotationVector.Norm();

        if (RotationAngle < 1.0e-10)
        {
            return NormalizeQuaternion({
                1.0,
                0.5 * RotationVector.X,
                0.5 * RotationVector.Y,
                0.5 * RotationVector.Z
                });
        }

        const double HalfAngle = 0.5 * RotationAngle;
        const double VectorScale =
            std::sin(HalfAngle) / RotationAngle;

        return {
            std::cos(HalfAngle),
            RotationVector.X * VectorScale,
            RotationVector.Y * VectorScale,
            RotationVector.Z * VectorScale
        };
    }
}

namespace C152::FlightDynamics
{
    bool FRigidBody6DofModel::Propagate(
        FAircraftState& InOutState,
        const FBodyForcesAndMoments& AppliedLoads,
        const FAircraftMassProperties& MassProperties,
        const FVector3& GravityAccelerationNedMetersPerSecondSquared,
        const double DeltaTimeSeconds) const
    {
        if (!std::isfinite(DeltaTimeSeconds)
            || DeltaTimeSeconds < 0.0
            || !MassProperties.IsValid()
            || !GravityAccelerationNedMetersPerSecondSquared.IsFinite()
            || !IsValidLoads(AppliedLoads)
            || !IsValidState(InOutState))
        {
            return false;
        }

        if (DeltaTimeSeconds == 0.0)
        {
            return true;
        }

        FAircraftState NextState = InOutState;

        const FQuaternion CurrentAttitude =
            NormalizeQuaternion(InOutState.AttitudeBodyToNed);

        const FVector3 CurrentVelocityBody =
            InOutState.VelocityBodyMetersPerSecond;

        const FVector3 CurrentAngularRateBody =
            InOutState.AngularRateBodyRadiansPerSecond;

        const FVector3 CurrentVelocityNed =
            CurrentAttitude.RotateVector(CurrentVelocityBody);

        const FVector3 GravityAccelerationBody =
            CurrentAttitude.InverseRotateVector(
                GravityAccelerationNedMetersPerSecondSquared);

        const FVector3 TranslationalAccelerationBody =
            AppliedLoads.ForceBodyNewtons
            / MassProperties.MassKilograms
            + GravityAccelerationBody
            - Cross(
                CurrentAngularRateBody,
                CurrentVelocityBody);

        const FVector3 AngularMomentumBody =
            MassProperties.ApplyInertia(
                CurrentAngularRateBody);

        const FVector3 EffectiveMomentBody =
            AppliedLoads.MomentBodyNewtonMeters
            - Cross(
                CurrentAngularRateBody,
                AngularMomentumBody);

        FVector3 AngularAccelerationBody{};

        if (!MassProperties.TryApplyInverseInertia(
            EffectiveMomentBody,
            AngularAccelerationBody))
        {
            return false;
        }

        NextState.VelocityBodyMetersPerSecond =
            CurrentVelocityBody
            + TranslationalAccelerationBody
            * DeltaTimeSeconds;

        NextState.AngularRateBodyRadiansPerSecond =
            CurrentAngularRateBody
            + AngularAccelerationBody
            * DeltaTimeSeconds;

        const FVector3 AverageAngularRateBody =
            (CurrentAngularRateBody
                + NextState.AngularRateBodyRadiansPerSecond)
            * 0.5;

        const FQuaternion AttitudeIncrement =
            MakeAttitudeIncrement(
                AverageAngularRateBody,
                DeltaTimeSeconds);

        NextState.AttitudeBodyToNed =
            NormalizeQuaternion(
                CurrentAttitude * AttitudeIncrement);

        const FVector3 NextVelocityNed =
            NextState.AttitudeBodyToNed.RotateVector(
                NextState.VelocityBodyMetersPerSecond);

        NextState.PositionNedMeters +=
            (CurrentVelocityNed + NextVelocityNed)
            * (0.5 * DeltaTimeSeconds);

        if (!IsValidState(NextState))
        {
            return false;
        }

        InOutState = NextState;
        return true;
    }
}