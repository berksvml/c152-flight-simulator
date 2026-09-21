#include "FlightDynamics/AirDataModel.h"

#include <algorithm>
#include <cmath>

namespace C152::FlightDynamics
{
    bool FAirDataModel::TryEvaluate(
        const FAircraftState& AircraftState,
        const FAtmosphereState& Atmosphere,
        const FVector3& WindVelocityNedMetersPerSecond,
        FAirData& OutAirData)
    {
        const double Density =
            Atmosphere.DensityKilogramsPerCubicMeter;

        const double SpeedOfSound =
            Atmosphere.SpeedOfSoundMetersPerSecond;

        if (!AircraftState.VelocityBodyMetersPerSecond.IsFinite()
            || !AircraftState.AttitudeBodyToNed.IsNormalized(1.0e-6)
            || !WindVelocityNedMetersPerSecond.IsFinite()
            || !std::isfinite(Density)
            || Density <= 0.0
            || !std::isfinite(SpeedOfSound)
            || SpeedOfSound <= 0.0)
        {
            return false;
        }

        const FVector3 WindVelocityBody =
            AircraftState.AttitudeBodyToNed.InverseRotateVector(
                WindVelocityNedMetersPerSecond);

        const FVector3 RelativeVelocityBody =
            AircraftState.VelocityBodyMetersPerSecond
            - WindVelocityBody;

        if (!RelativeVelocityBody.IsFinite())
        {
            return false;
        }

        const double TrueAirspeed =
            RelativeVelocityBody.Norm();

        const double DynamicPressure =
            0.5 * Density * TrueAirspeed * TrueAirspeed;

        const double MachNumber =
            TrueAirspeed / SpeedOfSound;

        if (!std::isfinite(TrueAirspeed)
            || !std::isfinite(DynamicPressure)
            || !std::isfinite(MachNumber))
        {
            return false;
        }

        double AngleOfAttack = 0.0;
        double SideslipAngle = 0.0;

        // Flow angles are undefined at zero airspeed.
        if (TrueAirspeed > 1.0e-6)
        {
            // FRD axes: X forward, Y right, Z down.
            AngleOfAttack = std::atan2(
                RelativeVelocityBody.Z,
                RelativeVelocityBody.X);

            SideslipAngle = std::asin(std::clamp(
                RelativeVelocityBody.Y / TrueAirspeed,
                -1.0,
                1.0));
        }

        OutAirData = FAirData{
            RelativeVelocityBody,
            TrueAirspeed,
            AngleOfAttack,
            SideslipAngle,
            DynamicPressure,
            MachNumber
        };

        return true;
    }
}