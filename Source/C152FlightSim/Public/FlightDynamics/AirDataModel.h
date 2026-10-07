#pragma once

#include "FlightDynamics/FlightDynamicsTypes.h"
#include "FlightDynamics/StandardAtmosphere.h"

namespace C152::FlightDynamics
{
    struct FAirData
    {
        FVector3 RelativeVelocityBodyMetersPerSecond{};
        double TrueAirspeedMetersPerSecond = 0.0;
        double AngleOfAttackRadians = 0.0;
        double SideslipAngleRadians = 0.0;
        double DynamicPressurePascals = 0.0;
        double MachNumber = 0.0;

        // Flow direction is intentionally suppressed below the
        // model's minimum directional airspeed because alpha and beta
        // become numerically unstable as the velocity magnitude tends
        // to zero. Scalar air-data values remain valid.
        bool bFlowAnglesValid = false;
    };

    class FAirDataModel final
    {
    public:
        static constexpr double
            MinimumFlowAngleAirspeedMetersPerSecond = 0.5;

        // Wind is the velocity of the air mass in Navigation/NED axes.
        // Returns false without changing OutAirData if inputs are invalid.
        [[nodiscard]]
        static bool TryEvaluate(
            const FAircraftState& AircraftState,
            const FAtmosphereState& Atmosphere,
            const FVector3& WindVelocityNedMetersPerSecond,
            FAirData& OutAirData);
    };
}
