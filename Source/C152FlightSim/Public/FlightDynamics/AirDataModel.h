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
    };

    class FAirDataModel final
    {
    public:
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