#pragma once

#include "FlightDynamics/AircraftMassProperties.h"
#include "FlightDynamics/FlightDynamicsTypes.h"

namespace C152::FlightDynamics
{
    class FRigidBody6DofModel final
    {
    public:
        [[nodiscard]] bool Propagate(
            FAircraftState& InOutState,
            const FBodyForcesAndMoments& AppliedLoads,
            const FAircraftMassProperties& MassProperties,
            const FVector3& GravityAccelerationNedMetersPerSecondSquared,
            double DeltaTimeSeconds) const;
    };
}