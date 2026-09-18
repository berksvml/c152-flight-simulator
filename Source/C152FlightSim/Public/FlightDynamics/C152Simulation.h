#pragma once

#include "FlightDynamics/AircraftMassProperties.h"
#include "FlightDynamics/C152ControlSurfaceModel.h"
#include "FlightDynamics/FixedStepClock.h"
#include "FlightDynamics/FlightDynamicsTypes.h"
#include "FlightDynamics/RigidBody6DofModel.h"

#include <cstdint>

namespace C152::FlightDynamics
{
    struct FC152SimulationConfiguration
    {
        FAircraftMassProperties MassProperties{};

        FVector3 GravityAccelerationNedMetersPerSecondSquared{
            0.0,
            0.0,
            9.80665
        };
    };

    class FC152Simulation final
    {
    public:
        FC152Simulation();

        void Reset();

        void Reset(
            const FAircraftState& InitialAircraftState);

        [[nodiscard]]
        bool SetDynamicsConfiguration(
            const FC152SimulationConfiguration& Configuration);

        void ClearDynamicsConfiguration();

        [[nodiscard]]
        bool IsDynamicsConfigured() const;

        void SetAppliedBodyLoads(
            const FBodyForcesAndMoments& AppliedLoads);

        [[nodiscard]]
        std::uint32_t Advance(
            const FControlCommand& Command,
            double FrameDeltaSeconds);

        [[nodiscard]]
        const FAircraftState& GetAircraftState() const;

        [[nodiscard]]
        const FControlSurfaceState& GetControlSurfaceState() const;

        [[nodiscard]]
        double GetFixedDeltaSeconds() const;

        [[nodiscard]]
        bool WasLastDynamicsStepSuccessful() const;

    private:
        [[nodiscard]]
        bool Step(
            const FControlCommand& Command,
            double FixedDeltaSeconds);

        FAircraftState AircraftState{};

        FC152ControlSurfaceModel ControlSurfaceModel;
        FFixedStepClock SimulationClock;
        FRigidBody6DofModel RigidBodyModel;

        FC152SimulationConfiguration DynamicsConfiguration{};
        FBodyForcesAndMoments AppliedBodyLoads{};

        bool bDynamicsConfigured = false;
        bool bLastDynamicsStepSuccessful = true;
    };
}