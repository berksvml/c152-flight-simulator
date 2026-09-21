#pragma once

#include "FlightDynamics/AircraftMassProperties.h"
#include "FlightDynamics/C152ControlSurfaceModel.h"
#include "FlightDynamics/FixedStepClock.h"
#include "FlightDynamics/FlightDynamicsTypes.h"
#include "FlightDynamics/RigidBody6DofModel.h"
#include "FlightDynamics/AirDataModel.h"
#include "FlightDynamics/StandardAtmosphere.h"
#include "FlightDynamics/WindModel.h"

#include <cstdint>

namespace C152::FlightDynamics
{
    struct FEnvironmentConfiguration
    {
        // Geopotential altitude of the local NED origin above mean sea level [m].
        double OriginGeopotentialAltitudeMeters = 0.0;

        // Mean air-mass velocity in Navigation/NED axes [m/s].
        FVector3 WindVelocityNedMetersPerSecond{};

        // Standard deviation of zero-mean turbulence in NED axes [m/s].
        FVector3 TurbulenceStandardDeviationNedMetersPerSecond{};

        // Shared first-order turbulence correlation time [s].
        double TurbulenceCorrelationTimeSeconds = 1.0;

        // Equal seeds produce equal turbulence sequences.
        std::uint32_t TurbulenceRandomSeed = 1U;
    };

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

        [[nodiscard]]
        bool SetEnvironmentConfiguration(
            const FEnvironmentConfiguration& Configuration);

        void ClearEnvironmentConfiguration();

        [[nodiscard]]
        bool IsEnvironmentConfigured() const;

        // Calculates data from the current aircraft state.
        // On failure, neither output is changed.
        [[nodiscard]]
        bool TryGetEnvironmentSample(
            FAtmosphereState& OutAtmosphere,
            FAirData& OutAirData) const;

        [[nodiscard]]
        const FVector3&
            GetWindVelocityNedMetersPerSecond() const;

        [[nodiscard]]
        const FVector3&
            GetTurbulenceVelocityNedMetersPerSecond() const;

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
        FEnvironmentConfiguration EnvironmentConfiguration{};
        FWindModel WindModel;

        bool bEnvironmentConfigured = false;
        bool bDynamicsConfigured = false;
        bool bLastDynamicsStepSuccessful = true;
    };
}