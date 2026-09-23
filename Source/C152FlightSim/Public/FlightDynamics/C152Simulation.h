#pragma once

#include "FlightDynamics/AerodynamicModel.h"
#include "FlightDynamics/AircraftMassProperties.h"
#include "FlightDynamics/AirDataModel.h"
#include "FlightDynamics/C152ControlSurfaceModel.h"
#include "FlightDynamics/FixedStepClock.h"
#include "FlightDynamics/FlightDynamicsTypes.h"
#include "FlightDynamics/GroundReactionModel.h"
#include "FlightDynamics/PowerplantModel.h"
#include "FlightDynamics/RigidBody6DofModel.h"
#include "FlightDynamics/StandardAtmosphere.h"
#include "FlightDynamics/WindModel.h"

#include <cstdint>

namespace C152::FlightDynamics
{
    struct FEnvironmentConfiguration
    {
        // Geopotential altitude of the local NED origin
        // above mean sea level [m].
        double OriginGeopotentialAltitudeMeters = 0.0;

        // Mean air-mass velocity in Navigation/NED axes [m/s].
        FVector3 WindVelocityNedMetersPerSecond{};

        // Standard deviation of zero-mean turbulence
        // in Navigation/NED axes [m/s].
        FVector3
            TurbulenceStandardDeviationNedMetersPerSecond{};

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

    struct FC152AircraftModelConfiguration
    {
        FAerodynamicModelConfiguration Aerodynamics{};
        FPowerplantModelConfiguration Powerplant{};
        FGroundReactionModelConfiguration GroundReaction{};

        // Initial usable fuel assigned to the powerplant model [kg].
        double InitialUsableFuelKilograms = 0.0;

        // Down coordinate of a flat horizontal runway [m].
        double GroundPlaneDownMeters = 0.0;
    };

    struct FC152AircraftModelStepOutput
    {
        FAtmosphereState Atmosphere{};
        FAirData AirData{};

        FAerodynamicCoefficients AerodynamicCoefficients{};
        FPowerplantOutput Powerplant{};
        FGroundReactionResult GroundReaction{};

        FBodyForcesAndMoments AerodynamicLoads{};
        FBodyForcesAndMoments PowerplantLoads{};
        FBodyForcesAndMoments GroundReactionLoads{};
        FBodyForcesAndMoments TotalBodyLoads{};

        bool bValid = false;
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

        // External loads may be used for tests or additional systems.
        // When aircraft models are enabled, these loads are added
        // to aerodynamic, powerplant, and ground-reaction loads.
        void SetAppliedBodyLoads(
            const FBodyForcesAndMoments& AppliedLoads);

        [[nodiscard]]
        bool SetAircraftModelConfiguration(
            const FC152AircraftModelConfiguration& Configuration);

        void ClearAircraftModelConfiguration();

        [[nodiscard]]
        bool IsAircraftModelConfigured() const;

        [[nodiscard]]
        bool SetBrakeCommand(
            double NewBrakeCommand);

        [[nodiscard]]
        double GetBrakeCommand() const;

        [[nodiscard]]
        std::uint32_t Advance(
            const FControlCommand& Command,
            double FrameDeltaSeconds);

        [[nodiscard]]
        const FAircraftState& GetAircraftState() const;

        [[nodiscard]]
        const FControlSurfaceState&
            GetControlSurfaceState() const;

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

        [[nodiscard]]
        const FC152AircraftModelStepOutput&
            GetLastAircraftModelStepOutput() const;

    private:
        [[nodiscard]]
        bool Step(
            const FControlCommand& Command,
            double FixedDeltaSeconds);

        [[nodiscard]]
        bool TryEvaluateEnvironment(
            const FAircraftState& State,
            const FVector3& WindVelocityNedMetersPerSecond,
            FAtmosphereState& OutAtmosphere,
            FAirData& OutAirData) const;

        [[nodiscard]]
        bool ResetPowerplantState();

        static void AddBodyLoads(
            FBodyForcesAndMoments& InOutTotal,
            const FBodyForcesAndMoments& LoadsToAdd);

        FAircraftState AircraftState{};

        FC152ControlSurfaceModel ControlSurfaceModel;
        FFixedStepClock SimulationClock;
        FRigidBody6DofModel RigidBodyModel;

        FAerodynamicModel AerodynamicModel;
        FPowerplantModel PowerplantModel;
        FGroundReactionModel GroundReactionModel;

        FC152SimulationConfiguration
            DynamicsConfiguration{};

        FC152AircraftModelConfiguration
            AircraftModelConfiguration{};

        FBodyForcesAndMoments AppliedBodyLoads{};

        FEnvironmentConfiguration
            EnvironmentConfiguration{};

        FC152AircraftModelStepOutput
            LastAircraftModelStepOutput{};

        FWindModel WindModel;

        double BrakeCommand = 0.0;

        bool bEnvironmentConfigured = false;
        bool bDynamicsConfigured = false;
        bool bAircraftModelConfigured = false;
        bool bLastDynamicsStepSuccessful = true;
    };
}