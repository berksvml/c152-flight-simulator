#include "FlightDynamics/C152Simulation.h"

#include <cmath>

namespace C152::FlightDynamics
{
    FC152Simulation::FC152Simulation()
    {
        Reset();
    }

    void FC152Simulation::Reset()
    {
        Reset(FAircraftState{});
    }

    void FC152Simulation::Reset(
        const FAircraftState& InitialAircraftState)
    {
        AircraftState = InitialAircraftState;
        AircraftState.AttitudeBodyToNed.Normalize();

        ControlSurfaceModel.Reset();
        SimulationClock.Reset();

        bLastDynamicsStepSuccessful = true;
    }

    bool FC152Simulation::SetEnvironmentConfiguration(
        const FEnvironmentConfiguration& Configuration)
    {
        const double OriginAltitude =
            Configuration.OriginGeopotentialAltitudeMeters;

        if (!std::isfinite(OriginAltitude)
            || OriginAltitude < 0.0
            || OriginAltitude
            > FStandardAtmosphere::MaxGeopotentialAltitudeMeters
            || !Configuration
            .WindVelocityNedMetersPerSecond.IsFinite())
        {
            return false;
        }

        EnvironmentConfiguration = Configuration;
        bEnvironmentConfigured = true;

        return true;
    }

    void FC152Simulation::ClearEnvironmentConfiguration()
    {
        EnvironmentConfiguration = FEnvironmentConfiguration{};
        bEnvironmentConfigured = false;
    }

    bool FC152Simulation::IsEnvironmentConfigured() const
    {
        return bEnvironmentConfigured;
    }

    bool FC152Simulation::TryGetEnvironmentSample(
        FAtmosphereState& OutAtmosphere,
        FAirData& OutAirData) const
    {
        if (!bEnvironmentConfigured)
        {
            return false;
        }

        // Local flat-Earth approximation: NED down displacement decreases
        // altitude above mean sea level.
        const double AltitudeMeters =
            EnvironmentConfiguration.OriginGeopotentialAltitudeMeters
            - AircraftState.PositionNedMeters.Z;

        FAtmosphereState AtmosphereSample{};

        if (!FStandardAtmosphere::TryEvaluate(
            AltitudeMeters,
            AtmosphereSample))
        {
            return false;
        }

        FAirData AirDataSample{};

        if (!FAirDataModel::TryEvaluate(
            AircraftState,
            AtmosphereSample,
            EnvironmentConfiguration
            .WindVelocityNedMetersPerSecond,
            AirDataSample))
        {
            return false;
        }

        OutAtmosphere = AtmosphereSample;
        OutAirData = AirDataSample;

        return true;
    }

    bool FC152Simulation::SetDynamicsConfiguration(
        const FC152SimulationConfiguration& Configuration)
    {
        if (!Configuration.MassProperties.IsValid()
            || !Configuration
            .GravityAccelerationNedMetersPerSecondSquared
            .IsFinite())
        {
            return false;
        }

        DynamicsConfiguration = Configuration;
        bDynamicsConfigured = true;
        bLastDynamicsStepSuccessful = true;

        return true;
    }

    void FC152Simulation::ClearDynamicsConfiguration()
    {
        DynamicsConfiguration =
            FC152SimulationConfiguration{};

        AppliedBodyLoads =
            FBodyForcesAndMoments{};

        bDynamicsConfigured = false;
        bLastDynamicsStepSuccessful = true;
    }

    bool FC152Simulation::IsDynamicsConfigured() const
    {
        return bDynamicsConfigured;
    }

    void FC152Simulation::SetAppliedBodyLoads(
        const FBodyForcesAndMoments& AppliedLoads)
    {
        AppliedBodyLoads = AppliedLoads;
    }

    std::uint32_t FC152Simulation::Advance(
        const FControlCommand& Command,
        const double FrameDeltaSeconds)
    {
        const std::uint32_t StepCount =
            SimulationClock.Advance(FrameDeltaSeconds);

        const double FixedDeltaSeconds =
            SimulationClock.GetFixedDeltaSeconds();

        bLastDynamicsStepSuccessful = true;

        for (
            std::uint32_t StepIndex = 0U;
            StepIndex < StepCount;
            ++StepIndex)
        {
            if (!Step(Command, FixedDeltaSeconds))
            {
                bLastDynamicsStepSuccessful = false;
                break;
            }
        }

        return StepCount;
    }

    const FAircraftState&
        FC152Simulation::GetAircraftState() const
    {
        return AircraftState;
    }

    const FControlSurfaceState&
        FC152Simulation::GetControlSurfaceState() const
    {
        return ControlSurfaceModel.GetState();
    }

    double FC152Simulation::GetFixedDeltaSeconds() const
    {
        return SimulationClock.GetFixedDeltaSeconds();
    }

    bool FC152Simulation::
        WasLastDynamicsStepSuccessful() const
    {
        return bLastDynamicsStepSuccessful;
    }

    bool FC152Simulation::Step(
        const FControlCommand& Command,
        const double FixedDeltaSeconds)
    {
        ControlSurfaceModel.Update(
            Command,
            FixedDeltaSeconds);

        if (!bDynamicsConfigured)
        {
            return true;
        }

        return RigidBodyModel.Propagate(
            AircraftState,
            AppliedBodyLoads,
            DynamicsConfiguration.MassProperties,
            DynamicsConfiguration
            .GravityAccelerationNedMetersPerSecondSquared,
            FixedDeltaSeconds);
    }
}