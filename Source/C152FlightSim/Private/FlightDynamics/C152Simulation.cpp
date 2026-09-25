#include "FlightDynamics/C152Simulation.h"

#include <algorithm>
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

        if (bEnvironmentConfigured)
        {
            WindModel.Reset();
        }

        if (bDynamicsConfigured)
        {
            CurrentMassProperties =
                DynamicsConfiguration.MassProperties;
        }
        else
        {
            CurrentMassProperties =
                FAircraftMassProperties{};
        }

        BrakeCommand = 0.0;

        LastAircraftModelStepOutput =
            FC152AircraftModelStepOutput{};

        bLastDynamicsStepSuccessful =
            ResetPowerplantState();
    }

    bool FC152Simulation::SetEnvironmentConfiguration(
        const FEnvironmentConfiguration& Configuration)
    {
        const double OriginAltitude =
            Configuration.OriginGeopotentialAltitudeMeters;

        if (!std::isfinite(OriginAltitude)
            || OriginAltitude < 0.0
            || OriginAltitude
            > FStandardAtmosphere::
            MaxGeopotentialAltitudeMeters)
        {
            return false;
        }

        FWindModelConfiguration WindConfiguration{};

        WindConfiguration
            .SteadyWindVelocityNedMetersPerSecond =
            Configuration.WindVelocityNedMetersPerSecond;

        WindConfiguration
            .TurbulenceStandardDeviationNedMetersPerSecond =
            Configuration
            .TurbulenceStandardDeviationNedMetersPerSecond;

        WindConfiguration.TurbulenceCorrelationTimeSeconds =
            Configuration.TurbulenceCorrelationTimeSeconds;

        WindConfiguration.RandomSeed =
            Configuration.TurbulenceRandomSeed;

        FWindModel CandidateWindModel;

        if (!CandidateWindModel.Configure(
            WindConfiguration))
        {
            return false;
        }

        EnvironmentConfiguration = Configuration;
        WindModel = CandidateWindModel;
        bEnvironmentConfigured = true;

        return true;
    }

    void FC152Simulation::ClearEnvironmentConfiguration()
    {
        EnvironmentConfiguration =
            FEnvironmentConfiguration{};

        WindModel.Clear();

        bEnvironmentConfigured = false;
    }

    bool FC152Simulation::IsEnvironmentConfigured() const
    {
        return bEnvironmentConfigured;
    }

    const FVector3&
        FC152Simulation::
        GetWindVelocityNedMetersPerSecond() const
    {
        return WindModel
            .GetWindVelocityNedMetersPerSecond();
    }

    const FVector3&
        FC152Simulation::
        GetTurbulenceVelocityNedMetersPerSecond() const
    {
        return WindModel
            .GetTurbulenceVelocityNedMetersPerSecond();
    }

    bool FC152Simulation::TryEvaluateEnvironment(
        const FAircraftState& State,
        const FVector3& WindVelocityNedMetersPerSecond,
        FAtmosphereState& OutAtmosphere,
        FAirData& OutAirData) const
    {
        if (!bEnvironmentConfigured)
        {
            return false;
        }

        // In local NED coordinates, positive Down displacement
        // decreases altitude above mean sea level.
        const double AltitudeMeters =
            EnvironmentConfiguration
            .OriginGeopotentialAltitudeMeters
            - State.PositionNedMeters.Z;

        FAtmosphereState AtmosphereSample{};

        if (!FStandardAtmosphere::TryEvaluate(
            AltitudeMeters,
            AtmosphereSample))
        {
            return false;
        }

        FAirData AirDataSample{};

        if (!FAirDataModel::TryEvaluate(
            State,
            AtmosphereSample,
            WindVelocityNedMetersPerSecond,
            AirDataSample))
        {
            return false;
        }

        OutAtmosphere = AtmosphereSample;
        OutAirData = AirDataSample;

        return true;
    }

    bool FC152Simulation::TryGetEnvironmentSample(
        FAtmosphereState& OutAtmosphere,
        FAirData& OutAirData) const
    {
        return TryEvaluateEnvironment(
            AircraftState,
            WindModel.GetWindVelocityNedMetersPerSecond(),
            OutAtmosphere,
            OutAirData);
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

        // The reference dynamics mass includes initial usable fuel.
        // Enough non-fuel mass must remain after complete fuel depletion.
        if (bAircraftModelConfigured
            && Configuration.MassProperties.MassKilograms
            <= AircraftModelConfiguration
            .InitialUsableFuelKilograms)
        {
            return false;
        }

        DynamicsConfiguration = Configuration;

        CurrentMassProperties =
            Configuration.MassProperties;

        bDynamicsConfigured = true;
        bLastDynamicsStepSuccessful = true;

        return true;
    }

    void FC152Simulation::ClearDynamicsConfiguration()
    {
        DynamicsConfiguration =
            FC152SimulationConfiguration{};

        CurrentMassProperties =
            FAircraftMassProperties{};

        AppliedBodyLoads =
            FBodyForcesAndMoments{};

        LastAircraftModelStepOutput =
            FC152AircraftModelStepOutput{};

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

    bool FC152Simulation::SetAircraftModelConfiguration(
        const FC152AircraftModelConfiguration& Configuration)
    {
        if (!std::isfinite(
            Configuration.InitialUsableFuelKilograms)
            || Configuration.InitialUsableFuelKilograms < 0.0
            || !std::isfinite(
                Configuration.GroundPlaneDownMeters))
        {
            return false;
        }

        // Dynamics mass represents the total aircraft mass
        // at the beginning of the simulation and therefore
        // includes the selected initial usable fuel.
        if (bDynamicsConfigured
            && DynamicsConfiguration
            .MassProperties.MassKilograms
            <= Configuration.InitialUsableFuelKilograms)
        {
            return false;
        }

        FAerodynamicModel CandidateAerodynamicModel;

        if (!CandidateAerodynamicModel.SetConfiguration(
            Configuration.Aerodynamics))
        {
            return false;
        }

        FPowerplantModel CandidatePowerplantModel;

        if (!CandidatePowerplantModel.SetConfiguration(
            Configuration.Powerplant))
        {
            return false;
        }

        if (!CandidatePowerplantModel
            .TrySetInitialUsableFuelKilograms(
                Configuration.InitialUsableFuelKilograms))
        {
            return false;
        }

        FGroundReactionModel CandidateGroundReactionModel;

        if (!CandidateGroundReactionModel.SetConfiguration(
            Configuration.GroundReaction))
        {
            return false;
        }

        AircraftModelConfiguration =
            Configuration;

        AerodynamicModel =
            CandidateAerodynamicModel;

        PowerplantModel =
            CandidatePowerplantModel;

        GroundReactionModel =
            CandidateGroundReactionModel;

        if (bDynamicsConfigured)
        {
            CurrentMassProperties =
                DynamicsConfiguration.MassProperties;
        }
        else
        {
            CurrentMassProperties =
                FAircraftMassProperties{};
        }

        LastAircraftModelStepOutput =
            FC152AircraftModelStepOutput{};

        bAircraftModelConfigured = true;
        bLastDynamicsStepSuccessful = true;

        return true;
    }

    void FC152Simulation::ClearAircraftModelConfiguration()
    {
        AircraftModelConfiguration =
            FC152AircraftModelConfiguration{};

        AerodynamicModel.ClearConfiguration();
        PowerplantModel.ClearConfiguration();
        GroundReactionModel.ClearConfiguration();

        LastAircraftModelStepOutput =
            FC152AircraftModelStepOutput{};

        if (bDynamicsConfigured)
        {
            CurrentMassProperties =
                DynamicsConfiguration.MassProperties;
        }
        else
        {
            CurrentMassProperties =
                FAircraftMassProperties{};
        }

        BrakeCommand = 0.0;

        bAircraftModelConfigured = false;
        bLastDynamicsStepSuccessful = true;
    }

    bool FC152Simulation::IsAircraftModelConfigured() const
    {
        return bAircraftModelConfigured;
    }

    bool FC152Simulation::SetBrakeCommand(
        const double NewBrakeCommand)
    {
        if (!std::isfinite(NewBrakeCommand)
            || NewBrakeCommand < 0.0
            || NewBrakeCommand > 1.0)
        {
            return false;
        }

        BrakeCommand = NewBrakeCommand;

        return true;
    }

    double FC152Simulation::GetBrakeCommand() const
    {
        return BrakeCommand;
    }

    bool FC152Simulation::ResetPowerplantState()
    {
        if (!bAircraftModelConfigured)
        {
            return true;
        }

        FPowerplantModel ResetPowerplantModel;

        if (!ResetPowerplantModel.SetConfiguration(
            AircraftModelConfiguration.Powerplant))
        {
            return false;
        }

        if (!ResetPowerplantModel
            .TrySetInitialUsableFuelKilograms(
                AircraftModelConfiguration
                .InitialUsableFuelKilograms))
        {
            return false;
        }

        PowerplantModel = ResetPowerplantModel;

        return true;
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

    const FAircraftMassProperties&
        FC152Simulation::GetCurrentMassProperties() const
    {
        return CurrentMassProperties;
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

    const FC152AircraftModelStepOutput&
        FC152Simulation::
        GetLastAircraftModelStepOutput() const
    {
        return LastAircraftModelStepOutput;
    }

    void FC152Simulation::AddBodyLoads(
        FBodyForcesAndMoments& InOutTotal,
        const FBodyForcesAndMoments& LoadsToAdd)
    {
        InOutTotal.ForceBodyNewtons +=
            LoadsToAdd.ForceBodyNewtons;

        InOutTotal.MomentBodyNewtonMeters +=
            LoadsToAdd.MomentBodyNewtonMeters;
    }

    bool FC152Simulation::Step(
        const FControlCommand& Command,
        const double FixedDeltaSeconds)
    {
        FWindModel CandidateWindModel =
            WindModel;

        if (bEnvironmentConfigured
            && !CandidateWindModel.Update(
                FixedDeltaSeconds))
        {
            return false;
        }

        FC152ControlSurfaceModel
            CandidateControlSurfaceModel =
            ControlSurfaceModel;

        CandidateControlSurfaceModel.Update(
            Command,
            FixedDeltaSeconds);

        if (!bDynamicsConfigured)
        {
            WindModel = CandidateWindModel;
            ControlSurfaceModel =
                CandidateControlSurfaceModel;

            LastAircraftModelStepOutput =
                FC152AircraftModelStepOutput{};

            return true;
        }

        FBodyForcesAndMoments TotalBodyLoads =
            AppliedBodyLoads;

        FAircraftMassProperties CandidateMassProperties =
            DynamicsConfiguration.MassProperties;

        FC152AircraftModelStepOutput
            CandidateStepOutput{};

        FPowerplantModel CandidatePowerplantModel =
            PowerplantModel;

        if (bAircraftModelConfigured)
        {
            if (!bEnvironmentConfigured)
            {
                return false;
            }

            if (!TryEvaluateEnvironment(
                AircraftState,
                CandidateWindModel
                .GetWindVelocityNedMetersPerSecond(),
                CandidateStepOutput.Atmosphere,
                CandidateStepOutput.AirData))
            {
                return false;
            }

            if (!AerodynamicModel.TryEvaluate(
                CandidateStepOutput.AirData,
                AircraftState
                .AngularRateBodyRadiansPerSecond,
                CandidateControlSurfaceModel.GetState(),
                CandidateStepOutput.AerodynamicLoads,
                CandidateStepOutput
                .AerodynamicCoefficients))
            {
                return false;
            }

            const FVector3 WindVelocityBodyMetersPerSecond =
                AircraftState.AttitudeBodyToNed
                .InverseRotateVector(
                    CandidateWindModel
                    .GetWindVelocityNedMetersPerSecond());

            const double
                AirRelativeAxialVelocityMetersPerSecond =
                AircraftState
                .VelocityBodyMetersPerSecond.X
                - WindVelocityBodyMetersPerSecond.X;

            // The current simplified fixed-pitch propulsion model
            // is evaluated with non-negative axial inflow.
            const double
                PropellerAxialAirspeedMetersPerSecond =
                std::max(
                    0.0,
                    AirRelativeAxialVelocityMetersPerSecond);

            if (!CandidatePowerplantModel.TryAdvance(
                CandidateControlSurfaceModel
                .GetState().Throttle,
                CandidateStepOutput.Atmosphere
                .DensityKilogramsPerCubicMeter,
                PropellerAxialAirspeedMetersPerSecond,
                FixedDeltaSeconds,
                CandidateStepOutput.PowerplantLoads,
                CandidateStepOutput.Powerplant))
            {
                return false;
            }

            const double ConsumedFuelKilograms =
                CandidateStepOutput.Powerplant.Fuel
                .TotalFuelConsumedKilograms;

            if (!std::isfinite(ConsumedFuelKilograms)
                || ConsumedFuelKilograms < 0.0)
            {
                return false;
            }

            // The reference mass includes the initial usable fuel.
            // Only consumed fuel is removed from the current mass.
            //
            // Fuel-distribution effects on CG and inertia are intentionally
            // deferred until tank geometry is modelled.
            CandidateMassProperties.MassKilograms =
                DynamicsConfiguration
                .MassProperties.MassKilograms
                - ConsumedFuelKilograms;

            if (!CandidateMassProperties.IsValid())
            {
                return false;
            }

            AddBodyLoads(
                TotalBodyLoads,
                CandidateStepOutput.AerodynamicLoads);

            AddBodyLoads(
                TotalBodyLoads,
                CandidateStepOutput.PowerplantLoads);

            if (!GroundReactionModel.TryEvaluate(
                AircraftState,
                AircraftModelConfiguration
                .GroundPlaneDownMeters,
                BrakeCommand,
                TotalBodyLoads,
                CandidateStepOutput
                .GroundReactionLoads,
                CandidateStepOutput.GroundReaction))
            {
                return false;
            }

            AddBodyLoads(
                TotalBodyLoads,
                CandidateStepOutput.GroundReactionLoads);

            CandidateStepOutput.TotalBodyLoads =
                TotalBodyLoads;
        }

        FAircraftState CandidateAircraftState =
            AircraftState;

        if (bAircraftModelConfigured)
        {
            CandidateStepOutput.MassPropertiesUsed =
                CandidateMassProperties;
        }

        if (!RigidBodyModel.Propagate(
            CandidateAircraftState,
            TotalBodyLoads,
            CandidateMassProperties,
            DynamicsConfiguration
            .GravityAccelerationNedMetersPerSecondSquared,
            FixedDeltaSeconds))
        {
            return false;
        }

        AircraftState = CandidateAircraftState;
        CurrentMassProperties =
            CandidateMassProperties;
        WindModel = CandidateWindModel;
        ControlSurfaceModel =
            CandidateControlSurfaceModel;

        if (bAircraftModelConfigured)
        {
            PowerplantModel =
                CandidatePowerplantModel;

            CandidateStepOutput.bValid = true;

            LastAircraftModelStepOutput =
                CandidateStepOutput;
        }
        else
        {
            LastAircraftModelStepOutput =
                FC152AircraftModelStepOutput{};
        }

        return true;
    }
}