#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152AircraftConfiguration.h"
#include "FlightDynamics/C152Simulation.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace C152SimulationAircraftModelTestSupport
{
    bool IsValueNear(
        const double Actual,
        const double Expected,
        const double Tolerance = 1.0e-9)
    {
        return std::abs(Actual - Expected)
            <= Tolerance;
    }

    bool IsSimulationAircraftModelVectorNear(
        const C152::FlightDynamics::FVector3& Actual,
        const C152::FlightDynamics::FVector3& Expected,
        const double Tolerance = 1.0e-9)
    {
        return IsValueNear(
            Actual.X,
            Expected.X,
            Tolerance)
            && IsValueNear(
                Actual.Y,
                Expected.Y,
                Tolerance)
            && IsValueNear(
                Actual.Z,
                Expected.Z,
                Tolerance);
    }

    bool ConfigureSimulation(
        C152::FlightDynamics::FC152Simulation& Simulation,
        const C152::FlightDynamics::FAircraftState& InitialState,
        const double GroundPlaneDownMeters)
    {
        using namespace C152::FlightDynamics;

        const FC152AircraftConfiguration AircraftConfiguration =
            FC152AircraftConfiguration::Create1979Model152();

        if (!AircraftConfiguration.IsValid())
        {
            return false;
        }

        FPowerplantModelConfiguration
            PowerplantConfiguration{};

        if (!AircraftConfiguration
            .TryGetDevelopmentPowerplantConfiguration(
                PowerplantConfiguration))
        {
            return false;
        }

        FC152SimulationConfiguration
            DynamicsConfiguration{};

        DynamicsConfiguration.MassProperties =
            AircraftConfiguration
            .DevelopmentMassPropertiesEstimate;

        if (!Simulation.SetDynamicsConfiguration(
            DynamicsConfiguration))
        {
            return false;
        }

        FEnvironmentConfiguration
            EnvironmentConfiguration{};

        EnvironmentConfiguration
            .OriginGeopotentialAltitudeMeters =
            100.0;

        if (!Simulation.SetEnvironmentConfiguration(
            EnvironmentConfiguration))
        {
            return false;
        }

        FC152AircraftModelConfiguration
            AircraftModelConfiguration{};

        AircraftModelConfiguration.Aerodynamics =
            AircraftConfiguration
            .DevelopmentAerodynamicEstimate;

        AircraftModelConfiguration.Powerplant =
            PowerplantConfiguration;

        AircraftModelConfiguration.GroundReaction =
            AircraftConfiguration
            .DevelopmentGroundReactionEstimate;

        AircraftModelConfiguration
            .InitialUsableFuelKilograms =
            10.0;

        AircraftModelConfiguration
            .GroundPlaneDownMeters =
            GroundPlaneDownMeters;

        if (!Simulation.SetAircraftModelConfiguration(
            AircraftModelConfiguration))
        {
            return false;
        }

        Simulation.Reset(InitialState);

        return Simulation.IsDynamicsConfigured()
            && Simulation.IsEnvironmentConfigured()
            && Simulation.IsAircraftModelConfigured()
            && Simulation
            .WasLastDynamicsStepSuccessful();
    }

    C152::FlightDynamics::FBodyForcesAndMoments
        SumAircraftModelLoads(
            const C152::FlightDynamics::
            FC152AircraftModelStepOutput& Output)
    {
        using namespace C152::FlightDynamics;

        FBodyForcesAndMoments Total{};

        Total.ForceBodyNewtons =
            Output.AerodynamicLoads.ForceBodyNewtons
            + Output.PowerplantLoads.ForceBodyNewtons
            + Output.GroundReactionLoads.ForceBodyNewtons;

        Total.MomentBodyNewtonMeters =
            Output.AerodynamicLoads.MomentBodyNewtonMeters
            + Output.PowerplantLoads.MomentBodyNewtonMeters
            + Output.GroundReactionLoads.MomentBodyNewtonMeters;

        return Total;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationAircraftModelConfigurationTest,
    "C152FlightSim.FlightDynamics."
    "Simulation.AircraftModels.Configuration",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationAircraftModelConfigurationTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    FC152Simulation Simulation;
    FAircraftState InitialState{};

    const bool bConfigurationSucceeded =
        C152SimulationAircraftModelTestSupport::
        ConfigureSimulation(
            Simulation,
            InitialState,
            1000.0);

    TestTrue(
        TEXT("Integrated aircraft model configuration succeeds"),
        bConfigurationSucceeded);

    if (!bConfigurationSucceeded)
    {
        return false;
    }

    TestTrue(
        TEXT("Zero brake command is the reset value"),
        C152SimulationAircraftModelTestSupport::
        IsValueNear(
            Simulation.GetBrakeCommand(),
            0.0));

    TestFalse(
        TEXT("Negative brake command is rejected"),
        Simulation.SetBrakeCommand(-0.1));

    TestFalse(
        TEXT("Brake command above one is rejected"),
        Simulation.SetBrakeCommand(1.1));

    TestTrue(
        TEXT("Valid brake command is accepted"),
        Simulation.SetBrakeCommand(0.5));

    TestTrue(
        TEXT("Accepted brake command is stored"),
        C152SimulationAircraftModelTestSupport::
        IsValueNear(
            Simulation.GetBrakeCommand(),
            0.5));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationAircraftModelAirborneLoadsTest,
    "C152FlightSim.FlightDynamics."
    "Simulation.AircraftModels.AirborneLoads",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationAircraftModelAirborneLoadsTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    using namespace
        C152SimulationAircraftModelTestSupport;

    (void)Parameters;

    FC152Simulation Simulation;

    FAircraftState InitialState{};

    InitialState.VelocityBodyMetersPerSecond =
        FVector3{
            30.0,
            0.0,
            0.0
    };

    const bool bConfigurationSucceeded =
        ConfigureSimulation(
            Simulation,
            InitialState,
            1000.0);

    TestTrue(
        TEXT("Airborne simulation configuration succeeds"),
        bConfigurationSucceeded);

    if (!bConfigurationSucceeded)
    {
        return false;
    }

    FControlCommand Command{};
    Command.Throttle = 0.75;

    const std::uint32_t StepCount =
        Simulation.Advance(
            Command,
            Simulation.GetFixedDeltaSeconds());

    TestEqual(
        TEXT("Exactly one fixed step is executed"),
        StepCount,
        static_cast<std::uint32_t>(1U));

    TestTrue(
        TEXT("Integrated dynamics step succeeds"),
        Simulation.WasLastDynamicsStepSuccessful());

    const FC152AircraftModelStepOutput& Output =
        Simulation.GetLastAircraftModelStepOutput();

    TestTrue(
        TEXT("Integrated model output is valid"),
        Output.bValid);

    TestFalse(
        TEXT("Airborne aircraft has no ground contact"),
        Output.GroundReaction.bOnGround);

    TestEqual(
        TEXT("Airborne aircraft has zero active contacts"),
        Output.GroundReaction.ActiveContactCount,
        0);

    TestTrue(
        TEXT("Aerodynamic drag acts rearward"),
        Output.AerodynamicLoads
        .ForceBodyNewtons.X < 0.0);

    TestTrue(
        TEXT("Aerodynamic lift acts upward in Body FRD"),
        Output.AerodynamicLoads
        .ForceBodyNewtons.Z < 0.0);

    TestTrue(
        TEXT("Powerplant produces forward thrust"),
        Output.PowerplantLoads
        .ForceBodyNewtons.X > 0.0);

    TestTrue(
        TEXT("Powerplant reports power production"),
        Output.Powerplant.bProducingPower);

    const FBodyForcesAndMoments ExpectedTotalLoads =
        SumAircraftModelLoads(Output);

    TestTrue(
        TEXT("Total force equals the sum of model forces"),
        IsSimulationAircraftModelVectorNear(
            Output.TotalBodyLoads.ForceBodyNewtons,
            ExpectedTotalLoads.ForceBodyNewtons));

    TestTrue(
        TEXT("Total moment equals the sum of model moments"),
        IsSimulationAircraftModelVectorNear(
            Output.TotalBodyLoads.MomentBodyNewtonMeters,
            ExpectedTotalLoads.MomentBodyNewtonMeters));

    TestTrue(
        TEXT("Propagated aircraft state remains finite"),
        Simulation.GetAircraftState()
        .PositionNedMeters.IsFinite()
        && Simulation.GetAircraftState()
        .VelocityBodyMetersPerSecond.IsFinite()
        && Simulation.GetAircraftState()
        .AngularRateBodyRadiansPerSecond.IsFinite());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationAircraftModelGroundContactTest,
    "C152FlightSim.FlightDynamics."
    "Simulation.AircraftModels.GroundContact",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationAircraftModelGroundContactTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    using namespace
        C152SimulationAircraftModelTestSupport;

    (void)Parameters;

    FC152Simulation Simulation;

    FAircraftState InitialState{};

    // Contact points are one meter below the aircraft origin.
    // This position creates 0.02 m of initial penetration
    // into a runway at NED Down = 0.
    InitialState.PositionNedMeters.Z =
        -0.98;

    const bool bConfigurationSucceeded =
        ConfigureSimulation(
            Simulation,
            InitialState,
            0.0);

    TestTrue(
        TEXT("Ground simulation configuration succeeds"),
        bConfigurationSucceeded);

    if (!bConfigurationSucceeded)
    {
        return false;
    }

    const std::uint32_t StepCount =
        Simulation.Advance(
            FControlCommand{},
            Simulation.GetFixedDeltaSeconds());

    TestEqual(
        TEXT("Exactly one fixed step is executed"),
        StepCount,
        static_cast<std::uint32_t>(1U));

    TestTrue(
        TEXT("Ground dynamics step succeeds"),
        Simulation.WasLastDynamicsStepSuccessful());

    const FC152AircraftModelStepOutput& Output =
        Simulation.GetLastAircraftModelStepOutput();

    TestTrue(
        TEXT("Ground model output is valid"),
        Output.bValid);

    TestTrue(
        TEXT("Aircraft is reported on the ground"),
        Output.GroundReaction.bOnGround);

    TestEqual(
        TEXT("All three landing-gear contacts are active"),
        Output.GroundReaction.ActiveContactCount,
        3);

    TestTrue(
        TEXT("Ground reaction produces positive normal force"),
        Output.GroundReaction.TotalNormalForceNewtons > 0.0);

    TestTrue(
        TEXT("Ground reaction acts upward in Body FRD"),
        Output.GroundReactionLoads
        .ForceBodyNewtons.Z < 0.0);

    const FBodyForcesAndMoments ExpectedTotalLoads =
        SumAircraftModelLoads(Output);

    TestTrue(
        TEXT("Ground-case total force equals summed model forces"),
        IsSimulationAircraftModelVectorNear(
            Output.TotalBodyLoads.ForceBodyNewtons,
            ExpectedTotalLoads.ForceBodyNewtons));

    TestTrue(
        TEXT("Ground-case total moment equals summed model moments"),
        IsSimulationAircraftModelVectorNear(
            Output.TotalBodyLoads.MomentBodyNewtonMeters,
            ExpectedTotalLoads.MomentBodyNewtonMeters));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationAircraftModelFuelMassCouplingTest,
    "C152FlightSim.FlightDynamics."
    "Simulation.AircraftModels.FuelMassCoupling",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationAircraftModelFuelMassCouplingTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    using namespace
        C152SimulationAircraftModelTestSupport;

    (void)Parameters;

    FC152Simulation Simulation;

    FAircraftState InitialState{};

    InitialState.VelocityBodyMetersPerSecond =
        FVector3{
            30.0,
            0.0,
            0.0
    };

    const bool bConfigurationSucceeded =
        ConfigureSimulation(
            Simulation,
            InitialState,
            1000.0);

    TestTrue(
        TEXT("Fuel-mass simulation configuration succeeds"),
        bConfigurationSucceeded);

    if (!bConfigurationSucceeded)
    {
        return false;
    }

    const double InitialMassKilograms =
        Simulation.GetCurrentMassProperties()
        .MassKilograms;

    FControlCommand Command{};
    Command.Throttle = 1.0;

    const std::uint32_t FirstStepCount =
        Simulation.Advance(
            Command,
            Simulation.GetFixedDeltaSeconds());

    TestEqual(
        TEXT("First fuel-mass update executes one fixed step"),
        FirstStepCount,
        static_cast<std::uint32_t>(1U));

    TestTrue(
        TEXT("First fuel-mass dynamics step succeeds"),
        Simulation.WasLastDynamicsStepSuccessful());

    const FC152AircraftModelStepOutput& FirstOutput =
        Simulation.GetLastAircraftModelStepOutput();

    const double FirstConsumedFuelKilograms =
        FirstOutput.Powerplant.Fuel
        .TotalFuelConsumedKilograms;

    TestTrue(
        TEXT("Powerplant consumes fuel"),
        FirstConsumedFuelKilograms > 0.0);

    const double ExpectedFirstMassKilograms =
        InitialMassKilograms
        - FirstConsumedFuelKilograms;

    TestTrue(
        TEXT("Current aircraft mass decreases by consumed fuel"),
        IsValueNear(
            Simulation.GetCurrentMassProperties()
            .MassKilograms,
            ExpectedFirstMassKilograms,
            1.0e-12));

    TestTrue(
        TEXT("Step output records the mass used by 6DOF"),
        IsValueNear(
            FirstOutput.MassPropertiesUsed.MassKilograms,
            ExpectedFirstMassKilograms,
            1.0e-12));

    const double MassAfterFirstStepKilograms =
        Simulation.GetCurrentMassProperties()
        .MassKilograms;

    const std::uint32_t SecondStepCount =
        Simulation.Advance(
            Command,
            Simulation.GetFixedDeltaSeconds());

    TestEqual(
        TEXT("Second fuel-mass update executes one fixed step"),
        SecondStepCount,
        static_cast<std::uint32_t>(1U));

    TestTrue(
        TEXT("Mass continues decreasing while producing power"),
        Simulation.GetCurrentMassProperties()
        .MassKilograms
        < MassAfterFirstStepKilograms);

    Simulation.Reset(InitialState);

    TestTrue(
        TEXT("Reset restores initial reference mass"),
        IsValueNear(
            Simulation.GetCurrentMassProperties()
            .MassKilograms,
            InitialMassKilograms,
            1.0e-12));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationAircraftModelAileronRollResponseTest,
    "C152FlightSim.FlightDynamics."
    "Simulation.AircraftModels.AileronRollResponse",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationAircraftModelAileronRollResponseTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    using namespace
        C152SimulationAircraftModelTestSupport;

    (void)Parameters;

    FC152Simulation Simulation;

    FAircraftState InitialState{};

    InitialState.VelocityBodyMetersPerSecond =
        FVector3{
            30.0,
            0.0,
            0.0
    };

    const bool bConfigurationSucceeded =
        ConfigureSimulation(
            Simulation,
            InitialState,
            1000.0);

    TestTrue(
        TEXT("Aileron-response simulation configuration succeeds"),
        bConfigurationSucceeded);

    if (!bConfigurationSucceeded)
    {
        return false;
    }

    FControlCommand Command{};

    // Positive roll command represents a right-roll command.
    Command.Roll = 1.0;

    const std::uint32_t StepCount =
        Simulation.Advance(
            Command,
            Simulation.GetFixedDeltaSeconds());

    TestEqual(
        TEXT("Aileron response executes one fixed step"),
        StepCount,
        static_cast<std::uint32_t>(1U));

    TestTrue(
        TEXT("Aileron-response dynamics step succeeds"),
        Simulation.WasLastDynamicsStepSuccessful());

    const FControlSurfaceState& SurfaceState =
        Simulation.GetControlSurfaceState();

    TestTrue(
        TEXT("Positive roll command produces positive aileron"),
        SurfaceState.AileronRad > 0.0);

    const FC152AircraftModelStepOutput& Output =
        Simulation.GetLastAircraftModelStepOutput();

    TestTrue(
        TEXT("Aileron-response model output is valid"),
        Output.bValid);

    TestFalse(
        TEXT("Aileron-response aircraft remains airborne"),
        Output.GroundReaction.bOnGround);

    TestTrue(
        TEXT("Positive aileron produces positive rolling moment"),
        Output.AerodynamicLoads
        .MomentBodyNewtonMeters.X > 0.0);

    TestTrue(
        TEXT("Positive rolling moment produces positive roll rate"),
        Simulation.GetAircraftState()
        .AngularRateBodyRadiansPerSecond.X > 0.0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationAircraftModelRudderYawResponseTest,
    "C152FlightSim.FlightDynamics."
    "Simulation.AircraftModels.RudderYawResponse",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationAircraftModelRudderYawResponseTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    using namespace
        C152SimulationAircraftModelTestSupport;

    (void)Parameters;

    FC152Simulation Simulation;

    FAircraftState InitialState{};

    InitialState.VelocityBodyMetersPerSecond =
        FVector3{
            30.0,
            0.0,
            0.0
    };

    const bool bConfigurationSucceeded =
        ConfigureSimulation(
            Simulation,
            InitialState,
            1000.0);

    TestTrue(
        TEXT("Rudder-response simulation configuration succeeds"),
        bConfigurationSucceeded);

    if (!bConfigurationSucceeded)
    {
        return false;
    }

    FControlCommand Command{};

    // Positive yaw command represents a nose-right command.
    Command.Yaw = 1.0;

    const std::uint32_t StepCount =
        Simulation.Advance(
            Command,
            Simulation.GetFixedDeltaSeconds());

    TestEqual(
        TEXT("Rudder response executes one fixed step"),
        StepCount,
        static_cast<std::uint32_t>(1U));

    TestTrue(
        TEXT("Rudder-response dynamics step succeeds"),
        Simulation.WasLastDynamicsStepSuccessful());

    const FControlSurfaceState& SurfaceState =
        Simulation.GetControlSurfaceState();

    TestTrue(
        TEXT("Positive yaw command produces positive rudder"),
        SurfaceState.RudderRad > 0.0);

    const FC152AircraftModelStepOutput& Output =
        Simulation.GetLastAircraftModelStepOutput();

    TestTrue(
        TEXT("Rudder-response model output is valid"),
        Output.bValid);

    TestFalse(
        TEXT("Rudder-response aircraft remains airborne"),
        Output.GroundReaction.bOnGround);

    TestTrue(
        TEXT("Positive rudder produces negative body side force"),
        Output.AerodynamicLoads
        .ForceBodyNewtons.Y < 0.0);

    TestTrue(
        TEXT("Positive rudder produces positive yawing moment"),
        Output.AerodynamicLoads
        .MomentBodyNewtonMeters.Z > 0.0);

    TestTrue(
        TEXT("Positive yawing moment produces positive yaw rate"),
        Simulation.GetAircraftState()
        .AngularRateBodyRadiansPerSecond.Z > 0.0);

    return true;
}

#endif