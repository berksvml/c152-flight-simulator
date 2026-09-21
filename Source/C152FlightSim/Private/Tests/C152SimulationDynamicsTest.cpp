#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152Simulation.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <cstdint>

namespace
{
    bool IsSimulationDynamicsValueNear(
        const double Actual,
        const double Expected,
        const double Tolerance = 1.0e-8)
    {
        return std::abs(Actual - Expected) <= Tolerance;
    }

    C152::FlightDynamics::FC152SimulationConfiguration
        MakeTestConfiguration()
    {
        using namespace C152::FlightDynamics;

        FC152SimulationConfiguration Configuration;

        Configuration.MassProperties.MassKilograms = 2.0;

        Configuration.MassProperties
            .InertiaXxKilogramMetersSquared = 2.0;

        Configuration.MassProperties
            .InertiaYyKilogramMetersSquared = 2.0;

        Configuration.MassProperties
            .InertiaZzKilogramMetersSquared = 2.0;

        Configuration.MassProperties
            .ProductOfInertiaXzKilogramMetersSquared = 0.0;

        Configuration
            .GravityAccelerationNedMetersPerSecondSquared = {};

        return Configuration;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationConfigurationTest,
    "C152FlightSim.FlightDynamics.Simulation."
    "DynamicsConfiguration",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationConfigurationTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    FC152Simulation Simulation;

    TestFalse(
        TEXT("Dynamics are disabled by default"),
        Simulation.IsDynamicsConfigured());

    FC152SimulationConfiguration InvalidConfiguration;

    const bool bInvalidConfigurationAccepted =
        Simulation.SetDynamicsConfiguration(
            InvalidConfiguration);

    TestFalse(
        TEXT("Invalid mass properties are rejected"),
        bInvalidConfigurationAccepted);

    TestFalse(
        TEXT("Rejected configuration does not enable dynamics"),
        Simulation.IsDynamicsConfigured());

    const bool bValidConfigurationAccepted =
        Simulation.SetDynamicsConfiguration(
            MakeTestConfiguration());

    TestTrue(
        TEXT("Valid dynamics configuration is accepted"),
        bValidConfigurationAccepted);

    TestTrue(
        TEXT("Accepted configuration enables dynamics"),
        Simulation.IsDynamicsConfigured());

    Simulation.ClearDynamicsConfiguration();

    TestFalse(
        TEXT("Dynamics configuration can be cleared"),
        Simulation.IsDynamicsConfigured());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationRigidBodyIntegrationTest,
    "C152FlightSim.FlightDynamics.Simulation."
    "RigidBodyIntegration",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationRigidBodyIntegrationTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    FC152Simulation Simulation;

    const bool bConfigurationAccepted =
        Simulation.SetDynamicsConfiguration(
            MakeTestConfiguration());

    TestTrue(
        TEXT("Test configuration is accepted"),
        bConfigurationAccepted);

    FBodyForcesAndMoments AppliedLoads;
    AppliedLoads.ForceBodyNewtons = {
        4.0,
        0.0,
        0.0
    };

    Simulation.SetAppliedBodyLoads(AppliedLoads);

    const FControlCommand Command{};
    const double FixedDeltaSeconds =
        Simulation.GetFixedDeltaSeconds();

    std::uint32_t TotalStepCount = 0U;

    for (std::uint32_t FrameIndex = 0U;
        FrameIndex < 120U;
        ++FrameIndex)
    {
        TotalStepCount += Simulation.Advance(
            Command,
            FixedDeltaSeconds);
    }

    const FAircraftState& State =
        Simulation.GetAircraftState();

    TestEqual(
        TEXT("Simulation executes 120 fixed steps"),
        TotalStepCount,
        static_cast<std::uint32_t>(120U));

    TestTrue(
        TEXT("All rigid-body steps succeed"),
        Simulation.WasLastDynamicsStepSuccessful());

    TestTrue(
        TEXT("Facade produces expected forward velocity"),
        IsSimulationDynamicsValueNear(
            State.VelocityBodyMetersPerSecond.X,
            2.0));

    TestTrue(
        TEXT("Facade produces expected forward position"),
        IsSimulationDynamicsValueNear(
            State.PositionNedMeters.X,
            1.0));

    TestTrue(
        TEXT("Unforced lateral states remain zero"),
        IsSimulationDynamicsValueNear(State.PositionNedMeters.Y, 0.0)
        && IsSimulationDynamicsValueNear(State.PositionNedMeters.Z, 0.0)
        && IsSimulationDynamicsValueNear(
            State.VelocityBodyMetersPerSecond.Y,
            0.0)
        && IsSimulationDynamicsValueNear(
            State.VelocityBodyMetersPerSecond.Z,
            0.0));

    return true;
}

#endif