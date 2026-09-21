#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152Simulation.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <cstdint>
#include <limits>

namespace
{
    bool IsNear(
        const double Actual,
        const double Expected,
        const double Tolerance = 1.0e-6)
    {
        return std::abs(Actual - Expected) <= Tolerance;
    }

    bool IsVectorNear(
        const C152::FlightDynamics::FVector3& Actual,
        const C152::FlightDynamics::FVector3& Expected,
        const double Tolerance = 1.0e-12)
    {
        return IsNear(Actual.X, Expected.X, Tolerance)
            && IsNear(Actual.Y, Expected.Y, Tolerance)
            && IsNear(Actual.Z, Expected.Z, Tolerance);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationEnvironmentSampleTest,
    "C152FlightSim.FlightDynamics.Simulation.EnvironmentSample",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationEnvironmentSampleTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    constexpr double TestPi = 3.14159265358979323846;

    FC152Simulation Simulation;

    FAircraftState InitialState{};
    InitialState.PositionNedMeters.Z = -100.0;
    InitialState.VelocityBodyMetersPerSecond.X = 10.0;
    InitialState.AttitudeBodyToNed = FQuaternion{
        std::cos(TestPi / 4.0),
        0.0,
        0.0,
        std::sin(TestPi / 4.0)
    };

    Simulation.Reset(InitialState);

    FEnvironmentConfiguration Environment{};
    Environment.OriginGeopotentialAltitudeMeters = 1200.0;
    Environment.WindVelocityNedMetersPerSecond =
        FVector3{ 0.0, 4.0, 0.0 };

    TestTrue(
        TEXT("Environment configuration is accepted"),
        Simulation.SetEnvironmentConfiguration(Environment));

    FAtmosphereState Atmosphere{};
    FAirData AirData{};

    const bool bInitialSampleSucceeded =
        Simulation.TryGetEnvironmentSample(
            Atmosphere,
            AirData);

    TestTrue(
        TEXT("Initial environment sample succeeds"),
        bInitialSampleSucceeded);
    TestTrue(
        TEXT("NED position gives an altitude of 1300 m"),
        IsNear(Atmosphere.TemperatureKelvin, 279.7));
    TestTrue(
        TEXT("Eastward wind reduces eastward airspeed to 6 m/s"),
        IsNear(AirData.TrueAirspeedMetersPerSecond, 6.0));

    FC152SimulationConfiguration Dynamics{};
    Dynamics.MassProperties.MassKilograms = 2.0;
    Dynamics.MassProperties.InertiaXxKilogramMetersSquared = 2.0;
    Dynamics.MassProperties.InertiaYyKilogramMetersSquared = 2.0;
    Dynamics.MassProperties.InertiaZzKilogramMetersSquared = 2.0;
    Dynamics.GravityAccelerationNedMetersPerSecondSquared = {};

    if (!Simulation.SetDynamicsConfiguration(Dynamics))
    {
        AddError(TEXT("Test dynamics configuration was rejected"));
        return false;
    }

    FBodyForcesAndMoments Loads{};
    Loads.ForceBodyNewtons.X = 4.0;
    Simulation.SetAppliedBodyLoads(Loads);

    std::uint32_t TotalSteps = 0U;

    for (std::uint32_t Index = 0U; Index < 120U; ++Index)
    {
        TotalSteps += Simulation.Advance(
            FControlCommand{},
            Simulation.GetFixedDeltaSeconds());
    }

    TestEqual(
        TEXT("One second executes 120 fixed steps"),
        TotalSteps,
        static_cast<std::uint32_t>(120U));

    const bool bUpdatedSampleSucceeded =
        Simulation.TryGetEnvironmentSample(
            Atmosphere,
            AirData);

    TestTrue(
        TEXT("Updated environment sample succeeds"),
        bUpdatedSampleSucceeded);
    TestTrue(
        TEXT("Acceleration raises body forward speed to 12 m/s"),
        IsNear(
            Simulation.GetAircraftState()
            .VelocityBodyMetersPerSecond.X,
            12.0));
    TestTrue(
        TEXT("Updated airspeed accounts for the same wind"),
        IsNear(AirData.TrueAirspeedMetersPerSecond, 8.0));
    TestTrue(
        TEXT("Dynamic pressure follows updated airspeed"),
        IsNear(
            AirData.DynamicPressurePascals,
            0.5 * Atmosphere.DensityKilogramsPerCubicMeter * 64.0));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationEnvironmentValidityTest,
    "C152FlightSim.FlightDynamics.Simulation.EnvironmentValidity",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationEnvironmentValidityTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FC152Simulation Simulation;

    FAtmosphereState Atmosphere{ 1.0, 2.0, 3.0, 4.0 };
    FAirData AirData{};
    AirData.TrueAirspeedMetersPerSecond = 123.0;

    TestFalse(
        TEXT("Sample requires an environment configuration"),
        Simulation.TryGetEnvironmentSample(
            Atmosphere,
            AirData));

    FEnvironmentConfiguration Invalid{};
    Invalid.WindVelocityNedMetersPerSecond.X =
        std::numeric_limits<double>::quiet_NaN();

    TestFalse(
        TEXT("Nonfinite wind is rejected"),
        Simulation.SetEnvironmentConfiguration(Invalid));
    TestFalse(
        TEXT("Rejected configuration is not activated"),
        Simulation.IsEnvironmentConfigured());

    FEnvironmentConfiguration Valid{};
    TestTrue(
        TEXT("Sea-level origin is accepted"),
        Simulation.SetEnvironmentConfiguration(Valid));

    FAircraftState BelowOrigin{};
    BelowOrigin.PositionNedMeters.Z = 1.0;
    Simulation.Reset(BelowOrigin);

    TestFalse(
        TEXT("Altitude below the atmosphere model range is rejected"),
        Simulation.TryGetEnvironmentSample(
            Atmosphere,
            AirData));
    TestTrue(
        TEXT("Failed sample leaves both outputs unchanged"),
        Atmosphere.TemperatureKelvin == 1.0
        && AirData.TrueAirspeedMetersPerSecond == 123.0);

    Simulation.ClearEnvironmentConfiguration();

    TestFalse(
        TEXT("Environment can be cleared"),
        Simulation.IsEnvironmentConfigured());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FC152SimulationEnvironmentTurbulenceTest,
    "C152FlightSim.FlightDynamics.Simulation.EnvironmentTurbulence",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FC152SimulationEnvironmentTurbulenceTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FC152Simulation FirstSimulation;
    FC152Simulation SecondSimulation;

    FAircraftState InitialState{};
    InitialState.VelocityBodyMetersPerSecond =
        FVector3{ 10.0, 0.0, 0.0 };

    FirstSimulation.Reset(InitialState);
    SecondSimulation.Reset(InitialState);

    FEnvironmentConfiguration Environment{};
    Environment.OriginGeopotentialAltitudeMeters = 100.0;
    Environment.WindVelocityNedMetersPerSecond =
        FVector3{ 1.0, 0.0, 0.0 };

    Environment
        .TurbulenceStandardDeviationNedMetersPerSecond =
        FVector3{ 0.5, 0.25, 0.1 };

    Environment.TurbulenceCorrelationTimeSeconds = 1.5;
    Environment.TurbulenceRandomSeed = 424242U;

    if (!FirstSimulation.SetEnvironmentConfiguration(Environment)
        || !SecondSimulation.SetEnvironmentConfiguration(Environment))
    {
        AddError(TEXT("Test environment configuration was rejected"));
        return false;
    }

    bool bSequencesMatch = true;
    bool bTurbulenceGenerated = false;

    std::uint32_t FirstStepCount = 0U;
    std::uint32_t SecondStepCount = 0U;

    for (std::uint32_t StepIndex = 0U;
        StepIndex < 120U;
        ++StepIndex)
    {
        FirstStepCount += FirstSimulation.Advance(
            FControlCommand{},
            FirstSimulation.GetFixedDeltaSeconds());

        SecondStepCount += SecondSimulation.Advance(
            FControlCommand{},
            SecondSimulation.GetFixedDeltaSeconds());

        bSequencesMatch =
            bSequencesMatch
            && IsVectorNear(
                FirstSimulation
                .GetWindVelocityNedMetersPerSecond(),
                SecondSimulation
                .GetWindVelocityNedMetersPerSecond());

        bTurbulenceGenerated =
            bTurbulenceGenerated
            || FirstSimulation
            .GetTurbulenceVelocityNedMetersPerSecond()
            .NormSquared() > 0.0;
    }

    TestEqual(
        TEXT("First simulation executes 120 fixed steps"),
        FirstStepCount,
        static_cast<std::uint32_t>(120U));

    TestEqual(
        TEXT("Second simulation executes 120 fixed steps"),
        SecondStepCount,
        static_cast<std::uint32_t>(120U));

    TestTrue(
        TEXT("Equal seeds produce equal simulation wind sequences"),
        bSequencesMatch);

    TestTrue(
        TEXT("Simulation environment produces turbulence"),
        bTurbulenceGenerated);

    TestTrue(
        TEXT("Environment updates remain successful"),
        FirstSimulation.WasLastDynamicsStepSuccessful()
        && SecondSimulation.WasLastDynamicsStepSuccessful());

    FAtmosphereState Atmosphere{};
    FAirData AirData{};

    const bool bSampleSucceeded =
        FirstSimulation.TryGetEnvironmentSample(
            Atmosphere,
            AirData);

    TestTrue(
        TEXT("Air-data sample succeeds after turbulence updates"),
        bSampleSucceeded);

    const FVector3 ExpectedRelativeVelocity =
        InitialState.VelocityBodyMetersPerSecond
        - FirstSimulation
        .GetWindVelocityNedMetersPerSecond();

    TestTrue(
        TEXT("Air data uses the current turbulent wind"),
        IsVectorNear(
            AirData.RelativeVelocityBodyMetersPerSecond,
            ExpectedRelativeVelocity));

    return true;
}

#endif