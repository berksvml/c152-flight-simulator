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

#endif