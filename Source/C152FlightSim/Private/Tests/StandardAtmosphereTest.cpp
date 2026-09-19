#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/StandardAtmosphere.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <limits>

namespace
{
    bool IsNear(
        const double Actual,
        const double Expected,
        const double Tolerance)
    {
        return std::abs(Actual - Expected) <= Tolerance;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStandardAtmosphereSeaLevelTest,
    "C152FlightSim.FlightDynamics.Atmosphere.SeaLevel",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

bool FStandardAtmosphereSeaLevelTest::RunTest(
    const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FAtmosphereState Atmosphere{};
    const bool bSucceeded =
        FStandardAtmosphere::TryEvaluate(0.0, Atmosphere);

    TestTrue(TEXT("Sea level is in range"), bSucceeded);
    TestTrue(
        TEXT("Sea-level temperature is 288.15 K"),
        IsNear(Atmosphere.TemperatureKelvin, 288.15, 1.0e-9));
    TestTrue(
        TEXT("Sea-level pressure is 101325 Pa"),
        IsNear(Atmosphere.PressurePascals, 101325.0, 1.0e-6));
    TestTrue(
        TEXT("Sea-level density is approximately 1.225 kg/m^3"),
        IsNear(Atmosphere.DensityKilogramsPerCubicMeter, 1.225, 1.0e-4));
    TestTrue(
        TEXT("Sea-level speed of sound is approximately 340.3 m/s"),
        IsNear(Atmosphere.SpeedOfSoundMetersPerSecond, 340.3, 0.1));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStandardAtmosphereTroposphereTest,
    "C152FlightSim.FlightDynamics.Atmosphere.Troposphere",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

bool FStandardAtmosphereTroposphereTest::RunTest(
    const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FAtmosphereState AtFiveKilometers{};
    FAtmosphereState AtElevenKilometers{};

    const bool bFiveKilometersSucceeded =
        FStandardAtmosphere::TryEvaluate(5000.0, AtFiveKilometers);
    const bool bElevenKilometersSucceeded =
        FStandardAtmosphere::TryEvaluate(
            FStandardAtmosphere::MaxGeopotentialAltitudeMeters,
            AtElevenKilometers);

    TestTrue(TEXT("5 km is in range"), bFiveKilometersSucceeded);
    TestTrue(TEXT("11 km boundary is in range"), bElevenKilometersSucceeded);
    TestTrue(
        TEXT("Temperature at 5 km is 255.65 K"),
        IsNear(AtFiveKilometers.TemperatureKelvin, 255.65, 1.0e-9));
    TestTrue(
        TEXT("Pressure at 5 km agrees with the standard atmosphere"),
        IsNear(AtFiveKilometers.PressurePascals, 54019.9, 1.0));
    TestTrue(
        TEXT("Pressure at 11 km agrees with the standard atmosphere"),
        IsNear(AtElevenKilometers.PressurePascals, 22632.1, 1.0));
    TestTrue(
        TEXT("Density at 11 km is approximately 0.364 kg/m^3"),
        IsNear(
            AtElevenKilometers.DensityKilogramsPerCubicMeter,
            0.363918,
            1.0e-4));
    TestTrue(
        TEXT("Speed of sound at 11 km is approximately 295.1 m/s"),
        IsNear(
            AtElevenKilometers.SpeedOfSoundMetersPerSecond,
            295.1,
            0.1));
    TestTrue(
        TEXT("Density decreases with altitude"),
        AtFiveKilometers.DensityKilogramsPerCubicMeter
            > AtElevenKilometers.DensityKilogramsPerCubicMeter);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStandardAtmosphereInvalidAltitudeTest,
    "C152FlightSim.FlightDynamics.Atmosphere.InvalidAltitude",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

bool FStandardAtmosphereInvalidAltitudeTest::RunTest(
    const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FAtmosphereState Atmosphere{ 1.0, 2.0, 3.0, 4.0 };

    TestFalse(
        TEXT("Negative altitude is rejected"),
        FStandardAtmosphere::TryEvaluate(-1.0, Atmosphere));
    TestFalse(
        TEXT("Altitude above 11 km is rejected"),
        FStandardAtmosphere::TryEvaluate(11000.1, Atmosphere));
    TestFalse(
        TEXT("NaN altitude is rejected"),
        FStandardAtmosphere::TryEvaluate(
            std::numeric_limits<double>::quiet_NaN(),
            Atmosphere));
    TestFalse(
        TEXT("Infinite altitude is rejected"),
        FStandardAtmosphere::TryEvaluate(
            std::numeric_limits<double>::infinity(),
            Atmosphere));
    TestTrue(
        TEXT("Rejected input leaves output unchanged"),
        Atmosphere.TemperatureKelvin == 1.0
            && Atmosphere.PressurePascals == 2.0
            && Atmosphere.DensityKilogramsPerCubicMeter == 3.0
            && Atmosphere.SpeedOfSoundMetersPerSecond == 4.0);

    return true;
}

#endif
