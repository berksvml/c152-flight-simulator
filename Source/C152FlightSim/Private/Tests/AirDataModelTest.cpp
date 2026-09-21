#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/AirDataModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <limits>

namespace
{
    bool IsAirDataValueNear(
        const double Actual,
        const double Expected,
        const double Tolerance = 1.0e-6)
    {
        return std::abs(Actual - Expected) <= Tolerance;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FAirDataCalmAirTest,
    "C152FlightSim.FlightDynamics.AirData.CalmAir",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FAirDataCalmAirTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FAtmosphereState Atmosphere{};

    if (!FStandardAtmosphere::TryEvaluate(0.0, Atmosphere))
    {
        AddError(TEXT("Sea-level atmosphere evaluation failed"));
        return false;
    }

    FAircraftState Aircraft{};
    Aircraft.VelocityBodyMetersPerSecond =
        FVector3{ 3.0, 0.0, 4.0 };

    FAirData AirData{};

    const bool bSucceeded = FAirDataModel::TryEvaluate(
        Aircraft,
        Atmosphere,
        FVector3{},
        AirData);

    TestTrue(TEXT("Air data evaluation succeeds"), bSucceeded);
    TestTrue(
        TEXT("True airspeed is 5 m/s"),
        IsAirDataValueNear(AirData.TrueAirspeedMetersPerSecond, 5.0));
    TestTrue(
        TEXT("Positive body Z produces positive angle of attack"),
        IsAirDataValueNear(
            AirData.AngleOfAttackRadians,
            0.9272952180016122));
    TestTrue(
        TEXT("Sideslip is zero"),
        IsAirDataValueNear(AirData.SideslipAngleRadians, 0.0));
    TestTrue(
        TEXT("Dynamic pressure uses sea-level density"),
        IsAirDataValueNear(
            AirData.DynamicPressurePascals,
            15.3125,
            1.0e-3));
    TestTrue(
        TEXT("Mach is airspeed divided by speed of sound"),
        IsAirDataValueNear(
            AirData.MachNumber,
            5.0 / Atmosphere.SpeedOfSoundMetersPerSecond));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FAirDataWindAndAxesTest,
    "C152FlightSim.FlightDynamics.AirData.WindAndAxes",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FAirDataWindAndAxesTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    const FAtmosphereState Atmosphere{
        288.15, 101325.0, 1.225, 340.0
    };

    constexpr double Pi =
        3.14159265358979323846;

    const double HalfYaw =
        Pi / 4.0;

    FAircraftState Aircraft{};
    Aircraft.VelocityBodyMetersPerSecond =
        FVector3{ 10.0, 0.0, 0.0 };

    // Positive 90-degree NED yaw: aircraft nose points east.
    Aircraft.AttitudeBodyToNed = FQuaternion{
        std::cos(HalfYaw),
        0.0,
        0.0,
        std::sin(HalfYaw)
    };

    FAirData AirData{};

    const bool bSucceeded = FAirDataModel::TryEvaluate(
        Aircraft,
        Atmosphere,
        FVector3{ 0.0, 4.0, 0.0 },
        AirData);

    TestTrue(TEXT("Wind evaluation succeeds"), bSucceeded);
    TestTrue(
        TEXT("Eastward wind maps to body forward"),
        IsAirDataValueNear(
            AirData.RelativeVelocityBodyMetersPerSecond.X,
            6.0)
        && IsAirDataValueNear(
            AirData.RelativeVelocityBodyMetersPerSecond.Y,
            0.0)
        && IsAirDataValueNear(
            AirData.RelativeVelocityBodyMetersPerSecond.Z,
            0.0));
    TestTrue(
        TEXT("Airspeed differs from ground speed"),
        IsAirDataValueNear(AirData.TrueAirspeedMetersPerSecond, 6.0));
    TestTrue(
        TEXT("Dynamic pressure uses relative airspeed"),
        IsAirDataValueNear(AirData.DynamicPressurePascals, 22.05));

    Aircraft.AttitudeBodyToNed = FQuaternion{};

    const bool bCrosswindSucceeded = FAirDataModel::TryEvaluate(
        Aircraft,
        Atmosphere,
        FVector3{ 0.0, 4.0, 0.0 },
        AirData);

    TestTrue(TEXT("Crosswind evaluation succeeds"), bCrosswindSucceeded);
    TestTrue(
        TEXT("Eastward wind produces negative body Y relative velocity"),
        IsAirDataValueNear(
            AirData.RelativeVelocityBodyMetersPerSecond.Y,
            -4.0));
    TestTrue(
        TEXT("Negative body Y produces negative sideslip"),
        AirData.SideslipAngleRadians < 0.0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FAirDataZeroAndInvalidTest,
    "C152FlightSim.FlightDynamics.AirData.ZeroAndInvalid",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FAirDataZeroAndInvalidTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    const FAtmosphereState Atmosphere{
        288.15, 101325.0, 1.225, 340.0
    };

    FAircraftState Aircraft{};
    FAirData AirData{};

    const bool bStationarySucceeded =
        FAirDataModel::TryEvaluate(
            Aircraft,
            Atmosphere,
            FVector3{},
            AirData);

    TestTrue(
        TEXT("Stationary aircraft evaluation succeeds"),
        bStationarySucceeded);
    TestTrue(
        TEXT("Zero airspeed has zero angles and dynamic pressure"),
        AirData.TrueAirspeedMetersPerSecond == 0.0
        && AirData.AngleOfAttackRadians == 0.0
        && AirData.SideslipAngleRadians == 0.0
        && AirData.DynamicPressurePascals == 0.0);

    AirData.TrueAirspeedMetersPerSecond = 123.0;

    const FVector3 InvalidWind{
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.0
    };

    TestFalse(
        TEXT("Invalid wind is rejected"),
        FAirDataModel::TryEvaluate(
            Aircraft,
            Atmosphere,
            InvalidWind,
            AirData));
    TestTrue(
        TEXT("Rejected input preserves the previous output"),
        AirData.TrueAirspeedMetersPerSecond == 123.0);

    return true;
}

#endif