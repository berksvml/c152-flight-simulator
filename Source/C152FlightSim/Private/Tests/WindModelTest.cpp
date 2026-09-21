#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/WindModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <limits>

namespace
{
    bool IsWindValueNear(
        const double Actual,
        const double Expected,
        const double Tolerance = 1.0e-12)
    {
        return std::abs(Actual - Expected) <= Tolerance;
    }

    bool IsVectorNear(
        const C152::FlightDynamics::FVector3& Actual,
        const C152::FlightDynamics::FVector3& Expected,
        const double Tolerance = 1.0e-12)
    {
        return IsWindValueNear(Actual.X, Expected.X, Tolerance)
            && IsWindValueNear(Actual.Y, Expected.Y, Tolerance)
            && IsWindValueNear(Actual.Z, Expected.Z, Tolerance);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FWindModelSteadyWindTest,
    "C152FlightSim.FlightDynamics.Wind.Steady",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FWindModelSteadyWindTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FWindModelConfiguration Configuration{};
    Configuration.SteadyWindVelocityNedMetersPerSecond =
        FVector3{ 3.0, -2.0, 1.0 };

    FWindModel WindModel;

    TestTrue(
        TEXT("Valid wind configuration is accepted"),
        WindModel.Configure(Configuration));

    TestTrue(
        TEXT("Initial output equals the steady wind"),
        IsVectorNear(
            WindModel.GetWindVelocityNedMetersPerSecond(),
            Configuration.SteadyWindVelocityNedMetersPerSecond));

    TestTrue(
        TEXT("Wind model update succeeds"),
        WindModel.Update(1.0 / 120.0));

    TestTrue(
        TEXT("Zero turbulence preserves steady wind"),
        IsVectorNear(
            WindModel.GetWindVelocityNedMetersPerSecond(),
            Configuration.SteadyWindVelocityNedMetersPerSecond));

    TestTrue(
        TEXT("Zero turbulence produces no fluctuation"),
        IsVectorNear(
            WindModel.GetTurbulenceVelocityNedMetersPerSecond(),
            FVector3{}));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FWindModelDeterminismTest,
    "C152FlightSim.FlightDynamics.Wind.Determinism",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FWindModelDeterminismTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FWindModelConfiguration Configuration{};
    Configuration.SteadyWindVelocityNedMetersPerSecond =
        FVector3{ 4.0, 1.0, 0.0 };

    Configuration
        .TurbulenceStandardDeviationNedMetersPerSecond =
        FVector3{ 1.0, 0.5, 0.25 };

    Configuration.TurbulenceCorrelationTimeSeconds = 2.0;
    Configuration.RandomSeed = 12345U;

    FWindModel FirstModel;
    FWindModel SecondModel;

    if (!FirstModel.Configure(Configuration)
        || !SecondModel.Configure(Configuration))
    {
        AddError(TEXT("Test wind configuration was rejected"));
        return false;
    }

    bool bSequencesMatch = true;
    bool bTurbulenceGenerated = false;

    FVector3 FirstSample{};

    for (int32 StepIndex = 0; StepIndex < 240; ++StepIndex)
    {
        if (!FirstModel.Update(1.0 / 120.0)
            || !SecondModel.Update(1.0 / 120.0))
        {
            AddError(TEXT("Wind model update failed"));
            return false;
        }

        if (StepIndex == 0)
        {
            FirstSample =
                FirstModel.GetWindVelocityNedMetersPerSecond();
        }

        bSequencesMatch =
            bSequencesMatch
            && IsVectorNear(
                FirstModel.GetWindVelocityNedMetersPerSecond(),
                SecondModel.GetWindVelocityNedMetersPerSecond());

        bTurbulenceGenerated =
            bTurbulenceGenerated
            || FirstModel
            .GetTurbulenceVelocityNedMetersPerSecond()
            .NormSquared() > 0.0;
    }

    TestTrue(
        TEXT("Equal seeds produce equal wind sequences"),
        bSequencesMatch);

    TestTrue(
        TEXT("Non-zero turbulence configuration produces fluctuations"),
        bTurbulenceGenerated);

    FirstModel.Reset();

    TestTrue(
        TEXT("Reset returns output to steady wind"),
        IsVectorNear(
            FirstModel.GetWindVelocityNedMetersPerSecond(),
            Configuration.SteadyWindVelocityNedMetersPerSecond));

    TestTrue(
        TEXT("First update after reset succeeds"),
        FirstModel.Update(1.0 / 120.0));

    TestTrue(
        TEXT("Reset reproduces the original first sample"),
        IsVectorNear(
            FirstModel.GetWindVelocityNedMetersPerSecond(),
            FirstSample));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FWindModelValidationTest,
    "C152FlightSim.FlightDynamics.Wind.Validation",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FWindModelValidationTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;
    (void)Parameters;

    FWindModel WindModel;

    FWindModelConfiguration NegativeIntensity{};
    NegativeIntensity
        .TurbulenceStandardDeviationNedMetersPerSecond.X = -1.0;

    TestFalse(
        TEXT("Negative turbulence intensity is rejected"),
        WindModel.Configure(NegativeIntensity));

    FWindModelConfiguration InvalidTime{};
    InvalidTime.TurbulenceCorrelationTimeSeconds = 0.0;

    TestFalse(
        TEXT("Zero correlation time is rejected"),
        WindModel.Configure(InvalidTime));

    FWindModelConfiguration Valid{};
    Valid.SteadyWindVelocityNedMetersPerSecond =
        FVector3{ 2.0, 0.0, 0.0 };

    TestTrue(
        TEXT("Valid configuration is accepted"),
        WindModel.Configure(Valid));

    const FVector3 WindBeforeInvalidUpdate =
        WindModel.GetWindVelocityNedMetersPerSecond();

    TestFalse(
        TEXT("Nonfinite delta time is rejected"),
        WindModel.Update(
            std::numeric_limits<double>::quiet_NaN()));

    TestTrue(
        TEXT("Rejected update preserves wind state"),
        IsVectorNear(
            WindModel.GetWindVelocityNedMetersPerSecond(),
            WindBeforeInvalidUpdate));

    WindModel.Clear();

    TestFalse(
        TEXT("Clear disables the wind model"),
        WindModel.IsConfigured());

    return true;
}

#endif