#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/RigidBody6DofModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
    constexpr double GravityMetersPerSecondSquared = 9.80665;
    constexpr double FixedDeltaTimeSeconds = 1.0 / 120.0;

    bool IsRigidBodyValueNear(
        const double Actual,
        const double Expected,
        const double Tolerance = 1.0e-8)
    {
        return std::abs(Actual - Expected) <= Tolerance;
    }

    C152::FlightDynamics::FAircraftMassProperties
        MakeTestMassProperties()
    {
        C152::FlightDynamics::FAircraftMassProperties Properties;

        Properties.MassKilograms = 2.0;
        Properties.InertiaXxKilogramMetersSquared = 2.0;
        Properties.InertiaYyKilogramMetersSquared = 2.0;
        Properties.InertiaZzKilogramMetersSquared = 2.0;
        Properties.ProductOfInertiaXzKilogramMetersSquared = 0.0;

        return Properties;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FRigidBodyConstantForceTest,
    "C152FlightSim.FlightDynamics.RigidBody6Dof.ConstantForce",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FRigidBodyConstantForceTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    FRigidBody6DofModel Model;
    FAircraftState State{};
    FBodyForcesAndMoments Loads{};

    Loads.ForceBodyNewtons = { 4.0, 0.0, 0.0 };

    const bool bSucceeded = Model.Propagate(
        State,
        Loads,
        MakeTestMassProperties(),
        FVector3{},
        0.5);

    TestTrue(
        TEXT("Constant force propagation succeeds"),
        bSucceeded);

    TestTrue(
        TEXT("Constant force produces expected velocity"),
        IsRigidBodyValueNear(
            State.VelocityBodyMetersPerSecond.X,
            1.0));

    TestTrue(
        TEXT("Trapezoidal position integration is correct"),
        IsRigidBodyValueNear(
            State.PositionNedMeters.X,
            0.25));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FRigidBodyGravityTest,
    "C152FlightSim.FlightDynamics.RigidBody6Dof.Gravity",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FRigidBodyGravityTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    FRigidBody6DofModel Model;
    FAircraftState State{};
    const FBodyForcesAndMoments Loads{};

    bool bAllStepsSucceeded = true;

    for (int StepIndex = 0; StepIndex < 120; ++StepIndex)
    {
        const bool bStepSucceeded = Model.Propagate(
            State,
            Loads,
            MakeTestMassProperties(),
            FVector3{
                0.0,
                0.0,
                GravityMetersPerSecondSquared
            },
            FixedDeltaTimeSeconds);

        bAllStepsSucceeded =
            bAllStepsSucceeded && bStepSucceeded;
    }

    TestTrue(
        TEXT("All gravity propagation steps succeed"),
        bAllStepsSucceeded);

    TestTrue(
        TEXT("Gravity produces expected downward velocity"),
        IsRigidBodyValueNear(
            State.VelocityBodyMetersPerSecond.Z,
            GravityMetersPerSecondSquared));

    TestTrue(
        TEXT("Gravity produces expected downward position"),
        IsRigidBodyValueNear(
            State.PositionNedMeters.Z,
            0.5 * GravityMetersPerSecondSquared));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FRigidBodyConstantYawRateTest,
    "C152FlightSim.FlightDynamics.RigidBody6Dof.ConstantYawRate",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FRigidBodyConstantYawRateTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    constexpr double Pi = 3.14159265358979323846;

    FRigidBody6DofModel Model;
    FAircraftState State{};
    const FBodyForcesAndMoments Loads{};

    State.AngularRateBodyRadiansPerSecond = {
        0.0,
        0.0,
        0.5 * Pi
    };

    bool bAllStepsSucceeded = true;

    for (int StepIndex = 0; StepIndex < 120; ++StepIndex)
    {
        const bool bStepSucceeded = Model.Propagate(
            State,
            Loads,
            MakeTestMassProperties(),
            FVector3{},
            FixedDeltaTimeSeconds);

        bAllStepsSucceeded =
            bAllStepsSucceeded && bStepSucceeded;
    }

    const FVector3 RotatedForward =
        State.AttitudeBodyToNed.RotateVector({
            1.0,
            0.0,
            0.0
            });

    TestTrue(
        TEXT("All yaw propagation steps succeed"),
        bAllStepsSucceeded);

    TestTrue(
        TEXT("Positive yaw rotates north toward east"),
        IsRigidBodyValueNear(RotatedForward.X, 0.0, 1.0e-6)
        && IsRigidBodyValueNear(RotatedForward.Y, 1.0, 1.0e-6)
        && IsRigidBodyValueNear(RotatedForward.Z, 0.0, 1.0e-6));

    TestTrue(
        TEXT("Attitude quaternion remains normalized"),
        IsRigidBodyValueNear(
            State.AttitudeBodyToNed.Norm(),
            1.0,
            1.0e-10));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FRigidBodyConstantMomentTest,
    "C152FlightSim.FlightDynamics.RigidBody6Dof.ConstantMoment",
    EAutomationTestFlags::EditorContext |
    EAutomationTestFlags::EngineFilter)

    bool FRigidBodyConstantMomentTest::RunTest(
        const FString& Parameters)
{
    using namespace C152::FlightDynamics;

    (void)Parameters;

    FRigidBody6DofModel Model;
    FAircraftState State{};
    FBodyForcesAndMoments Loads{};

    Loads.MomentBodyNewtonMeters = {
        0.0,
        0.0,
        4.0
    };

    bool bAllStepsSucceeded = true;

    for (int StepIndex = 0; StepIndex < 120; ++StepIndex)
    {
        const bool bStepSucceeded = Model.Propagate(
            State,
            Loads,
            MakeTestMassProperties(),
            FVector3{},
            FixedDeltaTimeSeconds);

        bAllStepsSucceeded =
            bAllStepsSucceeded && bStepSucceeded;
    }

    TestTrue(
        TEXT("All moment propagation steps succeed"),
        bAllStepsSucceeded);

    TestTrue(
        TEXT("Constant moment produces expected yaw rate"),
        IsRigidBodyValueNear(
            State.AngularRateBodyRadiansPerSecond.Z,
            2.0));

    const FVector3 RotatedForward =
        State.AttitudeBodyToNed.RotateVector({
            1.0,
            0.0,
            0.0
            });

    TestTrue(
        TEXT("Constant moment produces expected yaw angle"),
        IsRigidBodyValueNear(RotatedForward.X, std::cos(1.0), 1.0e-6)
        && IsRigidBodyValueNear(RotatedForward.Y, std::sin(1.0), 1.0e-6));

    return true;
}

#endif