#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/AircraftMassProperties.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
	bool IsMassPropertyValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	bool IsVectorNear(
		const C152::FlightDynamics::FVector3& Actual,
		const C152::FlightDynamics::FVector3& Expected,
		const double Tolerance = 1.0e-9)
	{
		return IsMassPropertyValueNear(Actual.X, Expected.X, Tolerance)
			&& IsMassPropertyValueNear(Actual.Y, Expected.Y, Tolerance)
			&& IsMassPropertyValueNear(Actual.Z, Expected.Z, Tolerance);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMassPropertiesValidationTest,
	"C152FlightSim.FlightDynamics.MassProperties.Validation",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FMassPropertiesValidationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FAircraftMassProperties DefaultProperties{};

	TestFalse(
		TEXT("Default mass properties are invalid"),
		DefaultProperties.IsValid());

	FAircraftMassProperties ValidProperties{};

	ValidProperties.MassKilograms = 1000.0;
	ValidProperties.InertiaXxKilogramMetersSquared = 10.0;
	ValidProperties.InertiaYyKilogramMetersSquared = 20.0;
	ValidProperties.InertiaZzKilogramMetersSquared = 30.0;
	ValidProperties.ProductOfInertiaXzKilogramMetersSquared =
		2.0;

	TestTrue(
		TEXT("Positive-definite mass properties are valid"),
		ValidProperties.IsValid());

	ValidProperties.ProductOfInertiaXzKilogramMetersSquared =
		std::sqrt(
			ValidProperties
			.InertiaXxKilogramMetersSquared
			* ValidProperties
			.InertiaZzKilogramMetersSquared);

	TestFalse(
		TEXT("Singular inertia tensor is invalid"),
		ValidProperties.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMassPropertiesInertiaRoundTripTest,
	"C152FlightSim.FlightDynamics.MassProperties.InertiaRoundTrip",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FMassPropertiesInertiaRoundTripTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FAircraftMassProperties MassProperties{};

	MassProperties.MassKilograms = 1000.0;
	MassProperties.InertiaXxKilogramMetersSquared = 10.0;
	MassProperties.InertiaYyKilogramMetersSquared = 20.0;
	MassProperties.InertiaZzKilogramMetersSquared = 30.0;
	MassProperties.ProductOfInertiaXzKilogramMetersSquared =
		2.0;

	const FVector3 OriginalVector{
		3.0,
		4.0,
		5.0
	};

	const FVector3 InertiaApplied =
		MassProperties.ApplyInertia(
			OriginalVector);

	TestTrue(
		TEXT("Inertia tensor multiplication is correct"),
		IsVectorNear(
			InertiaApplied,
			FVector3{
				20.0,
				80.0,
				144.0
			}));

	FVector3 RecoveredVector{};

	const bool bInverseSucceeded =
		MassProperties.TryApplyInverseInertia(
			InertiaApplied,
			RecoveredVector);

	TestTrue(
		TEXT("Inverse inertia operation succeeds"),
		bInverseSucceeded);

	TestTrue(
		TEXT("Inertia round trip recovers the original vector"),
		IsVectorNear(
			RecoveredVector,
			OriginalVector));

	return true;
}

#endif