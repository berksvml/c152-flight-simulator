#if WITH_DEV_AUTOMATION_TESTS

#include "Integration/UnrealCoordinateAdapter.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
	bool IsNear(
		const double Actual,
		const double Expected)
	{
		return std::abs(Actual - Expected) <= 1.0e-9;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FNedPositionConversionTest,
	"C152FlightSim.Integration.Coordinates.NedPosition",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FNedPositionConversionTest::RunTest(
		const FString& Parameters)
{
	using namespace C152;

	(void)Parameters;

	const FlightDynamics::FVector3 NedPosition{
		1.0,
		2.0,
		3.0
	};

	const FVector UnrealPosition =
		UnrealIntegration::FUnrealCoordinateAdapter::
		NedPositionMetersToUnrealCentimeters(
			NedPosition);

	TestTrue(
		TEXT("North maps to positive Unreal X"),
		IsNear(UnrealPosition.X, 100.0));

	TestTrue(
		TEXT("East maps to positive Unreal Y"),
		IsNear(UnrealPosition.Y, 200.0));

	TestTrue(
		TEXT("Down maps to negative Unreal Z"),
		IsNear(UnrealPosition.Z, -300.0));

	const FlightDynamics::FVector3 RoundTripPosition =
		UnrealIntegration::FUnrealCoordinateAdapter::
		UnrealPositionCentimetersToNedMeters(
			UnrealPosition);

	TestTrue(
		TEXT("Position conversion round trip"),
		IsNear(RoundTripPosition.X, NedPosition.X)
		&& IsNear(RoundTripPosition.Y, NedPosition.Y)
		&& IsNear(RoundTripPosition.Z, NedPosition.Z));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FBodyAxesConversionTest,
	"C152FlightSim.Integration.Coordinates.BodyAxes",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FBodyAxesConversionTest::RunTest(
		const FString& Parameters)
{
	using namespace C152;

	(void)Parameters;

	const FVector Forward =
		UnrealIntegration::FUnrealCoordinateAdapter::
		BodyFrdDirectionToUnrealLocal(
			FlightDynamics::FVector3{ 1.0, 0.0, 0.0 });

	const FVector Right =
		UnrealIntegration::FUnrealCoordinateAdapter::
		BodyFrdDirectionToUnrealLocal(
			FlightDynamics::FVector3{ 0.0, 1.0, 0.0 });

	const FVector Down =
		UnrealIntegration::FUnrealCoordinateAdapter::
		BodyFrdDirectionToUnrealLocal(
			FlightDynamics::FVector3{ 0.0, 0.0, 1.0 });

	TestTrue(
		TEXT("Body forward maps to Unreal forward"),
		Forward.Equals(FVector(1.0, 0.0, 0.0)));

	TestTrue(
		TEXT("Body right maps to Unreal right"),
		Right.Equals(FVector(0.0, 1.0, 0.0)));

	TestTrue(
		TEXT("Body down maps to Unreal negative Z"),
		Down.Equals(FVector(0.0, 0.0, -1.0)));

	return true;

}

	IMPLEMENT_SIMPLE_AUTOMATION_TEST(
		FAttitudeConversionTest,
		"C152FlightSim.Integration.Coordinates.Attitude",
		EAutomationTestFlags::EditorContext |
		EAutomationTestFlags::EngineFilter)

		bool FAttitudeConversionTest::RunTest(
			const FString & Parameters)
	{
		using namespace C152;

		(void)Parameters;

		const double RollHalfAngle =
			FMath::DegreesToRadians(30.0) * 0.5;

		const FlightDynamics::FQuaternion CoreRoll{
			std::cos(RollHalfAngle),
			std::sin(RollHalfAngle),
			0.0,
			0.0
		};

		const FQuat UnrealRoll =
			UnrealIntegration::FUnrealCoordinateAdapter::
			BodyToNedAttitudeToUnrealRotation(CoreRoll);

		const FRotator UnrealRollRotator =
			UnrealRoll.Rotator();

		TestTrue(
			TEXT("Positive body roll remains positive in Unreal"),
			FMath::IsNearlyEqual(
				UnrealRollRotator.Roll,
				30.0,
				0.001));

		const double PitchHalfAngle =
			FMath::DegreesToRadians(20.0) * 0.5;

		const FlightDynamics::FQuaternion CorePitch{
			std::cos(PitchHalfAngle),
			0.0,
			std::sin(PitchHalfAngle),
			0.0
		};

		const FQuat UnrealPitch =
			UnrealIntegration::FUnrealCoordinateAdapter::
			BodyToNedAttitudeToUnrealRotation(CorePitch);

		TestTrue(
			TEXT("Positive body pitch remains positive in Unreal"),
			FMath::IsNearlyEqual(
				UnrealPitch.Rotator().Pitch,
				20.0,
				0.001));

		const double YawHalfAngle =
			FMath::DegreesToRadians(40.0) * 0.5;

		const FlightDynamics::FQuaternion CoreYaw{
			std::cos(YawHalfAngle),
			0.0,
			0.0,
			std::sin(YawHalfAngle)
		};

		const FQuat UnrealYaw =
			UnrealIntegration::FUnrealCoordinateAdapter::
			BodyToNedAttitudeToUnrealRotation(CoreYaw);

		TestTrue(
			TEXT("Positive body yaw remains positive in Unreal"),
			FMath::IsNearlyEqual(
				UnrealYaw.Rotator().Yaw,
				40.0,
				0.001));

		const FlightDynamics::FQuaternion RoundTripYaw =
			UnrealIntegration::FUnrealCoordinateAdapter::
			UnrealRotationToBodyToNedAttitude(UnrealYaw);

		TestTrue(
			TEXT("Attitude conversion round trip"),
			IsNear(RoundTripYaw.W, CoreYaw.W)
			&& IsNear(RoundTripYaw.X, CoreYaw.X)
			&& IsNear(RoundTripYaw.Y, CoreYaw.Y)
			&& IsNear(RoundTripYaw.Z, CoreYaw.Z));

		return true;
	}


#endif