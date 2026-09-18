#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/FlightDynamicsTypes.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace
{
	constexpr double Pi =
		3.14159265358979323846;

	bool IsNear(
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
		return IsNear(Actual.X, Expected.X, Tolerance)
			&& IsNear(Actual.Y, Expected.Y, Tolerance)
			&& IsNear(Actual.Z, Expected.Z, Tolerance);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FVectorAlgebraTest,
	"C152FlightSim.FlightDynamics.Math.VectorAlgebra",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FVectorAlgebraTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FVector3 First{ 1.0, 2.0, 3.0 };
	const FVector3 Second{ 4.0, -5.0, 6.0 };

	TestTrue(
		TEXT("Vector addition is correct"),
		IsVectorNear(
			First + Second,
			FVector3{ 5.0, -3.0, 9.0 }));

	TestTrue(
		TEXT("Vector subtraction is correct"),
		IsVectorNear(
			First - Second,
			FVector3{ -3.0, 7.0, -3.0 }));

	TestTrue(
		TEXT("Vector scalar multiplication is correct"),
		IsVectorNear(
			2.0 * First,
			FVector3{ 2.0, 4.0, 6.0 }));

	TestTrue(
		TEXT("Vector scalar division is correct"),
		IsVectorNear(
			First / 2.0,
			FVector3{ 0.5, 1.0, 1.5 }));

	TestTrue(
		TEXT("Vector dot product is correct"),
		IsNear(
			Dot(First, Second),
			12.0));

	TestTrue(
		TEXT("Vector cross product is correct"),
		IsVectorNear(
			Cross(First, Second),
			FVector3{ 27.0, 6.0, -13.0 }));

	TestTrue(
		TEXT("FRD basis follows the right-hand rule"),
		IsVectorNear(
			Cross(
				FVector3{ 1.0, 0.0, 0.0 },
				FVector3{ 0.0, 1.0, 0.0 }),
			FVector3{ 0.0, 0.0, 1.0 }));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FVectorNormalizationTest,
	"C152FlightSim.FlightDynamics.Math.VectorNormalization",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FVectorNormalizationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FVector3 Vector{ 3.0, 4.0, 0.0 };

	const bool bNormalized =
		Vector.Normalize();

	TestTrue(
		TEXT("Non-zero vector normalization succeeds"),
		bNormalized);

	TestTrue(
		TEXT("Normalized vector has unit length"),
		IsNear(Vector.Norm(), 1.0));

	TestTrue(
		TEXT("Normalized vector direction is preserved"),
		IsVectorNear(
			Vector,
			FVector3{ 0.6, 0.8, 0.0 }));

	FVector3 ZeroVector{};

	const bool bZeroNormalized =
		ZeroVector.Normalize();

	TestFalse(
		TEXT("Zero vector normalization fails"),
		bZeroNormalized);

	TestTrue(
		TEXT("Failed normalization returns a zero vector"),
		IsVectorNear(
			ZeroVector,
			FVector3{}));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FQuaternionRotationTest,
	"C152FlightSim.FlightDynamics.Math.QuaternionRotation",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FQuaternionRotationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const double HalfAngle =
		(Pi * 0.5) * 0.5;

	const FQuaternion PositiveYaw90{
		std::cos(HalfAngle),
		0.0,
		0.0,
		std::sin(HalfAngle)
	};

	const FVector3 ForwardNed =
		PositiveYaw90.RotateVector(
			FVector3{ 1.0, 0.0, 0.0 });

	TestTrue(
		TEXT("Positive yaw rotates forward toward east"),
		IsVectorNear(
			ForwardNed,
			FVector3{ 0.0, 1.0, 0.0 }));

	const FQuaternion PositivePitch90{
		std::cos(HalfAngle),
		0.0,
		std::sin(HalfAngle),
		0.0
	};

	const FVector3 ForwardAfterPitch =
		PositivePitch90.RotateVector(
			FVector3{ 1.0, 0.0, 0.0 });

	TestTrue(
		TEXT("Positive pitch rotates forward toward NED up"),
		IsVectorNear(
			ForwardAfterPitch,
			FVector3{ 0.0, 0.0, -1.0 }));

	const FQuaternion PositiveRoll90{
		std::cos(HalfAngle),
		std::sin(HalfAngle),
		0.0,
		0.0
	};

	const FVector3 RightAfterRoll =
		PositiveRoll90.RotateVector(
			FVector3{ 0.0, 1.0, 0.0 });

	TestTrue(
		TEXT("Positive roll rotates right toward NED down"),
		IsVectorNear(
			RightAfterRoll,
			FVector3{ 0.0, 0.0, 1.0 }));

	const FVector3 OriginalVector{
		2.0,
		-3.0,
		4.0
	};

	const FVector3 RoundTripVector =
		PositiveYaw90.InverseRotateVector(
			PositiveYaw90.RotateVector(
				OriginalVector));

	TestTrue(
		TEXT("Quaternion vector rotation survives round trip"),
		IsVectorNear(
			RoundTripVector,
			OriginalVector));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FQuaternionCompositionTest,
	"C152FlightSim.FlightDynamics.Math.QuaternionComposition",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FQuaternionCompositionTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const double Yaw45HalfAngle =
		(Pi * 0.25) * 0.5;

	const FQuaternion Yaw45{
		std::cos(Yaw45HalfAngle),
		0.0,
		0.0,
		std::sin(Yaw45HalfAngle)
	};

	FQuaternion Yaw90 =
		Yaw45 * Yaw45;

	Yaw90.Normalize();

	const FVector3 RotatedForward =
		Yaw90.RotateVector(
			FVector3{ 1.0, 0.0, 0.0 });

	TestTrue(
		TEXT("Two 45 degree yaw rotations produce 90 degrees"),
		IsVectorNear(
			RotatedForward,
			FVector3{ 0.0, 1.0, 0.0 }));

	TestTrue(
		TEXT("Quaternion composition remains normalized"),
		Yaw90.IsNormalized());

	return true;
}

#endif