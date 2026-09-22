#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/AerodynamicModel.h"
#include "Misc/AutomationTest.h"

#include <cmath>

namespace AerodynamicModelTestData
{
	bool IsAerodynamicModelValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}

	C152::FlightDynamics::FAerodynamicModelConfiguration
		CreateTestConfiguration()
	{
		using namespace C152::FlightDynamics;

		FAerodynamicModelConfiguration Configuration{};

		Configuration.WingAreaSquareMeters =
			10.0;

		Configuration.ReferenceChordMeters =
			2.0;

		Configuration.LiftCoefficientAtZeroAlpha =
			0.4;

		Configuration.LiftCurveSlopePerRadian =
			4.0;

		Configuration.LiftCoefficientPerElevatorRadian =
			1.0;

		Configuration.ZeroLiftDragCoefficient =
			0.02;

		Configuration.InducedDragFactor =
			0.1;

		Configuration.PitchMomentCoefficientAtZeroAlpha =
			0.05;

		Configuration.PitchMomentSlopePerRadian =
			-1.0;

		Configuration.PitchDampingDerivative =
			-10.0;

		Configuration.PitchMomentPerElevatorRadian =
			0.5;

		return Configuration;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAerodynamicConfigurationTest,
	"C152FlightSim.FlightDynamics."
	"Aerodynamics.Configuration",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FAerodynamicConfigurationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FAerodynamicModel Model;

	const FAerodynamicModelConfiguration
		InvalidConfiguration{};

	TestFalse(
		TEXT("Default aerodynamic configuration is rejected"),
		Model.SetConfiguration(InvalidConfiguration));

	TestFalse(
		TEXT("Model remains unconfigured after invalid input"),
		Model.IsConfigured());

	const FAerodynamicModelConfiguration
		ValidConfiguration =
		AerodynamicModelTestData::
		CreateTestConfiguration();

	TestTrue(
		TEXT("Valid aerodynamic configuration is accepted"),
		Model.SetConfiguration(ValidConfiguration));

	TestTrue(
		TEXT("Model reports configured state"),
		Model.IsConfigured());

	Model.ClearConfiguration();

	TestFalse(
		TEXT("Clear removes aerodynamic configuration"),
		Model.IsConfigured());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAerodynamicLongitudinalLoadsTest,
	"C152FlightSim.FlightDynamics."
	"Aerodynamics.LongitudinalLoads",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FAerodynamicLongitudinalLoadsTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FAerodynamicModel Model;

	const FAerodynamicModelConfiguration Configuration =
		AerodynamicModelTestData::
		CreateTestConfiguration();

	TestTrue(
		TEXT("Test configuration is accepted"),
		Model.SetConfiguration(Configuration));

	FAirData AirData{};

	AirData.TrueAirspeedMetersPerSecond =
		20.0;

	AirData.DynamicPressurePascals =
		100.0;

	AirData.AngleOfAttackRadians =
		0.0;

	FControlSurfaceState ControlSurfaceState{};

	ControlSurfaceState.ElevatorRad =
		0.1;

	const FVector3 AngularRateBody{};

	FBodyForcesAndMoments Loads{};
	FAerodynamicCoefficients Coefficients{};

	TestTrue(
		TEXT("Longitudinal aerodynamic evaluation succeeds"),
		Model.TryEvaluate(
			AirData,
			AngularRateBody,
			ControlSurfaceState,
			Loads,
			Coefficients));

	TestTrue(
		TEXT("Lift coefficient is correct"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Coefficients.LiftCoefficient,
			0.5));

	TestTrue(
		TEXT("Drag coefficient is correct"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Coefficients.DragCoefficient,
			0.045));

	TestTrue(
		TEXT("Pitch moment coefficient is correct"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Coefficients.PitchMomentCoefficient,
			0.1));

	TestTrue(
		TEXT("Body X contains negative drag"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Loads.ForceBodyNewtons.X,
			-45.0));

	TestTrue(
		TEXT("Body Z contains upward lift"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Loads.ForceBodyNewtons.Z,
			-500.0));

	TestTrue(
		TEXT("Pitching moment is correct"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Loads.MomentBodyNewtonMeters.Y,
			200.0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAerodynamicPitchDampingTest,
	"C152FlightSim.FlightDynamics."
	"Aerodynamics.AngleOfAttackAndPitchDamping",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FAerodynamicPitchDampingTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	FAerodynamicModel Model;

	TestTrue(
		TEXT("Test configuration is accepted"),
		Model.SetConfiguration(
			AerodynamicModelTestData::
			CreateTestConfiguration()));

	FAirData AirData{};

	AirData.TrueAirspeedMetersPerSecond =
		20.0;

	AirData.DynamicPressurePascals =
		100.0;

	AirData.AngleOfAttackRadians =
		0.1;

	const FVector3 AngularRateBody{
		0.0,
		0.2,
		0.0
	};

	const FControlSurfaceState ControlSurfaceState{};

	FBodyForcesAndMoments Loads{};
	FAerodynamicCoefficients Coefficients{};

	TestTrue(
		TEXT("Aerodynamic evaluation succeeds"),
		Model.TryEvaluate(
			AirData,
			AngularRateBody,
			ControlSurfaceState,
			Loads,
			Coefficients));

	const double ExpectedLiftCoefficient =
		0.8;

	const double ExpectedDragCoefficient =
		0.084;

	// q_hat = q * c / (2V) = 0.01
	const double ExpectedPitchMomentCoefficient =
		-0.15;

	const double ExpectedLiftNewtons =
		800.0;

	const double ExpectedDragNewtons =
		84.0;

	const double ExpectedBodyXNewtons =
		-ExpectedDragNewtons * std::cos(0.1)
		+ ExpectedLiftNewtons * std::sin(0.1);

	const double ExpectedBodyZNewtons =
		-ExpectedDragNewtons * std::sin(0.1)
		- ExpectedLiftNewtons * std::cos(0.1);

	TestTrue(
		TEXT("Angle of attack changes lift coefficient"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Coefficients.LiftCoefficient,
			ExpectedLiftCoefficient));

	TestTrue(
		TEXT("Quadratic drag polar is applied"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Coefficients.DragCoefficient,
			ExpectedDragCoefficient));

	TestTrue(
		TEXT("Pitch-rate damping is applied"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Coefficients.PitchMomentCoefficient,
			ExpectedPitchMomentCoefficient));

	TestTrue(
		TEXT("Lift and drag rotate into body X"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Loads.ForceBodyNewtons.X,
			ExpectedBodyXNewtons));

	TestTrue(
		TEXT("Lift and drag rotate into body Z"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Loads.ForceBodyNewtons.Z,
			ExpectedBodyZNewtons));

	TestTrue(
		TEXT("Pitch damping produces expected moment"),
		AerodynamicModelTestData::
		IsAerodynamicModelValueNear(
			Loads.MomentBodyNewtonMeters.Y,
			-300.0));

	return true;
}

#endif