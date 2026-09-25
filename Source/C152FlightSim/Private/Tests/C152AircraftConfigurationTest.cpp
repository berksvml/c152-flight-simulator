#if WITH_DEV_AUTOMATION_TESTS

#include "FlightDynamics/C152AircraftConfiguration.h"
#include "Misc/AutomationTest.h"

#include <cmath>
#include <limits>

namespace
{
	bool IsAircraftConfigurationValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152AircraftConfigurationReferenceDataTest,
	"C152FlightSim.FlightDynamics.AircraftConfiguration.ReferenceData",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152AircraftConfigurationReferenceDataTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	TestTrue(
		TEXT("1979 Model 152 configuration is valid"),
		Configuration.IsValid());

	TestTrue(
		TEXT("Maximum takeoff mass is 1670 lb in SI units"),
		IsAircraftConfigurationValueNear(
			Configuration.MassAndBalance
			.MaximumTakeoffMassKilograms,
			757.4992579));

	TestTrue(
		TEXT("Maximum ramp mass is 1675 lb in SI units"),
		IsAircraftConfigurationValueNear(
			Configuration.MassAndBalance
			.MaximumRampMassKilograms,
			759.76721975));

	TestTrue(
		TEXT("Light-mass C.G. breakpoint is 1350 lb"),
		IsAircraftConfigurationValueNear(
			Configuration.MassAndBalance
			.LightMassBreakPointKilograms,
			612.3496995));

	TestTrue(
		TEXT("Forward C.G. limit at light mass is 31 inches"),
		IsAircraftConfigurationValueNear(
			Configuration.MassAndBalance
			.ForwardLimitAtOrBelowLightMassMetersAftOfDatum,
			0.7874));

	TestTrue(
		TEXT("Forward C.G. limit at maximum mass is 32.65 inches"),
		IsAircraftConfigurationValueNear(
			Configuration.MassAndBalance
			.ForwardLimitAtMaximumMassMetersAftOfDatum,
			0.82931));

	TestTrue(
		TEXT("Aft C.G. limit is 36.5 inches"),
		IsAircraftConfigurationValueNear(
			Configuration.MassAndBalance
			.AftLimitMetersAftOfDatum,
			0.9271));

	TestTrue(
		TEXT("Rated engine power is 110 mechanical horsepower"),
		IsAircraftConfigurationValueNear(
			Configuration.Propulsion.RatedPowerWatts,
			82026.98587404973,
			1.0e-6));

	TestTrue(
		TEXT("Rated engine speed is 2550 rpm in SI units"),
		IsAircraftConfigurationValueNear(
			Configuration.Propulsion
			.RatedEngineSpeedRadiansPerSecond,
			267.0353755551324));

	TestTrue(
		TEXT("Propeller diameter range matches the TCDS"),
		IsAircraftConfigurationValueNear(
			Configuration.Propulsion
			.MinimumPropellerDiameterMeters,
			1.7145)
		&& IsAircraftConfigurationValueNear(
			Configuration.Propulsion
			.MaximumPropellerDiameterMeters,
			1.7526));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152AircraftConfigurationCgEnvelopeTest,
	"C152FlightSim.FlightDynamics.AircraftConfiguration.CgEnvelope",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152AircraftConfigurationCgEnvelopeTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FC152MassAndBalanceLimits& Limits =
		Configuration.MassAndBalance;

	double ForwardLimit = 0.0;
	double AftLimit = 0.0;

	const bool bMidMassSucceeded =
		Limits.TryGetCenterOfGravityLimits(
			684.9244787,
			ForwardLimit,
			AftLimit);

	TestTrue(
		TEXT("C.G. limits are available inside the mass envelope"),
		bMidMassSucceeded);

	TestTrue(
		TEXT("Forward C.G. limit is linearly interpolated"),
		IsAircraftConfigurationValueNear(
			ForwardLimit,
			0.808355));

	TestTrue(
		TEXT("Aft C.G. limit remains constant"),
		IsAircraftConfigurationValueNear(
			AftLimit,
			0.9271));

	TestTrue(
		TEXT("Forward boundary is accepted"),
		Limits.IsWithinCenterOfGravityEnvelope(
			684.9244787,
			ForwardLimit));

	TestTrue(
		TEXT("Aft boundary is accepted"),
		Limits.IsWithinCenterOfGravityEnvelope(
			684.9244787,
			AftLimit));

	TestFalse(
		TEXT("C.G. ahead of the forward limit is rejected"),
		Limits.IsWithinCenterOfGravityEnvelope(
			684.9244787,
			ForwardLimit - 0.001));

	TestFalse(
		TEXT("C.G. aft of the aft limit is rejected"),
		Limits.IsWithinCenterOfGravityEnvelope(
			684.9244787,
			AftLimit + 0.001));

	double UnchangedForward = 12.0;
	double UnchangedAft = 34.0;

	const bool bOverweightSucceeded =
		Limits.TryGetCenterOfGravityLimits(
			Limits.MaximumTakeoffMassKilograms + 0.001,
			UnchangedForward,
			UnchangedAft);

	TestFalse(
		TEXT("Mass above maximum takeoff mass is rejected"),
		bOverweightSucceeded);

	TestTrue(
		TEXT("Rejected mass leaves output values unchanged"),
		UnchangedForward == 12.0
		&& UnchangedAft == 34.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152AircraftConfigurationValidationTest,
	"C152FlightSim.FlightDynamics.AircraftConfiguration.Validation",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152AircraftConfigurationValidationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration DefaultConfiguration{};

	TestFalse(
		TEXT("Default configuration is invalid"),
		DefaultConfiguration.IsValid());

	FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	Configuration.MassAndBalance
		.MaximumRampMassKilograms =
		Configuration.MassAndBalance
		.MaximumTakeoffMassKilograms - 1.0;

	TestFalse(
		TEXT("Ramp mass below takeoff mass is invalid"),
		Configuration.IsValid());

	Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	Configuration.Propulsion.RatedPowerWatts =
		std::numeric_limits<double>::quiet_NaN();

	TestFalse(
		TEXT("Non-finite propulsion data is invalid"),
		Configuration.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152AircraftGeometryReferenceTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.Geometry",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FC152AircraftGeometryReferenceTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	TestTrue(
		TEXT("Geometry reference is valid"),
		Configuration.Geometry.IsValid());

	TestTrue(
		TEXT("Reference wing area matches the 1979 manual"),
		IsAircraftConfigurationValueNear(
			Configuration.Geometry
			.ReferenceWingAreaSquareMeters,
			14.81803488,
			1.0e-9));

	TestTrue(
		TEXT("Reference wing span matches the 1979 manual"),
		IsAircraftConfigurationValueNear(
			Configuration.Geometry.ReferenceWingSpanMeters,
			10.16,
			1.0e-9));

	TestTrue(
		TEXT("Equivalent rectangular chord is derived correctly"),
		IsAircraftConfigurationValueNear(
			Configuration.Geometry
			.GetEquivalentRectangularChordMeters(),
			1.458468,
			1.0e-9));

	TestTrue(
		TEXT("Wing aspect ratio is derived correctly"),
		IsAircraftConfigurationValueNear(
			Configuration.Geometry.GetAspectRatio(),
			6.966213862765589,
			1.0e-9));

	FC152GeometryReference InvalidGeometry =
		Configuration.Geometry;

	InvalidGeometry.ReferenceWingAreaSquareMeters = 0.0;

	TestFalse(
		TEXT("Zero wing area is rejected"),
		InvalidGeometry.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152DevelopmentMassPropertiesEstimateTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.DevelopmentMassPropertiesEstimate",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FC152DevelopmentMassPropertiesEstimateTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FAircraftMassProperties& Estimate =
		Configuration.DevelopmentMassPropertiesEstimate;

	TestTrue(
		TEXT("Development mass properties estimate is valid"),
		Estimate.IsValid());

	TestTrue(
		TEXT("Reference mass matches the research model"),
		IsAircraftConfigurationValueNear(
			Estimate.MassKilograms,
			650.0));

	TestTrue(
		TEXT("Development roll inertia matches the reference"),
		IsAircraftConfigurationValueNear(
			Estimate.InertiaXxKilogramMetersSquared,
			1420.9));

	TestTrue(
		TEXT("Development pitch inertia matches the reference"),
		IsAircraftConfigurationValueNear(
			Estimate.InertiaYyKilogramMetersSquared,
			4067.5));

	TestTrue(
		TEXT("Development yaw inertia matches the reference"),
		IsAircraftConfigurationValueNear(
			Estimate.InertiaZzKilogramMetersSquared,
			4786.0));

	TestTrue(
		TEXT("Symmetric estimate has zero XZ product of inertia"),
		IsAircraftConfigurationValueNear(
			Estimate.ProductOfInertiaXzKilogramMetersSquared,
			0.0));

	FC152AircraftConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.DevelopmentMassPropertiesEstimate
		.InertiaYyKilogramMetersSquared =
		0.0;

	TestFalse(
		TEXT("Configuration rejects invalid development inertia"),
		InvalidConfiguration.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152DevelopmentAerodynamicEstimateTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.DevelopmentAerodynamics",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FC152DevelopmentAerodynamicEstimateTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FAerodynamicModelConfiguration& Aerodynamics =
		Configuration.DevelopmentAerodynamicEstimate;

	TestTrue(
		TEXT("Development aerodynamic estimate is valid"),
		Aerodynamics.IsValid());

	TestTrue(
		TEXT("Aerodynamic wing area uses aircraft geometry"),
		IsAircraftConfigurationValueNear(
			Aerodynamics.WingAreaSquareMeters,
			Configuration.Geometry
			.ReferenceWingAreaSquareMeters));

	TestTrue(
		TEXT("Aerodynamic chord uses aircraft geometry"),
		IsAircraftConfigurationValueNear(
			Aerodynamics.ReferenceChordMeters,
			Configuration.Geometry
			.GetEquivalentRectangularChordMeters()));

	const double BestGlideLiftCoefficient =
		std::sqrt(
			Aerodynamics.ZeroLiftDragCoefficient
			/ Aerodynamics.InducedDragFactor);

	const double MaximumLiftToDragRatio =
		1.0
		/ (2.0
			* std::sqrt(
				Aerodynamics.ZeroLiftDragCoefficient
				* Aerodynamics.InducedDragFactor));

	TestTrue(
		TEXT("Drag polar reproduces the estimated glide lift coefficient"),
		IsAircraftConfigurationValueNear(
			BestGlideLiftCoefficient,
			0.859,
			0.001));

	TestTrue(
		TEXT("Drag polar reproduces the published approximate glide ratio"),
		IsAircraftConfigurationValueNear(
			MaximumLiftToDragRatio,
			9.72,
			0.02));

	TestTrue(
		TEXT("Lift increases with angle of attack"),
		Aerodynamics.LiftCurveSlopePerRadian > 0.0);

	TestTrue(
		TEXT("Pitch stiffness provides static stability"),
		Aerodynamics.PitchMomentSlopePerRadian < 0.0);

	TestTrue(
		TEXT("Pitch damping opposes pitch rate"),
		Aerodynamics.PitchDampingDerivative < 0.0);

	TestTrue(
		TEXT("Positive elevator command produces nose-up moment"),
		Aerodynamics.PitchMomentPerElevatorRadian > 0.0);

	FC152AircraftConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.DevelopmentAerodynamicEstimate
		.WingAreaSquareMeters =
		0.0;

	TestFalse(
		TEXT("Aircraft configuration rejects invalid aerodynamics"),
		InvalidConfiguration.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152DevelopmentPropulsionEstimateTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.DevelopmentPropulsion",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FC152DevelopmentPropulsionEstimateTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FPropulsionModelConfiguration& Propulsion =
		Configuration.DevelopmentPropulsionEstimate;

	TestTrue(
		TEXT("Development propulsion estimate is valid"),
		Propulsion.IsValid());

	TestTrue(
		TEXT("Propulsion model uses the rated engine power"),
		IsAircraftConfigurationValueNear(
			Propulsion.RatedPowerWatts,
			Configuration.Propulsion.RatedPowerWatts));

	TestTrue(
		TEXT("Propulsion model uses the mean approved propeller diameter"),
		IsAircraftConfigurationValueNear(
			Propulsion.PropellerDiameterMeters,
			1.73355));

	TestTrue(
		TEXT("Initial propeller profile efficiency is configured"),
		IsAircraftConfigurationValueNear(
			Propulsion.PropellerProfileEfficiency,
			0.80));

	TestTrue(
		TEXT("Propulsion model uses ISA sea-level density"),
		IsAircraftConfigurationValueNear(
			Propulsion
			.SeaLevelDensityKilogramsPerCubicMeter,
			1.225));

	FC152AircraftConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.DevelopmentPropulsionEstimate
		.PropellerProfileEfficiency =
		0.0;

	TestFalse(
		TEXT("Aircraft configuration rejects invalid propulsion"),
		InvalidConfiguration.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152DevelopmentFuelEstimateTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.DevelopmentFuel",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FC152DevelopmentFuelEstimateTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FFuelModelConfiguration& Fuel =
		Configuration.DevelopmentFuelEstimate;

	TestTrue(
		TEXT("Development fuel estimate is valid"),
		Fuel.IsValid());

	TestTrue(
		TEXT("Standard usable fuel capacity matches the POH"),
		IsAircraftConfigurationValueNear(
			Fuel.UsableFuelCapacityKilograms,
			66.67807839,
			1.0e-9));

	TestTrue(
		TEXT("Rated-power fuel flow is derived from cruise data"),
		IsAircraftConfigurationValueNear(
			Fuel.FuelFlowAtRatedPowerKilogramsPerSecond,
			0.006148696571111111,
			1.0e-12));

	const double ReferenceCruiseFlowKilogramsPerSecond =
		Fuel.FuelFlowAtRatedPowerKilogramsPerSecond
		* 0.75;

	TestTrue(
		TEXT("Seventy-five percent power reproduces 6.1 GPH"),
		IsAircraftConfigurationValueNear(
			ReferenceCruiseFlowKilogramsPerSecond,
			0.004611522428333333,
			1.0e-12));

	FC152AircraftConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.DevelopmentFuelEstimate
		.UsableFuelCapacityKilograms =
		0.0;

	TestFalse(
		TEXT("Aircraft configuration rejects invalid fuel data"),
		InvalidConfiguration.IsValid());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152PowerplantCompositionTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.PowerplantComposition",
	EAutomationTestFlags::EditorContext
	| EAutomationTestFlags::EngineFilter)

	bool FC152PowerplantCompositionTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	FPowerplantModelConfiguration PowerplantConfiguration{};

	const bool bCompositionSucceeded =
		Configuration
		.TryGetDevelopmentPowerplantConfiguration(
			PowerplantConfiguration);

	TestTrue(
		TEXT("Powerplant configuration composition succeeds"),
		bCompositionSucceeded);

	TestTrue(
		TEXT("Composed powerplant configuration is valid"),
		PowerplantConfiguration.IsValid());

	TestTrue(
		TEXT("Composed propulsion data matches aircraft configuration"),
		IsAircraftConfigurationValueNear(
			PowerplantConfiguration.Propulsion
			.RatedPowerWatts,
			Configuration.DevelopmentPropulsionEstimate
			.RatedPowerWatts)
		&& IsAircraftConfigurationValueNear(
			PowerplantConfiguration.Propulsion
			.PropellerDiameterMeters,
			Configuration.DevelopmentPropulsionEstimate
			.PropellerDiameterMeters));

	TestTrue(
		TEXT("Composed fuel data matches aircraft configuration"),
		IsAircraftConfigurationValueNear(
			PowerplantConfiguration.Fuel
			.UsableFuelCapacityKilograms,
			Configuration.DevelopmentFuelEstimate
			.UsableFuelCapacityKilograms)
		&& IsAircraftConfigurationValueNear(
			PowerplantConfiguration.Fuel
			.FuelFlowAtRatedPowerKilogramsPerSecond,
			Configuration.DevelopmentFuelEstimate
			.FuelFlowAtRatedPowerKilogramsPerSecond));

	FPowerplantModel PowerplantModel;

	TestTrue(
		TEXT("Composed configuration can configure powerplant model"),
		PowerplantModel.SetConfiguration(
			PowerplantConfiguration));

	FC152AircraftConfiguration InvalidConfiguration =
		Configuration;

	InvalidConfiguration.DevelopmentFuelEstimate
		.UsableFuelCapacityKilograms =
		0.0;

	FPowerplantModelConfiguration UnchangedOutput{};

	UnchangedOutput.Propulsion.RatedPowerWatts =
		123.0;

	UnchangedOutput.Fuel.UsableFuelCapacityKilograms =
		456.0;

	const bool bInvalidCompositionSucceeded =
		InvalidConfiguration
		.TryGetDevelopmentPowerplantConfiguration(
			UnchangedOutput);

	TestFalse(
		TEXT("Invalid source configuration is rejected"),
		bInvalidCompositionSucceeded);

	TestTrue(
		TEXT("Rejected composition leaves output unchanged"),
		UnchangedOutput.Propulsion.RatedPowerWatts
		== 123.0
		&& UnchangedOutput.Fuel
		.UsableFuelCapacityKilograms
		== 456.0);

	return true;
}

namespace C152GroundReactionConfigurationTestSupport
{
	bool IsValueNear(
		const double Actual,
		const double Expected,
		const double Tolerance = 1.0e-9)
	{
		return std::abs(Actual - Expected)
			<= Tolerance;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FC152DevelopmentGroundReactionConfigurationTest,
	"C152FlightSim.FlightDynamics."
	"AircraftConfiguration.DevelopmentGroundReaction",
	EAutomationTestFlags::EditorContext |
	EAutomationTestFlags::EngineFilter)

	bool FC152DevelopmentGroundReactionConfigurationTest::RunTest(
		const FString& Parameters)
{
	using namespace C152::FlightDynamics;
	using namespace
		C152GroundReactionConfigurationTestSupport;

	(void)Parameters;

	const FC152AircraftConfiguration Configuration =
		FC152AircraftConfiguration::Create1979Model152();

	const FGroundReactionModelConfiguration& GroundReaction =
		Configuration.DevelopmentGroundReactionEstimate;

	TestTrue(
		TEXT("Development ground-reaction configuration is valid"),
		GroundReaction.IsValid());

	const FGroundContactPointConfiguration& NoseGear =
		GroundReaction.ContactPoints[0];

	const FGroundContactPointConfiguration& LeftMainGear =
		GroundReaction.ContactPoints[1];

	const FGroundContactPointConfiguration& RightMainGear =
		GroundReaction.ContactPoints[2];

	constexpr double MetersPerInch =
		0.0254;

	constexpr double ExpectedWheelbaseMeters =
		58.0 * MetersPerInch;

	constexpr double ExpectedMainGearTrackMeters =
		(7.0 * 12.0 + 7.25) * MetersPerInch;

	const double MainGearLongitudinalPositionMeters =
		0.5
		* (LeftMainGear.PositionBodyMeters.X
			+ RightMainGear.PositionBodyMeters.X);

	const double ActualWheelbaseMeters =
		NoseGear.PositionBodyMeters.X
		- MainGearLongitudinalPositionMeters;

	const double ActualMainGearTrackMeters =
		RightMainGear.PositionBodyMeters.Y
		- LeftMainGear.PositionBodyMeters.Y;

	TestTrue(
		TEXT("Wheelbase matches the 58-inch reference"),
		IsValueNear(
			ActualWheelbaseMeters,
			ExpectedWheelbaseMeters));

	TestTrue(
		TEXT("Main-gear track matches the 7-foot 7.25-inch reference"),
		IsValueNear(
			ActualMainGearTrackMeters,
			ExpectedMainGearTrackMeters));

	TestTrue(
		TEXT("Nose gear is forward of the aircraft origin"),
		NoseGear.PositionBodyMeters.X > 0.0);

	TestTrue(
		TEXT("Main gears are aft of the aircraft origin"),
		LeftMainGear.PositionBodyMeters.X < 0.0
		&& RightMainGear.PositionBodyMeters.X < 0.0);

	TestTrue(
		TEXT("Main gears are laterally symmetric"),
		IsValueNear(
			LeftMainGear.PositionBodyMeters.Y,
			-RightMainGear.PositionBodyMeters.Y));

	TestTrue(
		TEXT("All ground contacts use the same vertical offset"),
		IsValueNear(
			NoseGear.PositionBodyMeters.Z,
			LeftMainGear.PositionBodyMeters.Z)
		&& IsValueNear(
			LeftMainGear.PositionBodyMeters.Z,
			RightMainGear.PositionBodyMeters.Z));

	TestTrue(
		TEXT("Nose wheel has no braking authority"),
		IsValueNear(
			NoseGear.BrakingAuthority,
			0.0));

	TestTrue(
		TEXT("Both main wheels have full braking authority"),
		IsValueNear(
			LeftMainGear.BrakingAuthority,
			1.0)
		&& IsValueNear(
			RightMainGear.BrakingAuthority,
			1.0));

	return true;
}

#endif