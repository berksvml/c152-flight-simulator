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

#endif