#include "FlightDynamics/C152AircraftConfiguration.h"

#include <cmath>

namespace C152AircraftConfigurationConstants
{
	constexpr double MinimumPositiveValue =
		1.0e-12;

	constexpr double KilogramsPerPound =
		0.45359237;

	constexpr double MetersPerInch =
		0.0254;

	constexpr double MetersPerFoot =
		0.3048;

	constexpr double SquareMetersPerSquareFoot =
		0.09290304;

	constexpr double WattsPerMechanicalHorsepower =
		745.6998715822702;

	constexpr double TwoPiRadians =
		6.28318530717958647692;

	constexpr double SecondsPerMinute =
		60.0;

	constexpr double RevolutionsPerMinuteToRadiansPerSecond(
		const double RevolutionsPerMinute)
	{
		return RevolutionsPerMinute
			* TwoPiRadians
			/ SecondsPerMinute;
	}

	constexpr double SecondsPerHour =
		3600.0;

	constexpr double FuelWeightPoundsPerUsGallon =
		6.0;

	constexpr double StandardUsableFuelGallons =
		24.5;

	constexpr double ReferenceCruisePowerFraction =
		0.75;

	constexpr double ReferenceCruiseFuelFlowGallonsPerHour =
		6.1;

	constexpr double UsGallonsPerHourToKilogramsPerSecond(
		const double GallonsPerHour)
	{
		return GallonsPerHour
			* FuelWeightPoundsPerUsGallon
			* KilogramsPerPound
			/ SecondsPerHour;
	}
}

namespace C152::FlightDynamics
{
	bool FC152GeometryReference::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(ReferenceWingAreaSquareMeters)
			&& std::isfinite(ReferenceWingSpanMeters);

		if (!bValuesAreFinite)
		{
			return false;
		}

		return ReferenceWingAreaSquareMeters
				> C152AircraftConfigurationConstants::
			MinimumPositiveValue
			&& ReferenceWingSpanMeters
				> C152AircraftConfigurationConstants::
			MinimumPositiveValue;
	}

	double FC152GeometryReference::
		GetEquivalentRectangularChordMeters() const
	{
		if (!IsValid())
		{
			return 0.0;
		}

		return ReferenceWingAreaSquareMeters
			/ ReferenceWingSpanMeters;
	}

	double FC152GeometryReference::GetAspectRatio() const
	{
		if (!IsValid())
		{
			return 0.0;
		}

		return ReferenceWingSpanMeters
			* ReferenceWingSpanMeters
			/ ReferenceWingAreaSquareMeters;
	}

	bool FC152MassAndBalanceLimits::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(MaximumTakeoffMassKilograms)
			&& std::isfinite(MaximumRampMassKilograms)
			&& std::isfinite(LightMassBreakPointKilograms)
			&& std::isfinite(
				ForwardLimitAtOrBelowLightMassMetersAftOfDatum)
			&& std::isfinite(
				ForwardLimitAtMaximumMassMetersAftOfDatum)
			&& std::isfinite(
				AftLimitMetersAftOfDatum);

		if (!bValuesAreFinite)
		{
			return false;
		}

		constexpr double MinimumAllowedPositiveValue =
			C152AircraftConfigurationConstants::
			MinimumPositiveValue;

		if (MaximumTakeoffMassKilograms
			<= MinimumAllowedPositiveValue
			|| MaximumRampMassKilograms
			< MaximumTakeoffMassKilograms
			|| LightMassBreakPointKilograms
			<= MinimumAllowedPositiveValue
			|| LightMassBreakPointKilograms
			>= MaximumTakeoffMassKilograms)
		{
			return false;
		}

		if (ForwardLimitAtOrBelowLightMassMetersAftOfDatum
			<= MinimumAllowedPositiveValue
			|| ForwardLimitAtMaximumMassMetersAftOfDatum
			< ForwardLimitAtOrBelowLightMassMetersAftOfDatum
			|| AftLimitMetersAftOfDatum
			< ForwardLimitAtMaximumMassMetersAftOfDatum)
		{
			return false;
		}

		return true;
	}

	bool FC152MassAndBalanceLimits::
		TryGetCenterOfGravityLimits(
			const double MassKilograms,
			double& OutForwardLimitMetersAftOfDatum,
			double& OutAftLimitMetersAftOfDatum) const
	{
		if (!IsValid()
			|| !std::isfinite(MassKilograms)
			|| MassKilograms
			<= C152AircraftConfigurationConstants::
			MinimumPositiveValue
			|| MassKilograms > MaximumTakeoffMassKilograms)
		{
			return false;
		}

		double ForwardLimit =
			ForwardLimitAtOrBelowLightMassMetersAftOfDatum;

		if (MassKilograms > LightMassBreakPointKilograms)
		{
			const double InterpolationRatio =
				(MassKilograms
					- LightMassBreakPointKilograms)
				/ (MaximumTakeoffMassKilograms
					- LightMassBreakPointKilograms);

			ForwardLimit +=
				InterpolationRatio
				* (ForwardLimitAtMaximumMassMetersAftOfDatum
					- ForwardLimitAtOrBelowLightMassMetersAftOfDatum);
		}

		if (!std::isfinite(ForwardLimit))
		{
			return false;
		}

		OutForwardLimitMetersAftOfDatum =
			ForwardLimit;

		OutAftLimitMetersAftOfDatum =
			AftLimitMetersAftOfDatum;

		return true;
	}

	bool FC152MassAndBalanceLimits::
		IsWithinCenterOfGravityEnvelope(
			const double MassKilograms,
			const double CenterOfGravityMetersAftOfDatum) const
	{
		if (!std::isfinite(
			CenterOfGravityMetersAftOfDatum))
		{
			return false;
		}

		double ForwardLimit = 0.0;
		double AftLimit = 0.0;

		if (!TryGetCenterOfGravityLimits(
			MassKilograms,
			ForwardLimit,
			AftLimit))
		{
			return false;
		}

		return CenterOfGravityMetersAftOfDatum
			>= ForwardLimit
			&& CenterOfGravityMetersAftOfDatum
			<= AftLimit;
	}

	bool FC152PropulsionReference::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(RatedPowerWatts)
			&& std::isfinite(
				RatedEngineSpeedRadiansPerSecond)
			&& std::isfinite(
				MinimumPropellerDiameterMeters)
			&& std::isfinite(
				MaximumPropellerDiameterMeters);

		if (!bValuesAreFinite)
		{
			return false;
		}

		return RatedPowerWatts
			> C152AircraftConfigurationConstants::
			MinimumPositiveValue
			&& RatedEngineSpeedRadiansPerSecond
				> C152AircraftConfigurationConstants::
			MinimumPositiveValue
			&& MinimumPropellerDiameterMeters
				> C152AircraftConfigurationConstants::
			MinimumPositiveValue
			&& MaximumPropellerDiameterMeters
			>= MinimumPropellerDiameterMeters;
	}

	bool FC152AircraftConfiguration::IsValid() const
	{
		return MassAndBalance.IsValid()
			&& Geometry.IsValid()
			&& DevelopmentAerodynamicEstimate.IsValid()
			&& DevelopmentMassPropertiesEstimate.IsValid()
			&& Propulsion.IsValid()
			&& DevelopmentPropulsionEstimate.IsValid()
			&& DevelopmentFuelEstimate.IsValid();
	}

	FC152AircraftConfiguration
		FC152AircraftConfiguration::Create1979Model152()
	{
		FC152AircraftConfiguration Configuration{};

		// FAA TCDS 3A19, Model 152.

		// Cessna 1979 Model 152 Information Manual, Figure 1-1.
		// Wing area: 159.5 ft^2.
		// Span: 33 ft 4 in with conical-camber wing tips and strobes.
		Configuration.Geometry.ReferenceWingAreaSquareMeters =
			159.5
			* C152AircraftConfigurationConstants::
			SquareMetersPerSquareFoot;

		Configuration.Geometry.ReferenceWingSpanMeters =
			(33.0 + 4.0 / 12.0)
			* C152AircraftConfigurationConstants::
			MetersPerFoot;

		// Initial longitudinal aerodynamic development estimate.
//
// The drag polar is derived approximately from the published
// maximum-glide condition:
// - 60 KCAS
// - maximum takeoff mass
// - ISA sea-level density
// - approximately 16 NM horizontal distance per 10,000 ft
//
// The resulting engine-out polar includes the influence of a
// windmilling propeller and is not treated as a powered-flight
// manufacturer polar.
//
// Lift, stability, damping, and elevator derivatives below are
// initial engineering estimates. They must later be calibrated
// against trim, stall, climb, and flight-test reference data.

		FAerodynamicModelConfiguration& Aerodynamics =
			Configuration.DevelopmentAerodynamicEstimate;

		Aerodynamics.WingAreaSquareMeters =
			Configuration.Geometry.ReferenceWingAreaSquareMeters;

		Aerodynamics.ReferenceChordMeters =
			Configuration.Geometry
			.GetEquivalentRectangularChordMeters();

		Aerodynamics.LiftCoefficientAtZeroAlpha =
			0.25;

		Aerodynamics.LiftCurveSlopePerRadian =
			4.8;

		// Positive elevator state represents a nose-up command.
		// The direct lift contribution is negative because additional
		// tail downforce reduces total aircraft lift.
		Aerodynamics.LiftCoefficientPerElevatorRadian =
			-0.30;

		Aerodynamics.ZeroLiftDragCoefficient =
			0.0442;

		Aerodynamics.InducedDragFactor =
			0.0599;

		Aerodynamics.PitchMomentCoefficientAtZeroAlpha =
			0.05;

		// Negative value provides longitudinal static stability.
		Aerodynamics.PitchMomentSlopePerRadian =
			-0.80;

		// Negative value opposes pitch rate.
		Aerodynamics.PitchDampingDerivative =
			-12.0;

		// Positive elevator state produces a nose-up pitching moment.
		Aerodynamics.PitchMomentPerElevatorRadian =
			1.10;

		// Development estimate based on the simplified Cessna 152
		// model documented by Krawczyk et al. (2024),
		// Aerospace 11(10), 830, Table 3.
		// DOI: 10.3390/aerospace11100830
		//
		// The paper assumes uniform component mass distributions.
		// These values are not measured or manufacturer-published
		// Cessna 152 mass properties.

		Configuration.DevelopmentMassPropertiesEstimate.MassKilograms =
			650.0;

		Configuration.DevelopmentMassPropertiesEstimate
			.InertiaXxKilogramMetersSquared =
			1420.9;

		Configuration.DevelopmentMassPropertiesEstimate
			.InertiaYyKilogramMetersSquared =
			4067.5;

		Configuration.DevelopmentMassPropertiesEstimate
			.InertiaZzKilogramMetersSquared =
			4786.0;

		// The simplified symmetric reference model does not provide
		// a non-zero XZ product of inertia.
		Configuration.DevelopmentMassPropertiesEstimate
			.ProductOfInertiaXzKilogramMetersSquared =
			0.0;

		// FAA TCDS 3A19, Model 152.

		Configuration.MassAndBalance
			.MaximumTakeoffMassKilograms =
			1670.0
			* C152AircraftConfigurationConstants::
			KilogramsPerPound;

		Configuration.MassAndBalance
			.MaximumRampMassKilograms =
			1675.0
			* C152AircraftConfigurationConstants::
			KilogramsPerPound;

		Configuration.MassAndBalance
			.LightMassBreakPointKilograms =
			1350.0
			* C152AircraftConfigurationConstants::
			KilogramsPerPound;

		Configuration.MassAndBalance
			.ForwardLimitAtOrBelowLightMassMetersAftOfDatum =
			31.0
			* C152AircraftConfigurationConstants::
			MetersPerInch;

		Configuration.MassAndBalance
			.ForwardLimitAtMaximumMassMetersAftOfDatum =
			32.65
			* C152AircraftConfigurationConstants::
			MetersPerInch;

		Configuration.MassAndBalance
			.AftLimitMetersAftOfDatum =
			36.5
			* C152AircraftConfigurationConstants::
			MetersPerInch;

		// Lycoming O-235-L2C reference rating.
		Configuration.Propulsion.RatedPowerWatts =
			110.0
			* C152AircraftConfigurationConstants::
			WattsPerMechanicalHorsepower;

		Configuration.Propulsion
			.RatedEngineSpeedRadiansPerSecond =
			C152AircraftConfigurationConstants::
			RevolutionsPerMinuteToRadiansPerSecond(
				2550.0);

		Configuration.Propulsion
			.MinimumPropellerDiameterMeters =
			67.5
			* C152AircraftConfigurationConstants::
			MetersPerInch;

		Configuration.Propulsion
			.MaximumPropellerDiameterMeters =
			69.0
			* C152AircraftConfigurationConstants::
			MetersPerInch;

		FPropulsionModelConfiguration& PropulsionEstimate =
			Configuration.DevelopmentPropulsionEstimate;

		PropulsionEstimate.RatedPowerWatts =
			Configuration.Propulsion.RatedPowerWatts;

		PropulsionEstimate.PropellerDiameterMeters =
			0.5
			* (Configuration.Propulsion
				.MinimumPropellerDiameterMeters
				+ Configuration.Propulsion
				.MaximumPropellerDiameterMeters);

		PropulsionEstimate.PropellerProfileEfficiency =
			0.80;

		PropulsionEstimate
			.SeaLevelDensityKilogramsPerCubicMeter =
			1.225;

		FFuelModelConfiguration& FuelEstimate =
			Configuration.DevelopmentFuelEstimate;

		FuelEstimate.UsableFuelCapacityKilograms =
			C152AircraftConfigurationConstants::
			StandardUsableFuelGallons
			* C152AircraftConfigurationConstants::
			FuelWeightPoundsPerUsGallon
			* C152AircraftConfigurationConstants::
			KilogramsPerPound;

		const double EstimatedRatedPowerFuelFlowGallonsPerHour =
			C152AircraftConfigurationConstants::
			ReferenceCruiseFuelFlowGallonsPerHour
			/ C152AircraftConfigurationConstants::
			ReferenceCruisePowerFraction;

		FuelEstimate.FuelFlowAtRatedPowerKilogramsPerSecond =
			C152AircraftConfigurationConstants::
			UsGallonsPerHourToKilogramsPerSecond(
				EstimatedRatedPowerFuelFlowGallonsPerHour);

		return Configuration;
	}
	
}