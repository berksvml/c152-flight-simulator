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
			&& Propulsion.IsValid();
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

		return Configuration;
	}
	
}