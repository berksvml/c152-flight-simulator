#include "FlightDynamics/C152AircraftConfiguration.h"

#include <cmath>

namespace
{
	constexpr double MinimumPositiveValue = 1.0e-12;

	constexpr double KilogramsPerPound =
		0.45359237;

	constexpr double MetersPerInch =
		0.0254;

	constexpr double WattsPerMechanicalHorsepower =
		745.6998715822702;

	constexpr double TwoPi =
		6.28318530717958647692;

	constexpr double SecondsPerMinute =
		60.0;

	constexpr double RevolutionsPerMinuteToRadiansPerSecond(
		const double RevolutionsPerMinute)
	{
		return RevolutionsPerMinute
			* TwoPi
			/ SecondsPerMinute;
	}
}

namespace C152::FlightDynamics
{
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

		if (MaximumTakeoffMassKilograms
			<= MinimumPositiveValue
			|| MaximumRampMassKilograms
			< MaximumTakeoffMassKilograms
			|| LightMassBreakPointKilograms
			<= MinimumPositiveValue
			|| LightMassBreakPointKilograms
			>= MaximumTakeoffMassKilograms)
		{
			return false;
		}

		if (ForwardLimitAtOrBelowLightMassMetersAftOfDatum
			<= MinimumPositiveValue
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
			|| MassKilograms <= MinimumPositiveValue
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

		return RatedPowerWatts > MinimumPositiveValue
			&& RatedEngineSpeedRadiansPerSecond
				> MinimumPositiveValue
			&& MinimumPropellerDiameterMeters
				> MinimumPositiveValue
			&& MaximumPropellerDiameterMeters
			>= MinimumPropellerDiameterMeters;
	}

	bool FC152AircraftConfiguration::IsValid() const
	{
		return MassAndBalance.IsValid()
			&& Propulsion.IsValid();
	}

	FC152AircraftConfiguration
		FC152AircraftConfiguration::Create1979Model152()
	{
		FC152AircraftConfiguration Configuration{};

		// FAA TCDS 3A19, Model 152.
		Configuration.MassAndBalance
			.MaximumTakeoffMassKilograms =
			1670.0 * KilogramsPerPound;

		Configuration.MassAndBalance
			.MaximumRampMassKilograms =
			1675.0 * KilogramsPerPound;

		Configuration.MassAndBalance
			.LightMassBreakPointKilograms =
			1350.0 * KilogramsPerPound;

		Configuration.MassAndBalance
			.ForwardLimitAtOrBelowLightMassMetersAftOfDatum =
			31.0 * MetersPerInch;

		Configuration.MassAndBalance
			.ForwardLimitAtMaximumMassMetersAftOfDatum =
			32.65 * MetersPerInch;

		Configuration.MassAndBalance
			.AftLimitMetersAftOfDatum =
			36.5 * MetersPerInch;

		// Lycoming O-235-L2C reference rating.
		Configuration.Propulsion.RatedPowerWatts =
			110.0 * WattsPerMechanicalHorsepower;

		Configuration.Propulsion
			.RatedEngineSpeedRadiansPerSecond =
			RevolutionsPerMinuteToRadiansPerSecond(
				2550.0);

		Configuration.Propulsion
			.MinimumPropellerDiameterMeters =
			67.5 * MetersPerInch;

		Configuration.Propulsion
			.MaximumPropellerDiameterMeters =
			69.0 * MetersPerInch;

		return Configuration;
	}
}