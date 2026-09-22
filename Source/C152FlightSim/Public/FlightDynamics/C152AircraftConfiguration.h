#pragma once

namespace C152::FlightDynamics
{
	struct FC152MassAndBalanceLimits
	{
		// Certification limits, not the current simulated aircraft mass.
		double MaximumTakeoffMassKilograms = 0.0;
		double MaximumRampMassKilograms = 0.0;

		// Published C.G. stations are measured aft of the firewall datum.
		double LightMassBreakPointKilograms = 0.0;

		double ForwardLimitAtOrBelowLightMassMetersAftOfDatum =
			0.0;

		double ForwardLimitAtMaximumMassMetersAftOfDatum =
			0.0;

		double AftLimitMetersAftOfDatum = 0.0;

		[[nodiscard]]
		bool IsValid() const;

		// On failure, neither output value is modified.
		[[nodiscard]]
		bool TryGetCenterOfGravityLimits(
			double MassKilograms,
			double& OutForwardLimitMetersAftOfDatum,
			double& OutAftLimitMetersAftOfDatum) const;

		[[nodiscard]]
		bool IsWithinCenterOfGravityEnvelope(
			double MassKilograms,
			double CenterOfGravityMetersAftOfDatum) const;
	};

	struct FC152GeometryReference
	{
		double ReferenceWingAreaSquareMeters{ 0.0 };
		double ReferenceWingSpanMeters{ 0.0 };

		[[nodiscard]]
		bool IsValid() const;

		[[nodiscard]]
		double GetEquivalentRectangularChordMeters() const;

		[[nodiscard]]
		double GetAspectRatio() const;
	};

	struct FC152PropulsionReference
	{
		// Reference engine rating. This is not yet a propulsion model.
		double RatedPowerWatts = 0.0;
		double RatedEngineSpeedRadiansPerSecond = 0.0;

		double MinimumPropellerDiameterMeters = 0.0;
		double MaximumPropellerDiameterMeters = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FC152AircraftConfiguration
	{
		FC152MassAndBalanceLimits MassAndBalance{};
		FC152GeometryReference Geometry{};
		FC152PropulsionReference Propulsion{};

		[[nodiscard]]
		bool IsValid() const;

		[[nodiscard]]
		static FC152AircraftConfiguration Create1979Model152();
	};
}