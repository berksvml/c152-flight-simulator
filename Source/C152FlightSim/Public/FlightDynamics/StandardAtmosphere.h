#pragma once

namespace C152::FlightDynamics
{
    // Dry-air properties in SI units.
    struct FAtmosphereState
    {
        double TemperatureKelvin = 0.0;
        double PressurePascals = 0.0;
        double DensityKilogramsPerCubicMeter = 0.0;
        double SpeedOfSoundMetersPerSecond = 0.0;
    };

    // U.S. Standard Atmosphere, 1976: the 0-11 km geopotential troposphere.
    // This model has no wind or nonstandard weather inputs.
    class FStandardAtmosphere final
    {
    public:
        static constexpr double MaxGeopotentialAltitudeMeters = 11000.0;

        // Altitude is geopotential height above mean sea level, not the
        // aircraft's NED displacement from the simulation origin.
        // Returns false for nonfinite/out-of-range input without changing output.
        [[nodiscard]]
        static bool TryEvaluate(
            double GeopotentialAltitudeMeters,
            FAtmosphereState& OutAtmosphere);
    };
}
