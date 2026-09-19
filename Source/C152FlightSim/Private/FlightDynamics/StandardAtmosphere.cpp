#include "FlightDynamics/StandardAtmosphere.h"

#include <cmath>

namespace C152::FlightDynamics
{
    bool FStandardAtmosphere::TryEvaluate(
        const double GeopotentialAltitudeMeters,
        FAtmosphereState& OutAtmosphere)
    {
        if (!std::isfinite(GeopotentialAltitudeMeters)
            || GeopotentialAltitudeMeters < 0.0
            || GeopotentialAltitudeMeters > MaxGeopotentialAltitudeMeters)
        {
            return false;
        }

        // U.S. Standard Atmosphere, 1976, hydrostatic troposphere.
        // https://ntrs.nasa.gov/citations/19770009539
        constexpr double SeaLevelTemperatureKelvin = 288.15;
        constexpr double SeaLevelPressurePascals = 101325.0;
        constexpr double TemperatureLapseRateKelvinPerMeter = 0.0065;
        constexpr double StandardGravityMetersPerSecondSquared = 9.80665;
        constexpr double SpecificGasConstantJoulesPerKilogramKelvin =
            287.05287;
        constexpr double RatioOfSpecificHeats = 1.4;

        const double TemperatureKelvin =
            SeaLevelTemperatureKelvin
            - TemperatureLapseRateKelvinPerMeter
                * GeopotentialAltitudeMeters;

        const double PressurePascals =
            SeaLevelPressurePascals
            * std::pow(
                TemperatureKelvin / SeaLevelTemperatureKelvin,
                StandardGravityMetersPerSecondSquared
                    / (TemperatureLapseRateKelvinPerMeter
                        * SpecificGasConstantJoulesPerKilogramKelvin));

        FAtmosphereState Result{};
        Result.TemperatureKelvin = TemperatureKelvin;
        Result.PressurePascals = PressurePascals;
        Result.DensityKilogramsPerCubicMeter =
            PressurePascals
            / (SpecificGasConstantJoulesPerKilogramKelvin
                * TemperatureKelvin);
        Result.SpeedOfSoundMetersPerSecond =
            std::sqrt(
                RatioOfSpecificHeats
                * SpecificGasConstantJoulesPerKilogramKelvin
                * TemperatureKelvin);

        OutAtmosphere = Result;
        return true;
    }
}
