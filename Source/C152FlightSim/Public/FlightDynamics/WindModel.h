#pragma once

#include "FlightDynamics/FlightDynamicsTypes.h"

#include <cstdint>

namespace C152::FlightDynamics
{
    struct FWindModelConfiguration
    {
        // Mean air-mass velocity in Navigation/NED axes [m/s].
        FVector3 SteadyWindVelocityNedMetersPerSecond{};

        // Standard deviation of zero-mean turbulence in NED axes [m/s].
        FVector3 TurbulenceStandardDeviationNedMetersPerSecond{};

        // Shared first-order turbulence correlation time [s].
        double TurbulenceCorrelationTimeSeconds = 1.0;

        // Equal seeds produce equal turbulence sequences.
        std::uint32_t RandomSeed = 1U;

        [[nodiscard]]
        bool IsValid() const;
    };

    class FWindModel final
    {
    public:
        [[nodiscard]]
        bool Configure(
            const FWindModelConfiguration& NewConfiguration);

        void Clear();
        void Reset();

        [[nodiscard]]
        bool IsConfigured() const;

        [[nodiscard]]
        bool Update(double DeltaTimeSeconds);

        [[nodiscard]]
        const FVector3&
            GetWindVelocityNedMetersPerSecond() const;

        [[nodiscard]]
        const FVector3&
            GetTurbulenceVelocityNedMetersPerSecond() const;

    private:
        [[nodiscard]]
        std::uint32_t NextRandomUnsignedInteger();

        [[nodiscard]]
        double NextUniformOpenInterval();

        [[nodiscard]]
        double NextStandardNormal();

        FWindModelConfiguration Configuration{};

        FVector3 TurbulenceVelocityNedMetersPerSecond{};
        FVector3 WindVelocityNedMetersPerSecond{};

        std::uint32_t RandomState = 1U;
        bool bConfigured = false;
    };
}