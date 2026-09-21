#include "FlightDynamics/WindModel.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr std::uint32_t ZeroSeedReplacement =
        0x6D2B79F5U;

    constexpr double TwoPi =
        6.28318530717958647692;

    constexpr double UnsignedIntegerRangePlusOne =
        4294967297.0;
}

namespace C152::FlightDynamics
{
    bool FWindModelConfiguration::IsValid() const
    {
        return SteadyWindVelocityNedMetersPerSecond.IsFinite()
            && TurbulenceStandardDeviationNedMetersPerSecond.IsFinite()
            && TurbulenceStandardDeviationNedMetersPerSecond.X >= 0.0
            && TurbulenceStandardDeviationNedMetersPerSecond.Y >= 0.0
            && TurbulenceStandardDeviationNedMetersPerSecond.Z >= 0.0
            && std::isfinite(TurbulenceCorrelationTimeSeconds)
            && TurbulenceCorrelationTimeSeconds > 0.0;
    }

    bool FWindModel::Configure(
        const FWindModelConfiguration& NewConfiguration)
    {
        if (!NewConfiguration.IsValid())
        {
            return false;
        }

        Configuration = NewConfiguration;
        bConfigured = true;

        Reset();

        return true;
    }

    void FWindModel::Clear()
    {
        Configuration = FWindModelConfiguration{};
        TurbulenceVelocityNedMetersPerSecond = FVector3{};
        WindVelocityNedMetersPerSecond = FVector3{};
        RandomState = 1U;
        bConfigured = false;
    }

    void FWindModel::Reset()
    {
        TurbulenceVelocityNedMetersPerSecond = FVector3{};

        WindVelocityNedMetersPerSecond =
            Configuration.SteadyWindVelocityNedMetersPerSecond;

        RandomState =
            Configuration.RandomSeed != 0U
            ? Configuration.RandomSeed
            : ZeroSeedReplacement;
    }

    bool FWindModel::IsConfigured() const
    {
        return bConfigured;
    }

    bool FWindModel::Update(
        const double DeltaTimeSeconds)
    {
        if (!bConfigured
            || !std::isfinite(DeltaTimeSeconds)
            || DeltaTimeSeconds <= 0.0)
        {
            return false;
        }

        const double Decay =
            std::exp(
                -DeltaTimeSeconds
                / Configuration.TurbulenceCorrelationTimeSeconds);

        const double InnovationScale =
            std::sqrt(std::max(
                0.0,
                1.0 - Decay * Decay));

        const std::uint32_t PreviousRandomState =
            RandomState;

        const FVector3 CandidateTurbulence{
            Decay * TurbulenceVelocityNedMetersPerSecond.X
                + InnovationScale
                * Configuration
                    .TurbulenceStandardDeviationNedMetersPerSecond.X
                * NextStandardNormal(),

            Decay * TurbulenceVelocityNedMetersPerSecond.Y
                + InnovationScale
                * Configuration
                    .TurbulenceStandardDeviationNedMetersPerSecond.Y
                * NextStandardNormal(),

            Decay * TurbulenceVelocityNedMetersPerSecond.Z
                + InnovationScale
                * Configuration
                    .TurbulenceStandardDeviationNedMetersPerSecond.Z
                * NextStandardNormal()
        };

        const FVector3 CandidateWind =
            Configuration.SteadyWindVelocityNedMetersPerSecond
            + CandidateTurbulence;

        if (!CandidateTurbulence.IsFinite()
            || !CandidateWind.IsFinite())
        {
            RandomState = PreviousRandomState;
            return false;
        }

        TurbulenceVelocityNedMetersPerSecond =
            CandidateTurbulence;

        WindVelocityNedMetersPerSecond =
            CandidateWind;

        return true;
    }

    const FVector3&
        FWindModel::GetWindVelocityNedMetersPerSecond() const
    {
        return WindVelocityNedMetersPerSecond;
    }

    const FVector3&
        FWindModel::
        GetTurbulenceVelocityNedMetersPerSecond() const
    {
        return TurbulenceVelocityNedMetersPerSecond;
    }

    std::uint32_t FWindModel::NextRandomUnsignedInteger()
    {
        RandomState ^= RandomState << 13U;
        RandomState ^= RandomState >> 17U;
        RandomState ^= RandomState << 5U;

        return RandomState;
    }

    double FWindModel::NextUniformOpenInterval()
    {
        return (
            static_cast<double>(
                NextRandomUnsignedInteger())
            + 1.0)
            / UnsignedIntegerRangePlusOne;
    }

    double FWindModel::NextStandardNormal()
    {
        const double FirstUniform =
            NextUniformOpenInterval();

        const double SecondUniform =
            NextUniformOpenInterval();

        return std::sqrt(
            -2.0 * std::log(FirstUniform))
            * std::cos(TwoPi * SecondUniform);
    }
}