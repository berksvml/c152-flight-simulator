#include "FlightDynamics/AerodynamicModel.h"

#include <cmath>

namespace AerodynamicModelConstants
{
	constexpr double MinimumPositiveValue =
		1.0e-12;

	constexpr double MinimumAirspeedMetersPerSecond =
		1.0e-6;
}

namespace C152::FlightDynamics
{
	bool FAerodynamicModelConfiguration::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(WingAreaSquareMeters)
			&& std::isfinite(ReferenceChordMeters)
			&& std::isfinite(LiftCoefficientAtZeroAlpha)
			&& std::isfinite(LiftCurveSlopePerRadian)
			&& std::isfinite(
				LiftCoefficientPerElevatorRadian)
			&& std::isfinite(ZeroLiftDragCoefficient)
			&& std::isfinite(InducedDragFactor)
			&& std::isfinite(
				PitchMomentCoefficientAtZeroAlpha)
			&& std::isfinite(PitchMomentSlopePerRadian)
			&& std::isfinite(PitchDampingDerivative)
			&& std::isfinite(
				PitchMomentPerElevatorRadian);

		if (!bValuesAreFinite)
		{
			return false;
		}

		return WingAreaSquareMeters
				> AerodynamicModelConstants::
			MinimumPositiveValue
			&& ReferenceChordMeters
				> AerodynamicModelConstants::
			MinimumPositiveValue
			&& ZeroLiftDragCoefficient >= 0.0
			&& InducedDragFactor >= 0.0;
	}

	bool FAerodynamicModel::SetConfiguration(
		const FAerodynamicModelConfiguration& NewConfiguration)
	{
		if (!NewConfiguration.IsValid())
		{
			return false;
		}

		Configuration = NewConfiguration;
		bIsConfigured = true;

		return true;
	}

	void FAerodynamicModel::ClearConfiguration()
	{
		Configuration = {};
		bIsConfigured = false;
	}

	bool FAerodynamicModel::IsConfigured() const
	{
		return bIsConfigured;
	}

	bool FAerodynamicModel::TryEvaluate(
		const FAirData& AirData,
		const FVector3& AngularRateBodyRadiansPerSecond,
		const FControlSurfaceState& ControlSurfaceState,
		FBodyForcesAndMoments& OutLoads,
		FAerodynamicCoefficients& OutCoefficients) const
	{
		const bool bInputsAreFinite =
			AirData.RelativeVelocityBodyMetersPerSecond.IsFinite()
			&& std::isfinite(
				AirData.TrueAirspeedMetersPerSecond)
			&& std::isfinite(
				AirData.AngleOfAttackRadians)
			&& std::isfinite(
				AirData.SideslipAngleRadians)
			&& std::isfinite(
				AirData.DynamicPressurePascals)
			&& std::isfinite(AirData.MachNumber)
			&& AngularRateBodyRadiansPerSecond.IsFinite()
			&& std::isfinite(ControlSurfaceState.ElevatorRad)
			&& std::isfinite(ControlSurfaceState.AileronRad)
			&& std::isfinite(ControlSurfaceState.RudderRad);

		if (!bIsConfigured
			|| !bInputsAreFinite
			|| AirData.TrueAirspeedMetersPerSecond < 0.0
			|| AirData.DynamicPressurePascals < 0.0)
		{
			return false;
		}

		double NormalizedPitchRate = 0.0;

		if (AirData.TrueAirspeedMetersPerSecond
			> AerodynamicModelConstants::
			MinimumAirspeedMetersPerSecond)
		{
			NormalizedPitchRate =
				AngularRateBodyRadiansPerSecond.Y
				* Configuration.ReferenceChordMeters
				/ (2.0
					* AirData.TrueAirspeedMetersPerSecond);
		}

		FAerodynamicCoefficients Coefficients{};

		Coefficients.LiftCoefficient =
			Configuration.LiftCoefficientAtZeroAlpha
			+ Configuration.LiftCurveSlopePerRadian
			* AirData.AngleOfAttackRadians
			+ Configuration.LiftCoefficientPerElevatorRadian
			* ControlSurfaceState.ElevatorRad;

		Coefficients.DragCoefficient =
			Configuration.ZeroLiftDragCoefficient
			+ Configuration.InducedDragFactor
			* Coefficients.LiftCoefficient
			* Coefficients.LiftCoefficient;

		Coefficients.PitchMomentCoefficient =
			Configuration.PitchMomentCoefficientAtZeroAlpha
			+ Configuration.PitchMomentSlopePerRadian
			* AirData.AngleOfAttackRadians
			+ Configuration.PitchDampingDerivative
			* NormalizedPitchRate
			+ Configuration.PitchMomentPerElevatorRadian
			* ControlSurfaceState.ElevatorRad;

		const bool bCoefficientsAreFinite =
			std::isfinite(Coefficients.LiftCoefficient)
			&& std::isfinite(Coefficients.DragCoefficient)
			&& std::isfinite(
				Coefficients.PitchMomentCoefficient);

		if (!bCoefficientsAreFinite)
		{
			return false;
		}

		const double DynamicPressureArea =
			AirData.DynamicPressurePascals
			* Configuration.WingAreaSquareMeters;

		const double LiftNewtons =
			DynamicPressureArea
			* Coefficients.LiftCoefficient;

		const double DragNewtons =
			DynamicPressureArea
			* Coefficients.DragCoefficient;

		const double CosineAlpha =
			std::cos(AirData.AngleOfAttackRadians);

		const double SineAlpha =
			std::sin(AirData.AngleOfAttackRadians);

		FBodyForcesAndMoments Loads{};

		// Body axes are FRD: X forward, Y right, Z down.
		Loads.ForceBodyNewtons.X =
			-DragNewtons * CosineAlpha
			+ LiftNewtons * SineAlpha;

		Loads.ForceBodyNewtons.Y =
			0.0;

		Loads.ForceBodyNewtons.Z =
			-DragNewtons * SineAlpha
			- LiftNewtons * CosineAlpha;

		Loads.MomentBodyNewtonMeters.X =
			0.0;

		Loads.MomentBodyNewtonMeters.Y =
			DynamicPressureArea
			* Configuration.ReferenceChordMeters
			* Coefficients.PitchMomentCoefficient;

		Loads.MomentBodyNewtonMeters.Z =
			0.0;

		if (!Loads.ForceBodyNewtons.IsFinite()
			|| !Loads.MomentBodyNewtonMeters.IsFinite())
		{
			return false;
		}

		OutLoads = Loads;
		OutCoefficients = Coefficients;

		return true;
	}
}