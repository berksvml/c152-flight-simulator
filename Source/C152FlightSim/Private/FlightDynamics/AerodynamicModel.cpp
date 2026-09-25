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
			&& std::isfinite(ReferenceSpanMeters)
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
				PitchMomentPerElevatorRadian)
			&& std::isfinite(
				SideForceSlopePerSideslipRadian)
			&& std::isfinite(
				SideForceRollRateDerivative)
			&& std::isfinite(
				SideForceYawRateDerivative)
			&& std::isfinite(
				SideForcePerAileronRadian)
			&& std::isfinite(
				SideForcePerRudderRadian)
			&& std::isfinite(
				RollMomentSlopePerSideslipRadian)
			&& std::isfinite(
				RollDampingDerivative)
			&& std::isfinite(
				RollYawRateDerivative)
			&& std::isfinite(
				RollMomentPerAileronRadian)
			&& std::isfinite(
				RollMomentPerRudderRadian)
			&& std::isfinite(
				YawMomentSlopePerSideslipRadian)
			&& std::isfinite(
				YawRollRateDerivative)
			&& std::isfinite(
				YawDampingDerivative)
			&& std::isfinite(
				YawMomentPerAileronRadian)
			&& std::isfinite(
				YawMomentPerRudderRadian);

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
			&& ReferenceSpanMeters
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

		double NormalizedRollRate = 0.0;
		double NormalizedPitchRate = 0.0;
		double NormalizedYawRate = 0.0;

		if (AirData.TrueAirspeedMetersPerSecond
			> AerodynamicModelConstants::
			MinimumAirspeedMetersPerSecond)
		{
			const double TwiceAirspeed =
				2.0
				* AirData.TrueAirspeedMetersPerSecond;

			NormalizedRollRate =
				AngularRateBodyRadiansPerSecond.X
				* Configuration.ReferenceSpanMeters
				/ TwiceAirspeed;

			NormalizedPitchRate =
				AngularRateBodyRadiansPerSecond.Y
				* Configuration.ReferenceChordMeters
				/ TwiceAirspeed;

			NormalizedYawRate =
				AngularRateBodyRadiansPerSecond.Z
				* Configuration.ReferenceSpanMeters
				/ TwiceAirspeed;
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

		Coefficients.SideForceCoefficient =
			Configuration.SideForceSlopePerSideslipRadian
			* AirData.SideslipAngleRadians
			+ Configuration.SideForceRollRateDerivative
			* NormalizedRollRate
			+ Configuration.SideForceYawRateDerivative
			* NormalizedYawRate
			+ Configuration.SideForcePerAileronRadian
			* ControlSurfaceState.AileronRad
			+ Configuration.SideForcePerRudderRadian
			* ControlSurfaceState.RudderRad;

		Coefficients.RollMomentCoefficient =
			Configuration.RollMomentSlopePerSideslipRadian
			* AirData.SideslipAngleRadians
			+ Configuration.RollDampingDerivative
			* NormalizedRollRate
			+ Configuration.RollYawRateDerivative
			* NormalizedYawRate
			+ Configuration.RollMomentPerAileronRadian
			* ControlSurfaceState.AileronRad
			+ Configuration.RollMomentPerRudderRadian
			* ControlSurfaceState.RudderRad;

		Coefficients.YawMomentCoefficient =
			Configuration.YawMomentSlopePerSideslipRadian
			* AirData.SideslipAngleRadians
			+ Configuration.YawRollRateDerivative
			* NormalizedRollRate
			+ Configuration.YawDampingDerivative
			* NormalizedYawRate
			+ Configuration.YawMomentPerAileronRadian
			* ControlSurfaceState.AileronRad
			+ Configuration.YawMomentPerRudderRadian
			* ControlSurfaceState.RudderRad;

		const bool bCoefficientsAreFinite =
			std::isfinite(Coefficients.LiftCoefficient)
			&& std::isfinite(Coefficients.DragCoefficient)
			&& std::isfinite(
				Coefficients.SideForceCoefficient)
			&& std::isfinite(
				Coefficients.RollMomentCoefficient)
			&& std::isfinite(
				Coefficients.PitchMomentCoefficient)
			&& std::isfinite(
				Coefficients.YawMomentCoefficient);

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

		// CY is currently treated as a body-axis side-force
		// coefficient. Positive force acts toward body-right.
		Loads.ForceBodyNewtons.Y =
			DynamicPressureArea
			* Coefficients.SideForceCoefficient;

		Loads.ForceBodyNewtons.Z =
			-DragNewtons * SineAlpha
			- LiftNewtons * CosineAlpha;

		// Positive moments follow the right-hand rule in FRD:
		// +X right roll, +Y nose up, +Z nose right.
		Loads.MomentBodyNewtonMeters.X =
			DynamicPressureArea
			* Configuration.ReferenceSpanMeters
			* Coefficients.RollMomentCoefficient;

		Loads.MomentBodyNewtonMeters.Y =
			DynamicPressureArea
			* Configuration.ReferenceChordMeters
			* Coefficients.PitchMomentCoefficient;

		Loads.MomentBodyNewtonMeters.Z =
			DynamicPressureArea
			* Configuration.ReferenceSpanMeters
			* Coefficients.YawMomentCoefficient;

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