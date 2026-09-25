#pragma once

#include "FlightDynamics/AirDataModel.h"
#include "FlightDynamics/C152ControlSurfaceModel.h"
#include "FlightDynamics/FlightDynamicsTypes.h"

namespace C152::FlightDynamics
{
	struct FAerodynamicModelConfiguration
	{
		double WingAreaSquareMeters = 0.0;
		double ReferenceChordMeters = 0.0;
		double ReferenceSpanMeters = 0.0;

		// Longitudinal derivatives.
		double LiftCoefficientAtZeroAlpha = 0.0;
		double LiftCurveSlopePerRadian = 0.0;
		double LiftCoefficientPerElevatorRadian = 0.0;

		double ZeroLiftDragCoefficient = 0.0;
		double InducedDragFactor = 0.0;

		double PitchMomentCoefficientAtZeroAlpha = 0.0;
		double PitchMomentSlopePerRadian = 0.0;
		double PitchDampingDerivative = 0.0;
		double PitchMomentPerElevatorRadian = 0.0;

		// Lateral-directional derivatives.
		double SideForceSlopePerSideslipRadian = 0.0;
		double SideForceRollRateDerivative = 0.0;
		double SideForceYawRateDerivative = 0.0;
		double SideForcePerAileronRadian = 0.0;
		double SideForcePerRudderRadian = 0.0;

		double RollMomentSlopePerSideslipRadian = 0.0;
		double RollDampingDerivative = 0.0;
		double RollYawRateDerivative = 0.0;
		double RollMomentPerAileronRadian = 0.0;
		double RollMomentPerRudderRadian = 0.0;

		double YawMomentSlopePerSideslipRadian = 0.0;
		double YawRollRateDerivative = 0.0;
		double YawDampingDerivative = 0.0;
		double YawMomentPerAileronRadian = 0.0;
		double YawMomentPerRudderRadian = 0.0;

		[[nodiscard]]
		bool IsValid() const;
	};

	struct FAerodynamicCoefficients
	{
		double LiftCoefficient = 0.0;
		double DragCoefficient = 0.0;
		double SideForceCoefficient = 0.0;

		double RollMomentCoefficient = 0.0;
		double PitchMomentCoefficient = 0.0;
		double YawMomentCoefficient = 0.0;
	};

	class FAerodynamicModel final
	{
	public:
		[[nodiscard]]
		bool SetConfiguration(
			const FAerodynamicModelConfiguration& NewConfiguration);

		void ClearConfiguration();

		[[nodiscard]]
		bool IsConfigured() const;

		// Returns false without changing either output.
		[[nodiscard]]
		bool TryEvaluate(
			const FAirData& AirData,
			const FVector3& AngularRateBodyRadiansPerSecond,
			const FControlSurfaceState& ControlSurfaceState,
			FBodyForcesAndMoments& OutLoads,
			FAerodynamicCoefficients& OutCoefficients) const;

	private:
		FAerodynamicModelConfiguration Configuration{};
		bool bIsConfigured = false;
	};
}