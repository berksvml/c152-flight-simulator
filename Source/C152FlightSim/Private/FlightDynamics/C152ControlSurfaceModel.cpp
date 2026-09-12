#include "FlightDynamics/C152ControlSurfaceModel.h"

#include <algorithm>

namespace C152::FlightDynamics
{
	void FC152ControlSurfaceModel::Update(
		const FControlCommand& Command,
		const double DeltaSeconds)
	{
		if (DeltaSeconds <= 0.0)
		{
			return;
		}

		const double PitchCommand =
			std::clamp(Command.Pitch, -1.0, 1.0);

		const double RollCommand =
			std::clamp(Command.Roll, -1.0, 1.0);

		const double YawCommand =
			std::clamp(Command.Yaw, -1.0, 1.0);

		const double ElevatorTargetRad =
			PitchCommand >= 0.0
			? PitchCommand * Limits.ElevatorNoseUpRad
			: PitchCommand * Limits.ElevatorNoseDownRad;

		const double AileronTargetRad =
			RollCommand * Limits.AileronRad;

		const double RudderTargetRad =
			YawCommand * Limits.RudderRad;

		const double MaximumSurfaceStep =
			std::max(0.0, Limits.SurfaceRateRadPerSecond)
			* DeltaSeconds;

		State.ElevatorRad = MoveTowards(
			State.ElevatorRad,
			ElevatorTargetRad,
			MaximumSurfaceStep);

		State.AileronRad = MoveTowards(
			State.AileronRad,
			AileronTargetRad,
			MaximumSurfaceStep);

		State.RudderRad = MoveTowards(
			State.RudderRad,
			RudderTargetRad,
			MaximumSurfaceStep);

		State.Throttle =
			std::clamp(Command.Throttle, 0.0, 1.0);
	}

	void FC152ControlSurfaceModel::Reset()
	{
		State = FControlSurfaceState{};
	}

	const FControlSurfaceState&
		FC152ControlSurfaceModel::GetState() const
	{
		return State;
	}

	void FC152ControlSurfaceModel::SetLimits(
		const FControlSurfaceLimits& NewLimits)
	{
		Limits = NewLimits;
	}

	const FControlSurfaceLimits&
		FC152ControlSurfaceModel::GetLimits() const
	{
		return Limits;
	}

	double FC152ControlSurfaceModel::MoveTowards(
		const double Current,
		const double Target,
		const double MaxDelta)
	{
		const double SafeMaxDelta = std::max(0.0, MaxDelta);
		const double Difference = Target - Current;

		return Current + std::clamp(
			Difference,
			-SafeMaxDelta,
			SafeMaxDelta);
	}
}