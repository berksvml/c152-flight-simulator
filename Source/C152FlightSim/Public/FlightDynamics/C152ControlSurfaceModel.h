#pragma once

namespace C152::FlightDynamics
{
	constexpr double DegreesToRadians(
		const double Degrees)
	{
		constexpr double RadiansPerDegree =
			0.01745329251994329577;

		return Degrees * RadiansPerDegree;
	}

	struct FControlCommand
	{
		// Normalized commands: [-1, 1]
		double Pitch = 0.0;
		double Roll = 0.0;
		double Yaw = 0.0;

		// Throttle position: [0, 1]
		double Throttle = 0.0;
	};

	struct FControlSurfaceState
	{
		// Generalized control coordinates in radians.
		// Positive Elevator: nose-up command.
		// Positive Aileron: right-roll command.
		// Positive Rudder: nose-right command.
		double ElevatorRad = 0.0;
		double AileronRad = 0.0;
		double RudderRad = 0.0;

		double Throttle = 0.0;
	};

	struct FControlSurfaceLimits
	{
		// Initial development values.
		// These must be validated before claiming C152 fidelity.
		double ElevatorNoseUpRad = DegreesToRadians(25.0);
		double ElevatorNoseDownRad = DegreesToRadians(18.0);
		double AileronRad = DegreesToRadians(20.0);
		double RudderRad = DegreesToRadians(23.0);

		// Numerical input shaping for keyboard commands.
		// This is not yet a physical actuator model.
		double SurfaceRateRadPerSecond = DegreesToRadians(60.0);
	};

	class FC152ControlSurfaceModel
	{
	public:
		void Update(
			const FControlCommand& Command,
			double DeltaSeconds);

		void Reset();

		const FControlSurfaceState& GetState() const;

		void SetLimits(const FControlSurfaceLimits& NewLimits);

		const FControlSurfaceLimits& GetLimits() const;

	private:
		static double MoveTowards(
			double Current,
			double Target,
			double MaxDelta);

		FControlSurfaceLimits Limits;
		FControlSurfaceState State;
	};
}