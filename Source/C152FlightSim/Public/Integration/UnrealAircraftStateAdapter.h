#pragma once

#include "FlightDynamics/FlightDynamicsTypes.h"
#include "Math/Transform.h"

namespace C152::UnrealIntegration
{
	class FUnrealAircraftStateAdapter final
	{
	public:
		[[nodiscard]]
		static FTransform ToUnrealTransform(
			const FlightDynamics::FAircraftState& AircraftState);

		static void UpdateCorePoseFromUnrealTransform(
			const FTransform& UnrealTransform,
			FlightDynamics::FAircraftState& InOutAircraftState);
	};
}