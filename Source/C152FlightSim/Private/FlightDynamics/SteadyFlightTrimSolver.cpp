#include "FlightDynamics/SteadyFlightTrimSolver.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace SteadyFlightTrimSolverInternal
{
	constexpr double TrimSolverPi =
		3.14159265358979323846;

	constexpr double MinimumPositiveValue =
		1.0e-12;

	constexpr double LinearSolvePivotTolerance =
		1.0e-12;

	struct FTrimVariables
	{
		double AngleOfAttackRadians = 0.0;
		double ElevatorRadians = 0.0;
		double Throttle = 0.0;
	};

	struct FTrimEvaluation
	{
		C152::FlightDynamics::FSteadyFlightTrimResult Result{};
		C152::FlightDynamics::FVector3 NormalizedResidual{};
	};

	bool IsAtmosphereValid(
		const C152::FlightDynamics::FAtmosphereState& Atmosphere)
	{
		return std::isfinite(Atmosphere.TemperatureKelvin)
			&& std::isfinite(Atmosphere.PressurePascals)
			&& std::isfinite(
				Atmosphere.DensityKilogramsPerCubicMeter)
			&& std::isfinite(
				Atmosphere.SpeedOfSoundMetersPerSecond)
			&& Atmosphere.TemperatureKelvin
				> MinimumPositiveValue
			&& Atmosphere.PressurePascals
				> MinimumPositiveValue
			&& Atmosphere.DensityKilogramsPerCubicMeter
				> MinimumPositiveValue
			&& Atmosphere.SpeedOfSoundMetersPerSecond
				> MinimumPositiveValue;
	}

	double CalculateResidualNorm(
		const C152::FlightDynamics::FVector3& Residual)
	{
		return Residual.Norm();
	}

	double GetVariable(
		const FTrimVariables& Variables,
		const int Index)
	{
		if (Index == 0)
		{
			return Variables.AngleOfAttackRadians;
		}

		if (Index == 1)
		{
			return Variables.ElevatorRadians;
		}

		return Variables.Throttle;
	}

	void SetVariable(
		FTrimVariables& Variables,
		const int Index,
		const double Value)
	{
		if (Index == 0)
		{
			Variables.AngleOfAttackRadians = Value;
			return;
		}

		if (Index == 1)
		{
			Variables.ElevatorRadians = Value;
			return;
		}

		Variables.Throttle = Value;
	}

	double GetResidualComponent(
		const C152::FlightDynamics::FVector3& Residual,
		const int Index)
	{
		if (Index == 0)
		{
			return Residual.X;
		}

		if (Index == 1)
		{
			return Residual.Y;
		}

		return Residual.Z;
	}

	double GetMinimumVariable(
		const C152::FlightDynamics::FSteadyFlightTrimRequest& Request,
		const int Index)
	{
		if (Index == 0)
		{
			return Request.MinimumAngleOfAttackRadians;
		}

		if (Index == 1)
		{
			return Request.MinimumElevatorRadians;
		}

		return 0.0;
	}

	double GetMaximumVariable(
		const C152::FlightDynamics::FSteadyFlightTrimRequest& Request,
		const int Index)
	{
		if (Index == 0)
		{
			return Request.MaximumAngleOfAttackRadians;
		}

		if (Index == 1)
		{
			return Request.MaximumElevatorRadians;
		}

		return 1.0;
	}

	FTrimVariables ClampVariables(
		const FTrimVariables& Variables,
		const C152::FlightDynamics::FSteadyFlightTrimRequest& Request)
	{
		return FTrimVariables{
			std::clamp(
				Variables.AngleOfAttackRadians,
				Request.MinimumAngleOfAttackRadians,
				Request.MaximumAngleOfAttackRadians),
			std::clamp(
				Variables.ElevatorRadians,
				Request.MinimumElevatorRadians,
				Request.MaximumElevatorRadians),
			std::clamp(Variables.Throttle, 0.0, 1.0)
		};
	}

	bool TryEvaluate(
		const C152::FlightDynamics::FAerodynamicModel& AerodynamicModel,
		const C152::FlightDynamics::FPropulsionModel& PropulsionModel,
		const C152::FlightDynamics::FAircraftMassProperties&
		MassProperties,
		const C152::FlightDynamics::FAtmosphereState& Atmosphere,
		const C152::FlightDynamics::FSteadyFlightTrimRequest& Request,
		const FTrimVariables& Variables,
		FTrimEvaluation& OutEvaluation)
	{
		using namespace C152::FlightDynamics;

		FSteadyFlightTrimResult Result{};

		Result.AngleOfAttackRadians =
			Variables.AngleOfAttackRadians;

		Result.PitchAttitudeRadians =
			Variables.AngleOfAttackRadians
			+ Request.FlightPathAngleRadians;

		Result.ElevatorRadians =
			Variables.ElevatorRadians;

		Result.Throttle =
			Variables.Throttle;

		const double CosineAlpha =
			std::cos(Variables.AngleOfAttackRadians);

		const double SineAlpha =
			std::sin(Variables.AngleOfAttackRadians);

		Result.AircraftState.VelocityBodyMetersPerSecond =
			FVector3{
				Request.TrueAirspeedMetersPerSecond
					* CosineAlpha,
				0.0,
				Request.TrueAirspeedMetersPerSecond
					* SineAlpha
			};

		const double HalfPitchAttitude =
			0.5 * Result.PitchAttitudeRadians;

		Result.AircraftState.AttitudeBodyToNed =
			FQuaternion{
				std::cos(HalfPitchAttitude),
				0.0,
				std::sin(HalfPitchAttitude),
				0.0
			};

		if (!FAirDataModel::TryEvaluate(
			Result.AircraftState,
			Atmosphere,
			FVector3{},
			Result.AirData))
		{
			return false;
		}

		Result.ControlSurfaceState.ElevatorRad =
			Variables.ElevatorRadians;

		Result.ControlSurfaceState.Throttle =
			Variables.Throttle;

		if (!AerodynamicModel.TryEvaluate(
			Result.AirData,
			FVector3{},
			Result.ControlSurfaceState,
			Result.AerodynamicLoads,
			Result.AerodynamicCoefficients))
		{
			return false;
		}

		FPropulsionOutput PropulsionOutput{};

		if (!PropulsionModel.TryEvaluate(
			Variables.Throttle,
			Atmosphere.DensityKilogramsPerCubicMeter,
			std::max(
				0.0,
				Result.AirData
				.RelativeVelocityBodyMetersPerSecond.X),
			Result.PropulsionLoads,
			PropulsionOutput))
		{
			return false;
		}

		Result.Propulsion = PropulsionOutput;

		const double WeightNewtons =
			MassProperties.MassKilograms
			* Request.GravityAccelerationMetersPerSecondSquared;

		const FVector3 GravityForceNedNewtons{
			0.0,
			0.0,
			WeightNewtons
		};

		const FVector3 GravityForceBodyNewtons =
			Result.AircraftState.AttitudeBodyToNed
			.InverseRotateVector(GravityForceNedNewtons);

		Result.ResidualForceBodyNewtons =
			Result.AerodynamicLoads.ForceBodyNewtons
			+ Result.PropulsionLoads.ForceBodyNewtons
			+ GravityForceBodyNewtons;

		Result.ResidualPitchMomentNewtonMeters =
			Result.AerodynamicLoads
			.MomentBodyNewtonMeters.Y
			+ Result.PropulsionLoads
			.MomentBodyNewtonMeters.Y;
		OutEvaluation.Result = Result;

		return true;
	}

	bool TrySolveLinearSystem(
		const double Matrix[3][3],
		const C152::FlightDynamics::FVector3& RightHandSide,
		C152::FlightDynamics::FVector3& OutSolution)
	{
		double Augmented[3][4]{
			{
				Matrix[0][0],
				Matrix[0][1],
				Matrix[0][2],
				RightHandSide.X
			},
			{
				Matrix[1][0],
				Matrix[1][1],
				Matrix[1][2],
				RightHandSide.Y
			},
			{
				Matrix[2][0],
				Matrix[2][1],
				Matrix[2][2],
				RightHandSide.Z
			}
		};

		for (int PivotColumn = 0;
			PivotColumn < 3;
			++PivotColumn)
		{
			int PivotRow = PivotColumn;
			double LargestPivotMagnitude =
				std::abs(Augmented[PivotRow][PivotColumn]);

			for (int CandidateRow = PivotColumn + 1;
				CandidateRow < 3;
				++CandidateRow)
			{
				const double CandidateMagnitude =
					std::abs(
						Augmented[CandidateRow][PivotColumn]);

				if (CandidateMagnitude > LargestPivotMagnitude)
				{
					PivotRow = CandidateRow;
					LargestPivotMagnitude = CandidateMagnitude;
				}
			}

			if (!std::isfinite(LargestPivotMagnitude)
				|| LargestPivotMagnitude
				<= LinearSolvePivotTolerance)
			{
				return false;
			}

			if (PivotRow != PivotColumn)
			{
				for (int Column = PivotColumn;
					Column < 4;
					++Column)
				{
					std::swap(
						Augmented[PivotColumn][Column],
						Augmented[PivotRow][Column]);
				}
			}

			const double Pivot =
				Augmented[PivotColumn][PivotColumn];

			for (int Column = PivotColumn;
				Column < 4;
				++Column)
			{
				Augmented[PivotColumn][Column] /= Pivot;
			}

			for (int Row = 0; Row < 3; ++Row)
			{
				if (Row == PivotColumn)
				{
					continue;
				}

				const double EliminationFactor =
					Augmented[Row][PivotColumn];

				for (int Column = PivotColumn;
					Column < 4;
					++Column)
				{
					Augmented[Row][Column] -=
						EliminationFactor
						* Augmented[PivotColumn][Column];
				}
			}
		}

		const C152::FlightDynamics::FVector3 Solution{
			Augmented[0][3],
			Augmented[1][3],
			Augmented[2][3]
		};

		if (!Solution.IsFinite())
		{
			return false;
		}

		OutSolution = Solution;
		return true;
	}
}

namespace C152::FlightDynamics
{
	bool FSteadyFlightTrimRequest::IsValid() const
	{
		const bool bValuesAreFinite =
			std::isfinite(TrueAirspeedMetersPerSecond)
			&& std::isfinite(FlightPathAngleRadians)
			&& std::isfinite(
				GravityAccelerationMetersPerSecondSquared)
			&& std::isfinite(MinimumAngleOfAttackRadians)
			&& std::isfinite(MaximumAngleOfAttackRadians)
			&& std::isfinite(MinimumElevatorRadians)
			&& std::isfinite(MaximumElevatorRadians)
			&& std::isfinite(InitialAngleOfAttackRadians)
			&& std::isfinite(InitialElevatorRadians)
			&& std::isfinite(InitialThrottle);

		if (!bValuesAreFinite)
		{
			return false;
		}

		const double HalfPi =
			0.5 * SteadyFlightTrimSolverInternal::TrimSolverPi;

		return TrueAirspeedMetersPerSecond
			>= FAirDataModel::
			MinimumFlowAngleAirspeedMetersPerSecond
			&& std::abs(FlightPathAngleRadians) < HalfPi
			&& GravityAccelerationMetersPerSecondSquared
			> SteadyFlightTrimSolverInternal::MinimumPositiveValue
			&& MinimumAngleOfAttackRadians
			< MaximumAngleOfAttackRadians
			&& MinimumAngleOfAttackRadians > -HalfPi
			&& MaximumAngleOfAttackRadians < HalfPi
			&& MinimumElevatorRadians
			< MaximumElevatorRadians
			&& InitialAngleOfAttackRadians
			>= MinimumAngleOfAttackRadians
			&& InitialAngleOfAttackRadians
			<= MaximumAngleOfAttackRadians
			&& InitialElevatorRadians
			>= MinimumElevatorRadians
			&& InitialElevatorRadians
			<= MaximumElevatorRadians
			&& InitialThrottle >= 0.0
			&& InitialThrottle <= 1.0;
	}

	bool FSteadyFlightTrimSolverSettings::IsValid() const
	{
		return MaximumIterations > 0
			&& std::isfinite(NormalizedResidualTolerance)
			&& NormalizedResidualTolerance
			> SteadyFlightTrimSolverInternal::MinimumPositiveValue
			&& std::isfinite(
				FiniteDifferenceAngleStepRadians)
			&& FiniteDifferenceAngleStepRadians
			> SteadyFlightTrimSolverInternal::MinimumPositiveValue
			&& std::isfinite(
				FiniteDifferenceThrottleStep)
			&& FiniteDifferenceThrottleStep
			> SteadyFlightTrimSolverInternal::MinimumPositiveValue
			&& std::isfinite(MinimumLineSearchScale)
			&& MinimumLineSearchScale
			> SteadyFlightTrimSolverInternal::MinimumPositiveValue
			&& MinimumLineSearchScale <= 1.0;
	}

	bool FSteadyFlightTrimSolver::TrySolve(
		const FAerodynamicModelConfiguration&
		AerodynamicConfiguration,
		const FPropulsionModelConfiguration&
		PropulsionConfiguration,
		const FAircraftMassProperties& MassProperties,
		const FAtmosphereState& Atmosphere,
		const FSteadyFlightTrimRequest& Request,
		FSteadyFlightTrimResult& OutResult)
	{
		return TrySolve(
			AerodynamicConfiguration,
			PropulsionConfiguration,
			MassProperties,
			Atmosphere,
			Request,
			FSteadyFlightTrimSolverSettings{},
			OutResult);
	}

	bool FSteadyFlightTrimSolver::TrySolve(
		const FAerodynamicModelConfiguration&
		AerodynamicConfiguration,
		const FPropulsionModelConfiguration&
		PropulsionConfiguration,
		const FAircraftMassProperties& MassProperties,
		const FAtmosphereState& Atmosphere,
		const FSteadyFlightTrimRequest& Request,
		const FSteadyFlightTrimSolverSettings& Settings,
		FSteadyFlightTrimResult& OutResult)
	{
		using namespace SteadyFlightTrimSolverInternal;

		if (!MassProperties.IsValid()
			|| !IsAtmosphereValid(Atmosphere)
			|| !Request.IsValid()
			|| !Settings.IsValid())
		{
			return false;
		}

		FAerodynamicModel AerodynamicModel;

		if (!AerodynamicModel.SetConfiguration(
			AerodynamicConfiguration))
		{
			return false;
		}

		FPropulsionModel PropulsionModel;

		if (!PropulsionModel.SetConfiguration(
			PropulsionConfiguration))
		{
			return false;
		}

		FTrimVariables Variables{
			Request.InitialAngleOfAttackRadians,
			Request.InitialElevatorRadians,
			Request.InitialThrottle
		};

		FTrimEvaluation CurrentEvaluation{};

		const auto Evaluate =
			[&](
				const FTrimVariables& CandidateVariables,
				FTrimEvaluation& OutEvaluation)
			{
				if (!SteadyFlightTrimSolverInternal::TryEvaluate(
					AerodynamicModel,
					PropulsionModel,
					MassProperties,
					Atmosphere,
					Request,
					CandidateVariables,
					OutEvaluation))
				{
					return false;
				}

				const double WeightNewtons =
					MassProperties.MassKilograms
					* Request
					.GravityAccelerationMetersPerSecondSquared;

				const double PitchMomentScale =
					WeightNewtons
					* AerodynamicConfiguration
					.ReferenceChordMeters;

				OutEvaluation.NormalizedResidual =
					FVector3{
						OutEvaluation.Result
						.ResidualForceBodyNewtons.X
						/ WeightNewtons,
						OutEvaluation.Result
						.ResidualForceBodyNewtons.Z
						/ WeightNewtons,
						OutEvaluation.Result
						.ResidualPitchMomentNewtonMeters
						/ PitchMomentScale
					};

				OutEvaluation.Result.NormalizedResidual =
					CalculateResidualNorm(
						OutEvaluation.NormalizedResidual);

				return std::isfinite(
					OutEvaluation.Result.NormalizedResidual);
			};

		if (!Evaluate(Variables, CurrentEvaluation))
		{
			return false;
		}

		for (int Iteration = 0;
			Iteration <= Settings.MaximumIterations;
			++Iteration)
		{
			if (CurrentEvaluation.Result.NormalizedResidual
				<= Settings.NormalizedResidualTolerance)
			{
				CurrentEvaluation.Result.IterationCount =
					Iteration;

				OutResult = CurrentEvaluation.Result;
				return true;
			}

			if (Iteration == Settings.MaximumIterations)
			{
				break;
			}

			double Jacobian[3][3]{};

			for (int VariableIndex = 0;
				VariableIndex < 3;
				++VariableIndex)
			{
				const double DifferenceStep =
					VariableIndex < 2
					? Settings
					.FiniteDifferenceAngleStepRadians
					: Settings
					.FiniteDifferenceThrottleStep;

				const double CurrentVariable =
					GetVariable(Variables, VariableIndex);

				const double LowerVariable =
					std::max(
						GetMinimumVariable(
							Request,
							VariableIndex),
						CurrentVariable - DifferenceStep);

				const double UpperVariable =
					std::min(
						GetMaximumVariable(
							Request,
							VariableIndex),
						CurrentVariable + DifferenceStep);

				const double DifferenceDenominator =
					UpperVariable - LowerVariable;

				if (DifferenceDenominator
					<= MinimumPositiveValue)
				{
					return false;
				}

				FTrimVariables LowerVariables = Variables;
				FTrimVariables UpperVariables = Variables;

				SetVariable(
					LowerVariables,
					VariableIndex,
					LowerVariable);

				SetVariable(
					UpperVariables,
					VariableIndex,
					UpperVariable);

				FTrimEvaluation LowerEvaluation{};
				FTrimEvaluation UpperEvaluation{};

				if (!Evaluate(
					LowerVariables,
					LowerEvaluation)
					|| !Evaluate(
						UpperVariables,
						UpperEvaluation))
				{
					return false;
				}

				for (int ResidualIndex = 0;
					ResidualIndex < 3;
					++ResidualIndex)
				{
					Jacobian[ResidualIndex][VariableIndex] =
						(GetResidualComponent(
							UpperEvaluation.NormalizedResidual,
							ResidualIndex)
							- GetResidualComponent(
								LowerEvaluation.NormalizedResidual,
								ResidualIndex))
						/ DifferenceDenominator;
				}
			}

			FVector3 NewtonStep{};

			if (!TrySolveLinearSystem(
				Jacobian,
				-CurrentEvaluation.NormalizedResidual,
				NewtonStep))
			{
				return false;
			}

			double LineSearchScale = 1.0;
			bool bAcceptedStep = false;

			while (LineSearchScale
				>= Settings.MinimumLineSearchScale)
			{
				const FTrimVariables CandidateVariables =
					ClampVariables(
						FTrimVariables{
							Variables.AngleOfAttackRadians
								+ LineSearchScale * NewtonStep.X,
							Variables.ElevatorRadians
								+ LineSearchScale * NewtonStep.Y,
							Variables.Throttle
								+ LineSearchScale * NewtonStep.Z
						},
						Request);

				FTrimEvaluation CandidateEvaluation{};

				if (!Evaluate(
					CandidateVariables,
					CandidateEvaluation))
				{
					return false;
				}

				if (CandidateEvaluation.Result.NormalizedResidual
					< CurrentEvaluation.Result.NormalizedResidual)
				{
					Variables = CandidateVariables;
					CurrentEvaluation = CandidateEvaluation;
					bAcceptedStep = true;
					break;
				}

				LineSearchScale *= 0.5;
			}

			if (!bAcceptedStep)
			{
				return false;
			}
		}

		return false;
	}
}
