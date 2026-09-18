# C152FlightSim

An independent flight simulation project developed from scratch by **Berk Sevimli** using C++ and Unreal Engine.

> This project is under active development. Phase 1 of the simulation foundation is complete, including deterministic execution, coordinate-system integration, rigid-body 6DOF propagation, Unreal runtime integration, and automated verification.

## Project Overview

C152FlightSim is a modular flight simulation project based on a single-engine training aircraft configuration.

Unreal Engine is used for:

- Real-time visualization
- User input
- Pawn and Actor integration
- Camera and scene management
- Runtime debugging

The flight-dynamics and aircraft-system models are designed as engine-agnostic C++ components. The core simulation code does not use Unreal-specific types such as `FVector`, `FRotator`, `UObject`, `AActor`, or `APawn`.

## Engineering Objectives

The project is designed to demonstrate:

- Physics-based flight simulation development
- Modular C++ simulation architecture
- Deterministic fixed-step execution
- Flight-dynamics coordinate-system management
- Unit and integration testing
- Unreal Engine integration through explicit adapters
- Separation of simulation logic from visualization

## Architecture

```mermaid
flowchart LR
    Input["Enhanced Input"] --> Pawn["Aircraft Pawn"]
    Pawn --> Simulation["C++ Simulation Core"]
    Simulation --> State["Aircraft State"]
    State --> Adapter["Unreal Adapter"]
    Adapter --> Actor["Actor Transform"]
```

The Pawn produces control commands and invokes the simulation. It does not solve the aircraft equations of motion.

The simulation core owns:

- Fixed-step execution
- Control-surface state
- Aircraft state
- Aircraft mass and inertia properties
- Rigid-body 6DOF state propagation
- Applied body forces and moments
- Future atmosphere, aerodynamics, propulsion, and aircraft-system models

The Unreal integration layer converts between the simulation coordinate systems and Unreal Engine world coordinates.

## Coordinate Systems

### Body Frame — FRD

The simulation uses a right-handed Forward–Right–Down body frame.

| Axis | Direction |
|---|---|
| +X | Forward |
| +Y | Right |
| +Z | Down |

### Navigation Frame — NED

The navigation frame uses North–East–Down coordinates.

| Axis | Direction |
|---|---|
| +X | North |
| +Y | East |
| +Z | Down |

### Unreal World

The Unreal adapter converts the simulation state to Unreal Engine coordinates.

| Simulation | Unreal |
|---|---|
| North | +X |
| East | +Y |
| Down | -Z |
| Metres | Centimetres |

Attitude is represented by a normalized quaternion describing the active rotation from Body/FRD to Navigation/NED.

## Current Features

- C++ aircraft Pawn
- Enhanced Input integration
- Pitch, roll, yaw, and throttle commands
- Control-surface rate limiting
- Control-surface saturation
- Throttle clamping
- Deterministic 120 Hz fixed-step simulation clock
- Frame-hitch protection
- Engine-agnostic vector and quaternion types
- Vector algebra, dot product, and cross product
- Quaternion composition and vector rotation
- Aircraft-state representation
- Body/FRD and Navigation/NED conventions
- Unreal coordinate and attitude adapters
- Aircraft-state to Unreal-transform conversion
- Simulation facade through `FC152Simulation`
- Aircraft mass and inertia tensor representation
- Translational and rotational 6DOF propagation
- Gravity and applied body-load support
- Quaternion-based attitude propagation
- Runtime state-to-Pawn integration
- Phase 1 straight-line runtime verification scenario
- 27 passing Unreal Automation tests

## Current Simulation Status

Phase 1 of the simulation foundation is complete. The current implementation can propagate an aircraft state through the full rigid-body translational and rotational equations of motion at a deterministic 120 Hz simulation rate.

Implemented:

- Input command processing
- Control-surface command model
- Fixed-step simulation timing
- Vector and quaternion mathematics
- Aircraft-state storage
- Coordinate conversion
- Aircraft mass and inertia properties
- Body-force and body-moment application
- Gravity transformation from NED to body axes
- Translational and rotational 6DOF state propagation
- Quaternion attitude integration and normalization
- Analytical constant-force, constant-moment, gravity, and constant-rate verification
- Unreal runtime integration

Not yet implemented:

- Atmospheric model
- Aerodynamic force and moment model
- Propulsion model
- Landing-gear dynamics
- Detailed aircraft systems

The current Unreal runtime scenario initializes the aircraft with a constant forward body velocity to verify the complete core-to-Unreal state path. It intentionally uses synthetic mass and inertia properties with gravity disabled. These values are not Cessna 152 reference data and will be replaced by validated aircraft configuration data.

Control inputs currently drive the control-surface model, but they do not yet generate aerodynamic forces or moments. The aircraft is therefore not yet expected to respond aerodynamically to pilot input.

## Automated Tests

The project currently contains **27 passing automated tests**.

| Test group | Coverage | Tests |
|---|---|---:|
| Control surfaces | Neutral return, rate limiting, saturation, throttle clamping | 4 |
| Fixed-step clock | Step count and frame-hitch protection | 2 |
| Mass properties | Validation and inertia round trip | 2 |
| Mathematics | Vector algebra, normalization, quaternion rotation and composition | 4 |
| Rigid-body 6DOF | Constant force, constant moment, constant yaw rate, and gravity | 4 |
| Simulation facade | Advance, reset, configuration, and rigid-body integration | 4 |
| Aircraft state | Default initialization and quaternion normalization | 2 |
| Unreal aircraft-state adapter | Transform conversion and pose round trip | 2 |
| Coordinate adapter | NED position, FRD body axes, and attitude conversion | 3 |
| **Total** |  | **27** |

Run all tests from the Unreal console:

```text
Automation RunTests C152FlightSim
```

## Repository Structure

```text
C152FlightSim/
├── Config/
├── Content/
│   └── C152FlightSim/
│       ├── Aircraft/
│       ├── Input/
│       └── Maps/
├── Source/
│   └── C152FlightSim/
│       ├── Public/
│       │   ├── FlightDynamics/
│       │   └── Integration/
│       └── Private/
│           ├── FlightDynamics/
│           ├── Integration/
│           └── Tests/
├── C152FlightSim.uproject
└── README.md
```

## Build Requirements

- Unreal Engine 5.8.2
- Visual Studio 2022
- Windows x64
- Git
- Git LFS

## Building the Project

Clone the repository and download the LFS-managed assets:

```bash
git lfs install
git clone https://github.com/berksvml/c152-flight-simulator.git
cd c152-flight-simulator
git lfs pull
```

Generate the Visual Studio project files if required.

Build using:

```text
Configuration: Development Editor
Platform: x64
```

Open `C152FlightSim.uproject` after the build completes.

## Development Roadmap

### Phase 1 — Simulation Foundation — Complete

- [x] Enhanced Input
- [x] Control-surface command model
- [x] Deterministic fixed-step clock
- [x] Aircraft-state types
- [x] Vector and quaternion mathematics
- [x] Unreal coordinate adapter
- [x] Simulation facade
- [x] Mass and inertia properties
- [x] 6DOF rigid-body propagation
- [x] Gravity and constant-load verification
- [x] Unreal Pawn runtime integration

### Phase 2 — Atmosphere and Air Data

- [ ] Standard atmosphere
- [ ] Wind and turbulence
- [ ] Air-data calculations

### Phase 3 — Aircraft Flight Model

- [ ] Validated C152 mass and inertia data
- [ ] Aerodynamic coefficients
- [ ] Aerodynamic forces and moments
- [ ] Propulsion model
- [ ] Fuel and engine systems
- [ ] Landing-gear model

### Phase 4 — Visualization and Interaction

- [x] Initial runtime 6DOF visualization
- [ ] Control-surface animation
- [ ] Cockpit instruments
- [ ] Expanded debug telemetry

## Verification Strategy

Each model is developed and tested independently before being connected to the complete simulation.

The verification approach includes:

- Deterministic fixed-step tests
- Mathematical identity tests
- Coordinate-convention tests
- State round-trip tests
- Force and moment test cases
- Known analytical rigid-body scenarios
- Runtime Unreal integration checks

## Disclaimer

This project is an engineering development and portfolio project. It is not certified or intended for operational flight training, aircraft design approval, or real-world navigation.

## Author

**Berk Sevimli**

Aerospace Engineer  
C++ Simulation and Flight Dynamics
