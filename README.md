# C152FlightSim

An independent flight simulation project developed from scratch by **Berk Sevimli** using C++ and Unreal Engine.

> This project is under active development. The current implementation focuses on simulation architecture, deterministic execution, coordinate-system integration, and automated verification.

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
- Future flight-dynamics and aircraft-system models

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
- Runtime state-to-Pawn integration
- Unreal Automation tests

## Current Simulation Status

The project currently provides the architecture and mathematical foundation required for rigid-body flight dynamics.

Implemented:

- Input command processing
- Control-surface command model
- Fixed-step simulation timing
- Vector and quaternion mathematics
- Aircraft-state storage
- Coordinate conversion
- Unreal runtime integration

Not yet implemented:

- Rigid-body 6DOF state propagation
- Gravity model
- Atmospheric model
- Aerodynamic force and moment model
- Propulsion model
- Landing-gear dynamics
- Detailed aircraft systems

The aircraft is not yet expected to produce physically simulated flight.

## Automated Tests

The project currently contains automated tests covering:

- Control-surface neutral return
- Control-surface rate limiting
- Control-surface saturation
- Throttle clamping
- Fixed-step count
- Frame-hitch protection
- Simulation reset and advance behavior
- Default aircraft-state initialization
- Quaternion normalization
- Vector algebra and normalization
- Quaternion rotation and composition
- FRD/NED coordinate conventions
- Unreal position conversion
- Unreal attitude conversion
- Aircraft-state pose round trips

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
git clone https://github.com/berksvml/C152FlightSim.git
cd C152FlightSim
git lfs install
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

### Phase 1 — Simulation Foundation

- [x] Enhanced Input
- [x] Control-surface command model
- [x] Deterministic fixed-step clock
- [x] Aircraft-state types
- [x] Vector and quaternion mathematics
- [x] Unreal coordinate adapter
- [x] Simulation facade
- [ ] Mass and inertia properties
- [ ] 6DOF rigid-body propagation
- [ ] Gravity and constant-load verification

### Phase 2 — Flight Environment

- [ ] Standard atmosphere
- [ ] Wind and turbulence
- [ ] Air-data calculations

### Phase 3 — Aircraft Models

- [ ] Aerodynamic coefficients
- [ ] Aerodynamic forces and moments
- [ ] Propulsion model
- [ ] Fuel and engine systems
- [ ] Landing-gear model

### Phase 4 — Unreal Integration

- [ ] Runtime 6DOF visualization
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
