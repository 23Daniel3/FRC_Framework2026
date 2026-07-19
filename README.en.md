# FRC Framework 2026

English version of the project guide.

**Language:** [Português](./README.md) | **English**

FRC robot framework in Java (WPILib + GradleRIO + CTRE Phoenix 6 + PathPlanner + AdvantageKit), built to speed up subsystem development with a **Request → State (FSM)** architecture.

> This README is the main GitHub-facing guide in English. For deeper internal conventions, also see `src/main/java/frc/TEMPLATE.md`.

---

## Architecture overview

The project is split into 3 layers:

- `src/main/java/frc/lib/` → reusable infrastructure (FSM, utilities, control helpers, zones, wrappers).
- `src/main/java/frc/game/` → game-specific rules and field geometry (poses, zones, alliance logic).
- `src/main/java/frc/robot/` → season robot implementation (subsystems, commands, factories, bindings).

### Control flow (teleop/auto)

1. Driver input is interpreted by `SuperStructureCommands.manageRequests(...)`.
2. It writes a `RobotRequest` into `SuperStructure`.
3. `SuperStructure` FSM resolves the global robot state.
4. On each global state `onEnter`, `SuperStructure` writes subsystem requests (`setRequest(...)`).
5. Each subsystem runs its own FSM and applies outputs through IO/hardware.

This model prevents command conflicts and centralizes intent.

---

## Project layout (summary)

```text
src/
  main/
    java/frc/
      lib/
      game/
      robot/
        commands/
        factories/
        subsystems/
          drivetrain/
          superstructure/
          intake/
          conveyor/
          shooter/
          led/
          vision/
          example/
      tools/
        new_subsystem.py
  test/
    java/frc/
      lib/
```

---

## How subsystems work

The core pattern is `StateSubsystem<Request, State, Inputs, IO>`.

### Pattern contract

- **External input:** `setRequest(Request)`
- **Internal behavior:** declarative FSM (`StateMachine`)
- **Output:** IO control calls (`io.controlXxx()...`)
- **Completion criterion:** `atGoal()` derived from `bindRequest(...)`

### Team conventions

- Use `onEnter` for discrete actions (stop motor, change mode).
- Use `onUpdate` for dynamic setpoints (e.g., shot-on-the-move RPM).
- End each subsystem FSM setup with `fsm.validateComplete()`.
- Avoid `sensor == 0`; use tolerance/deadband.

### Canonical example

Use `src/main/java/frc/robot/subsystems/example/` as the reference implementation of:

- `XxxConstants`
- `XxxIO` (`@AutoLog` inputs)
- `XxxIOHardware`
- `XxxSubsystem` with FSM + `bindRequest`

---

## Swerve drivetrain (CTRE Phoenix 6)

Main file: `src/main/java/frc/robot/subsystems/drivetrain/Drivetrain.java` (extends `TunerSwerveDrivetrain`).

### Key responsibilities

- Field-relative and robot-relative motion control.
- `AutoBuilder` configuration (PathPlanner).
- Acceleration limiting + anti-tipping.
- SysId routines (translation, steer, rotation).
- Alliance-aware frame flipping.

### Command strategy

`src/main/java/frc/robot/factories/DrivetrainCommands.java` contains a shared `driveCore(...)` that:

- applies translation modifiers,
- resolves rotation strategy,
- applies alliance flip,
- sends final `ChassisSpeeds` to drivetrain.

This supports joystick, heading hold, aim assist, zone modifiers, and pathfinding commands.

### Speed-limit ownership model

Drivetrain arbitrates by **minimum** between:

- state-level cap (`setMaxSpeed`, written by `SuperStructure`),
- pilot-level cap (`setPilotMaxSpeed`, e.g., slow mode).

So neither side unintentionally overrides the other.

---

## SuperStructure (global orchestrator)

File: `src/main/java/frc/robot/subsystems/superstructure/SuperStructure.java`.

### Responsibilities

- Owns the global `RobotState` FSM.
- Maps `RobotRequest` to subsystem requests.
- Coordinates modes (collect, shoot, collect+shoot, close, idle).
- Updates drivetrain max-speed constraints by state.
- Computes active shot parameters (shot-on-the-move).

### Auto + LED integration

- LEDs observe robot state via `followRobotState`.
- Auto routines use `SuperStructureCommands` factories (`shoot`, `collect`, `collectShooting`, etc.), instead of writing subsystem requests directly.

---

## Commands and factories

- `src/main/java/frc/robot/commands/` → command implementations.
- `src/main/java/frc/robot/factories/` → reusable command factories by domain/subsystem.

### Recommended rule

For global robot intent, route through `SuperStructure` request factories rather than direct subsystem request writes.

---

## Automated tests

Tests live in `src/test/java/frc/` and focus on logic that can run without hardware.

### Current suites

- `StateMachineTest`
  - global > local priority,
  - max one transition per update,
  - `transitionAfter` in seconds,
  - `forceState`, `validateComplete`, time-in-state behavior.
- `StateSubsystemTest`
  - `bindRequest` contract,
  - derived `atGoal()`,
  - no unintended re-entry,
  - request change while in-flight.
- `SetpointTrackerTest`, `LinearInterpolationTest`, `Polygon2dTest`
  - math/geometry utility coverage.

### Run tests/build (PowerShell)

```powershell
./gradlew test
./gradlew build
```

---

## Generate new subsystems

Generator script:

- `src/main/java/frc/tools/new_subsystem.py`
- based on `src/main/java/frc/robot/subsystems/example/`

### Usage from repo root

```powershell
python src/main/java/frc/tools/new_subsystem.py Climber
python src/main/java/frc/tools/new_subsystem.py ArmPivot --vendor talonfx
```

Supported vendors include `sparkmax`, `sparkflex`, and `talonfx`.

### After generating

1. Update IDs/config in `XxxConstants`.
2. Define `Request` and `State` enums.
3. Implement FSM + `bindRequest(...)`.
4. Instantiate in `RobotContainer` for each mode (`REAL`, `SIM`, replay).
5. Integrate requests into `SuperStructure` FSM if part of global workflow.

---

## Create new commands

### 1) Define command intent

- local subsystem action?
- global robot intent via `RobotRequest`?
- auto/path behavior?

### 2) Place code correctly

- specific command: `src/main/java/frc/robot/commands/...`
- reusable factory: `src/main/java/frc/robot/factories/...`

### 3) Recommended patterns

- global intent: prefer `SuperStructureCommands`.
- continuous behavior: `Commands.run(...)`.
- start/stop lifecycle: `Commands.startEnd(...)`.
- use `withName(...)` for clean telemetry/logging.

### 4) Wire it in `RobotContainer`

- button bindings,
- default commands,
- contextual triggers (disabled/autonomous/teleop).

---

## Quick dev setup

Prerequisites:

- JDK 17 (WPILib 2026)
- Gradle Wrapper (already included)
- VS Code + WPILib/FRC extensions

Useful commands (PowerShell):

```powershell
./gradlew spotlessApply
./gradlew test
./gradlew build
```

---

## CI and code quality

- Spotless + Google Java Format is configured.
- `build` depends on `spotlessApply` in `build.gradle`.
- See `README-CI-TESTES.md` for CI/testing details and suite scope.

---

## Contributing notes

- Follow architecture rules in `TEMPLATE.md`.
- Avoid bypassing `SuperStructure` intent routing.
- When changing core infra (`lib/interfaces/fsm`, etc.), include/update tests.

---

## License

See `WPILib-License.md` and vendor licenses in `vendordeps/`.
