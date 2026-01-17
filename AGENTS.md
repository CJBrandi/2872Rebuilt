# Repository Guidelines

## Project Structure & Module Organization
- `src/main/java/frc/robot` is the code root. Key packages include `commands`, `subsystems/drive`, `subsystems/superstructure`, and `util`.
- `src/main/java/frc/robot/generated` contains generated constants (do not hand-edit).
- `src/main/deploy` holds deployable assets like PathPlanner paths/autos and settings.
- `vendordeps` stores vendor library definitions used by GradleRIO.
- Gradle wrapper files (`gradlew`, `build.gradle`) control builds and robot deployment.

## Build, Test, and Development Commands
- `./gradlew build` — compiles Java 17 sources and assembles the robot jar.
- `./gradlew test` — runs JUnit 5 tests with the project’s test configuration.
- `./gradlew deploy` — deploys code and static files to the RoboRIO (requires team number in `.wpilib/wpilib_preferences.json`).
- `./gradlew replayWatch` — starts AdvantageKit log replay viewer for analysis.
- `./gradlew spotlessApply` — formats Java/Gradle/JSON/Markdown per repository rules.

## Coding Style & Naming Conventions
- Java is formatted via Spotless using Google Java Format; formatting runs on compile.
- Gradle files use 4-space indentation; JSON/Markdown use 2 spaces.
- Package naming follows `frc.robot.*`; classes use `PascalCase`, methods `camelCase`, constants `UPPER_SNAKE_CASE`.
- Hardware interfaces use `*IO` / `*IOSim` (e.g., `ModuleIO`, `ModuleIOSim`).

## Testing Guidelines
- Tests use JUnit Jupiter (`org.junit.jupiter`). Place tests under `src/test/java` mirroring package structure.
- No explicit coverage gate is configured; include focused unit tests for new logic.

## Commit & Pull Request Guidelines
- Commit messages commonly use a short type prefix: `feat:`, `tune:`, `upload:` (e.g., `feat: Add intake feedforward`).
- Keep commits small and descriptive; mention simulation/hardware testing in PRs.
- If deploying from an `event*` branch, the build may auto-commit with a timestamped message; avoid rewriting those commits.

## Security & Configuration Tips
- Team number and deploy settings live in `.wpilib/wpilib_preferences.json` (local to each developer).
- Vendor updates live in `vendordeps/*.json`; keep these in sync with hardware firmware.
