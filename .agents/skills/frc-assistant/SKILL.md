---
name: frc-assistant
description: >
  Unified FRC support for WPILib architecture, AdvantageKit logging, robot debugging,
  .wpilog analysis, and live NetworkTables investigation. Invoke when the user asks about
  WPILib command/subsystem structure, scheduler behavior, AdvantageKit replay/logging,
  robot telemetry, NetworkTables, or general FRC debugging. Trigger on: "wpilib",
  "advantagekit", "frc docs", "networktables", "wpilog", "scope", "analyze log",
  "robot debugging", "codex-scope", "frc-assistant".
---

# FRC Assistant

Use this as the default project skill for FRC work in this repo. It is the single entrypoint for:

- WPILib and command-based reference questions
- AdvantageKit logging and replay questions
- robot architecture and debugging guidance
- `.wpilog` analysis
- live NetworkTables investigation

## Backend Routing

Choose the backend that matches the user intent instead of defaulting to one tool.

### 1. `frc-docs` MCP first for official/reference questions

Use the project-local `frc-docs` MCP server first when the user wants:

- WPILib command/subsystem patterns
- scheduler behavior
- API or framework reference
- official FRC docs-style explanations
- architecture guidance that should stay close to canonical documentation

Examples:

- "How should I structure a WPILib command/subsystem interaction?"
- "What is the right command-based pattern for default commands?"
- "How does the scheduler handle conflicting requirements?"

### 2. Global `frc` MCP second for broader FRC help

Use the global `frc` MCP server when:

- `frc-docs` is too narrow or cannot answer the question well
- you need broader FRC-specific guidance rather than official docs alone
- the question is about robot debugging or FRC workflows that are not just API reference

Examples:

- "What are good ways to debug intermittent autonomous state issues on an FRC robot?"
- "What FRC-specific failure modes should I check before blaming tuning?"

### 3. `ClaudeScope` only for telemetry and live-data analysis

Use the external `ClaudeScope` CLI only when the user provides or clearly asks for:

- a `.wpilog` file
- live NetworkTables inspection
- field values at a timestamp
- time-range analysis
- signal statistics
- threshold spans, enabled windows, or telemetry-driven debugging

Examples:

- "Analyze this `.wpilog` and check module tracking error."
- "Load this log and find when voltage dipped under load."
- "Query the current NT values from the robot."

## Fallback Rules

### If `ClaudeScope` is unavailable

`ClaudeScope` must be on `PATH` to do log or live NT work. If it is missing:

- say that clearly
- do not pretend telemetry tooling is available
- continue with reference-based debugging help if possible
- tell the user what is needed to enable the telemetry path

Verify with:

```bash
ClaudeScope version
```

### If docs are insufficient

If `frc-docs` does not answer the question well enough:

- fall back to the global `frc` MCP server
- keep moving instead of stopping at the first incomplete answer

### If the request is ambiguous

If the user asks a general FRC debugging question without a log, NT target, or explicit telemetry request:

- start with the reference side (`frc-docs`, then global `frc`)
- do not jump into `ClaudeScope`
- only move to telemetry tooling if the user provides a log or asks for live/log analysis

## Prerequisites

### `ClaudeScope`

`ClaudeScope` is the telemetry backend. It is optional for docs/reference work and required only for log/live-data analysis.

On this machine it is not currently installed, so telemetry flows must fail over cleanly unless the CLI is installed first.

Install from the [GitHub Releases page](https://github.com/rylero/TheFRCSuite/releases) and add it to `PATH`.

| Platform | Binary |
|---|---|
| Windows | `ClaudeScope-windows-amd64.exe` -> rename to `ClaudeScope.exe` |
| macOS (Apple Silicon) | `ClaudeScope-darwin-arm64` -> rename to `ClaudeScope` |
| macOS (Intel) | `ClaudeScope-darwin-amd64` -> rename to `ClaudeScope` |
| Linux | `ClaudeScope-linux-amd64` -> rename to `ClaudeScope` |

## Telemetry Workflow

Use this only when the request truly needs telemetry inspection.

```text
1. Load a log or connect to NT -> get session_id
2. Query the relevant fields with --session <id>
3. Synthesize the robot-level finding, not just raw command output
4. Disconnect when done
```

Git Bash note: keys that start with `/` may need `MSYS_NO_PATHCONV=1`.

## Common Telemetry Commands

### Load a `.wpilog` file

```bash
MSYS_NO_PATHCONV=1 ClaudeScope load "C:/path/to/file.wpilog"
```

### Connect to live NT

```bash
ClaudeScope connect 10.0.0.2
```

### Inspect session fields and time range

```bash
ClaudeScope info --session <id>
```

### Query a value at a timestamp

```bash
MSYS_NO_PATHCONV=1 ClaudeScope get /RealOutputs/Superstructure/State --session <id> --time 1500000
```

### Query a time range

```bash
MSYS_NO_PATHCONV=1 ClaudeScope range /RealOutputs/Drive/LeftVelocity --session <id> --start 1000000 --end 5000000
```

### Find enabled windows or boolean spans

```bash
MSYS_NO_PATHCONV=1 ClaudeScope find-bool /RealOutputs/Robot/Enabled true --session <id>
```

### Compute numeric stats

```bash
MSYS_NO_PATHCONV=1 ClaudeScope stats /RealOutputs/Drive/LeftVelocity --session <id> --start 0 --end 0
```

## AdvantageKit and Robot-Code Guidance

Use the reference backends for questions like:

- how to organize subsystems and commands in this robot codebase
- how to reason about scheduler behavior and command ownership
- how to structure logging for replay and debugging
- how to interpret common AdvantageKit field patterns before opening a log tool
- what FRC-specific debugging workflow makes sense before deeper telemetry inspection

Example:

- "Explain AdvantageKit replay/logging patterns for this robot codebase."

This should route to reference backends, not directly to `ClaudeScope`.

## Telemetry Patterns

Common `.wpilog` analysis patterns:

- compare setpoint vs measured position for tracking error
- find enabled windows before analyzing behavior
- inspect voltage/current/velocity around an event window
- group state transitions by value to estimate time-in-state
- use stats to summarize noisy signals before forming conclusions

## Working Rules

- Prefer the smallest backend that answers the question well.
- Use docs/reference tools before telemetry tools unless the user clearly asks for log or live-data analysis.
- When using telemetry tools, synthesize findings into robot behavior and likely causes.
- Be explicit about missing prerequisites instead of silently skipping a backend.
