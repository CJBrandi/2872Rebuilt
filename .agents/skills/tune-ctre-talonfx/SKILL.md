---
name: tune-ctre-talonfx
description: Use when tuning CTRE Phoenix 6 TalonFX velocity or Motion Magic loops on an FRC robot with a persistent robot-side tuner server, safe mechanism limits, current limits, telemetry, or iterative PID/feedforward gain tests.
---

# Tune CTRE TalonFX

## Overview

Use this skill to run bounded TalonFX tuning trials through a Phoenix-style robot-side server. Deploy the server once, wait for readiness, run repeated confirmed tests, analyze telemetry, and stop/neutralize after every trial.

## Required Context

Before running hardware, collect these details in chat:

- Mechanism type, control mode, and whether torque-current testing is allowed.
- Safe min/max position or safe max velocity.
- Initial setpoints, tolerance, max duration, and hard stop conditions.
- Max voltage, stator/supply current limits, and torque-current peak limits.
- Leader/follower behavior, inversion, neutral mode, gear ratio, and mechanism units.

If any hard safety limit is missing, do not run a closed-loop test. Start with a tiny smoke test.

## Workflow

1. Read `references/talonfx-tuning.md` for mode-specific tuning rules.
2. Verify the repo vendordep versions and local Phoenix source/Javadocs before relying on API names.
3. Start the persistent server once:
   ```bash
   python3 .agents/skills/tune-ctre-talonfx/scripts/talonfx_tuner.py deploy-server --team 2872 --leader-id <id> --can-bus <bus>
   ```
4. Wait for readiness:
   ```bash
   python3 .agents/skills/tune-ctre-talonfx/scripts/talonfx_tuner.py wait-ready --team 2872
   ```
5. Create a JSON test plan from the chat-agreed limits and gains, then run:
   ```bash
   python3 .agents/skills/tune-ctre-talonfx/scripts/talonfx_tuner.py run-test --team 2872 --plan /path/to/plan.json
   ```
6. Analyze returned telemetry. Propose one gain/current-limit change at a time.
7. Use `stop` before walking away, changing mechanisms, or redeploying normal robot code.

## Server Contract

The bundled asset `assets/talonfx-tuner-server` is a minimal WPILib robot project. It:

- Instantiates the configured leader and follower TalonFXs so Phoenix Diagnostics starts.
- Starts a separate HTTP tuner API on port `5805`.
- Provides `/health`, `/run`, `/stop`, and `/data/latest`.
- Keeps outputs neutral unless a confirmed, time-limited `/run` is active.

The tuner server replaces the normal deployed robot code until normal code is redeployed.

## Safety Rules

- Never run without typed `yes` confirmation in the CLI.
- Every run must include `durationSec`, setpoint bounds, and output/current caps.
- Position and Motion Magic tests must include min/max safe travel.
- Torque-current modes must use `PeakForwardTorqueCurrent` and `PeakReverseTorqueCurrent`; abort if `UnlicensedFeatureInUse` appears.
- Voltage modes must use conservative stator/supply limits and voltage peaks.
- Followers may only use Phoenix follower control from the leader.
- On timeout, Ctrl+C, HTTP failure, DS disconnect, or CTRE fault, send neutral and disable.

## Common Mistakes

| Mistake | Fix |
|---|---|
| Redeploying for each iteration | Deploy once, then reuse `/run` until tuning is done. |
| Letting the CLI choose safety limits | Safety limits come from chat/user mechanism details before testing. |
| Tuning torque current like voltage | Torque-current gains output amps; voltage gains output volts. |
| Ignoring current telemetry | Score both tracking and current draw before accepting gains. |
| Testing a full-range move first | Start with a tiny smoke test inside the safe envelope. |
