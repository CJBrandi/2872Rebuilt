---
name: rebuilt-game-info
description: Context and terminology for the 2026 FRC game REBUILT presented by Haas. Use when Codex needs to understand the 2026 game, field, zones, objectives, scoring flow, match phases, or robot-design implications before reasoning about robot code, autos, mechanisms, strategy, or telemetry in this repo. Trigger on requests about "2026 game", "REBUILT", "game info", field objects, zones, objectives, scoring context, or robot behavior in the context of this game.
---

# Rebuilt Game Info

Use this skill to orient robot-code work to the 2026 FRC game before making design or implementation decisions.

This skill is for context, vocabulary, and robot-relevant understanding. Do not use it as the final authority for exact legality, dimensions, or point values when those details matter. For exact wording, defer to the official manual and field drawings.

## Workflow

Start by summarizing REBUILT in robot terms:

- what game pieces are manipulated
- where they are collected and scored
- how match flow changes between auto, teleop, and endgame
- which field elements matter for driving, intake, shooting, defense, and climbing
- which robot archetypes the game appears to reward

Then load only the reference file that matches the task:

- `references/game-overview.md` for match flow, scoring phases, and the active/inactive hub concept
- `references/field-and-objectives.md` for the field mental map: where `HUB`, `TOWER`, `OUTPOST`, `DEPOT`, `BUMPS`, `TRENCHES`, and named zones sit relative to each other
- `references/robot-design-implications.md` for likely mechanism choices, control problems, auto implications, and telemetry priorities

## Working Rules

- Use official FIRST material as the primary source of truth.
- Use `frcmanual.com` as a readability aid and section map, not as the final rules authority.
- Prefer qualitative summaries unless the user explicitly needs exact numbers or wording.
- When exact dimensions, point values, or rule legality matter, say so and consult the official manual or drawings.
- Keep answers grounded in robot behavior, not generic game-summary prose.
- When a user needs the field to be fully comprehensible, describe the layout as a mirrored set of alliance-end zones around a large neutral middle, then place each major element within that map before discussing strategy or code.

## Expected Output Shape

When this skill is used, explain the game in a way that helps robot-code decisions:

- identify the manipulated object: `FUEL`
- identify the main scoring structure: `HUB`
- identify traversal and endgame constraints around the `TOWER`, `BUMPS`, and `TRENCHES`
- explain that hub scoring availability changes during the match and affects cycle planning
- call out likely subsystem concerns such as intake, indexing, shooting, pathing, climb sequencing, and driver feedback

## Sources

Base this skill on:

- the official 2026 FIRST game manual and field assets
- the official FIRST game overview material
- `https://www.frcmanual.com/2026/game-overview` and nearby manual sections for readable structure and terminology cross-checking
