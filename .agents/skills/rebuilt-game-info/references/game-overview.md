# Game Overview

`REBUILT` is a fuel-scoring and climbing game. Alliances score `FUEL`, traverse obstacles, and climb the `TOWER` before time expires.

## Match Flow

- Match structure follows the standard FRC pattern: autonomous, teleoperated play, then endgame.
- During autonomous, robots act without driver input and can score preloaded or collected `FUEL`.
- During teleop, robots continue collecting and scoring `FUEL`, play defense, reposition for later cycles, and prepare for climbing.
- Near the end of the match, all hubs become active and climbing becomes increasingly valuable.

## Core Scoring Loop

- The main game piece is `FUEL`, a foam ball.
- Robots can preload `FUEL`, receive it from the human player, collect it from `DEPOTS`, and gather it from the carpeted field.
- The main scoring target is the alliance `HUB`.
- A key game mechanic is that hub availability changes during the match: an alliance should not assume it can score into its hub at every moment.
- When a hub is inactive, productive play likely shifts toward collecting, repositioning, defending, or setting up for the next scoring window.

## Endgame

- Robots can climb the `TOWER`.
- Higher climbs matter for points and ranking outcomes.
- Endgame design is not just about the mechanism; it also drives pathing, timing, sequencing, and how early a robot must disengage from scoring cycles.

## Robot-Relevant Takeaways

- REBUILT rewards a complete cycle view rather than isolated mechanisms.
- Scoring throughput depends on both shooter performance and whether the hub is currently active.
- Good autos should combine early `FUEL` scoring, controlled routing, and a clean handoff into teleop.
- Driver-assist feedback should help operators understand scoring availability, cycle timing, and endgame transition timing.

## Scope Note

Use the official manual for exact point values, ranking thresholds, and any wording-sensitive claims.
