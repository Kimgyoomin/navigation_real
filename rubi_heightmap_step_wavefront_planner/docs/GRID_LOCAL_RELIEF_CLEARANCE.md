# Grid Local Relief hard clearance

This feature is intentionally Grid-only. Sampling/TRG still uses the legacy
`StepEvaluator::evaluateEdge()` contract and its RNG/graph construction is not
connected to the clearance snapshot.

## Hazard and distance contract

A seed is an observed height cell whose existing `supportedLocalRelief()`
result is valid and strictly greater than
`evaluation.local_relief_threshold_m`. Unknown or invalid relief evidence is
kept invalid; it is never converted to free space.

Each seed represents its closed square cell area. The configured hard distance
is measured from the robot reference-point path to the union of those squares,
not from seed centers, a physical step edge, the footprint, or the robot body.
Point-to-square and segment-to-square distances are analytic. Passing queries
whose candidate search is intentionally limited to the decision halo report a
`lower_bound`; failing distances are exact. Equality at the configured distance
is rejected conservatively with a single `1e-12` tolerance.

## Grid evaluation flow

`LocalReliefHazardSnapshot::build()` prepares immutable seed/evidence masks for
the complete height snapshot. `StepGridAStarPlanner` checks start, goal, and
every adjacent transition. `GridPathEvaluator` then replays raw paths, evaluates
continuous smoothing candidates, final paths, and remaining paths during map
updates. Adjacent raw replay delegates to `evaluateGridTransition()`, preserving
the A* distance, destination-cell inflation, height, and 5x5 Sobel cost exactly.

The cache identity includes height content/geometry and every parameter used by
the Local Relief seed calculation. A content or geometry change rebuilds it.
`sampling_only` does not build this snapshot.

## Experiment profiles

- `hybrid_grid_clearance_control.yaml`: clearance disabled.
- `hybrid_grid_clearance_020m.yaml`: hard clearance enabled at 0.20 m.

The profiles otherwise match: Grid-only, 5x5 Sobel soft weight 2.0, Local
Relief threshold 0.10 m, and refiner/replanning disabled for the initial A/B.
They do not modify the existing experiment profiles.
