# Original TRG node-construction adaptation

The `original_trg_random_ring` policy independently adapts the graph expansion
lifecycle observed in the upstream [TRG-planner](https://github.com/wasahaiah/TRG-planner)
`expandGraph()`, `isCollision()`, and `cleanGraph()` implementation. No upstream
source code was copied. RB-TRG remains a research reference only; body-aware
ordered-pair state is outside this Phase-1 experiment.

| Item | Original TRG | RUBI Sampling |
|---|---|---|
| Root | shifted/randomized root in the original workflow | same snapped Costmap start cell as Grid |
| Sampling | fixed-radius uniform random ring | same |
| Valid sample target | `sample_num` valid candidates | same; `trg_sample_num` |
| Trial cap | 1000 | `trg_max_trial_samples` per expanded reference node |
| Collision query | KD-tree raw terrain points within robot size | observed Heightmap cells within `trg_robot_size_m` |
| Collision statistic | sorted upper median and strict `outlier_ratio > threshold` | same logic |
| Node z | nearest terrain point z | deterministic nearest observed cell z |
| Existing-node merge | nearest distance `< robot_size` | same, independent of legacy `merge_radius_m` |
| Node states | Valid / Invalid / Frontier | same temporary lifecycle; invalid/isolated nodes cleaned |
| Parent/neighbor wiring | TRG `wireEdge()` | common RUBI Hybrid evaluator |
| Edge terrain risk | PCA/slope/footprint gradients | raw Costmap inflation plus height discontinuity |
| RNG | `random_device` seeded | fixed seed by default; optional randomized seed |
| Search | TRG A* | existing deterministic RUBI graph A* |

## Construction lifecycle

Each accepted reference node is popped from a FIFO queue. The builder attempts
to collect `trg_sample_num` terrain-valid fixed-radius samples, stopping after
`trg_max_trial_samples`. Costmap 253/254/255 is rejected before the local TRG
height test. The height test requires at least one observed cell, selects
`values[N/2]`, and rejects only when the fraction farther than
`trg_height_threshold_m` is strictly greater than `trg_collision_threshold`.

For each valid sample, the nearest accepted graph node is queried. A distance
strictly below `trg_robot_size_m` performs only source-to-existing rewiring. A
farther sample creates a temporary Frontier node, attempts its parent edge and
nearby edges within `trg_neighbor_connection_radius_m`, and enters the FIFO
only if at least one Hybrid edge is valid. Otherwise it becomes Invalid and an
isolated rejection is retained for RViz. Final cleaning removes Invalid,
zero-degree nodes, and their incident edges and deterministically reindexes the
remaining graph.

The Hybrid evaluator, not the TRG collision statistic, owns every edge's final
feasibility and cost. This preserves the common Grid/Sampling experimental
contract while holding representation and node construction apart.
