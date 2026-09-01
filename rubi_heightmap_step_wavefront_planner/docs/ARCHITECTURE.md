# Architecture

```text
PointCloud2 -> PointCloud2HeightmapAdapter -> immutable HeightmapSnapshot
                                                |
TF + Goal -> PlanningQueryResolver -> StepEvaluator -> WavefrontGraphBuilder -> TerrainGraph
                                      |                   |
                                      +-------------> AStarSearch
                                                            |
                                    path metrics/densify -> Path + Markers
```

The standard-C++ core contains snapshot, terrain evaluation, graph construction,
spatial indexing, A*, path revalidation, and lifecycle policy. ROS messages are
confined to the adapter, visualization, and runtime node targets.

| Component | Sole responsibility |
|---|---|
| `HeightmapSnapshot` | immutable observed/unknown elevation lattice and canonical hash |
| `StepEvaluator` | hard terrain validity and nonnegative height/clearance edge risk |
| `PlanningQueryResolver` | bounded deterministic projection onto a strict-valid lattice query |
| `UniformGridSpatialIndex2D` | exact deterministic merge/neighbor queries |
| `WavefrontGraphBuilder` | FIFO ring proposals and accepted graph topology |
| `AStarSearch` | deterministic optimal search on that graph |
| `StepWavefrontPlanner` | orchestration and path metrics |
| `PlanningFsm` | explicit verification, recovery, and bounded retry lifecycle |
| `PlannerNode` | TF, request worker, publication, and map/goal epochs |

The callback mutex owns the immutable map pointer together with its generation,
the current frame, goal epoch, queue, and active path. Planning copies the map
pointer into a request and runs outside the mutex. Publication rechecks goal
epoch, frame, and latest-map validation. The destructor signals and joins the
single worker.

## Hybrid comparison architecture

```text
Nav2 master Costmap --\
                       +--> Hybrid StepEvaluator --> implicit Grid A*
FastDEM Heightmap ----/                         \--> sampled graph --> A*
```

`CostmapSnapshot` and `HeightmapSnapshot` retain independent origins and
geometries; fusion queries are always in world coordinates. Grid and sampling
planners receive separate evaluator instances over the same immutable snapshot
pair. The comparison executable has no controller output and does not share the
production planner FSM.

Each accepted new complete snapshot increments `map_generation`. A frame change
fully resets Path and Markers. A same-frame update only revalidates the unpassed
path. The first invalid observation suspends public motion while retaining the
Path. Confirmation deletes it and begins fresh replanning; recovery confirmation
republishes it. Failed replans are gated by period, map generation, and a bounded
attempt budget. An external goal increments `goal_epoch` and has priority.
