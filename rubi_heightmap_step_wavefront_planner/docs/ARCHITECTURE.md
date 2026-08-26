# Architecture

```text
PointCloud2 -> PointCloud2HeightmapAdapter -> immutable HeightmapSnapshot
                                                |
TF + Goal -> StepEvaluator -> WavefrontGraphBuilder -> TerrainGraph
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
| `UniformGridSpatialIndex2D` | exact deterministic merge/neighbor queries |
| `WavefrontGraphBuilder` | FIFO ring proposals and accepted graph topology |
| `AStarSearch` | deterministic optimal search on that graph |
| `StepWavefrontPlanner` | orchestration and path metrics |
| `PlannerNode` | TF, request worker, publication, and map/goal epochs |

The callback mutex owns the immutable map pointer together with its generation,
the current frame, goal epoch, queue, and active path. Planning copies the map
pointer into a request and runs outside the mutex. Publication rechecks goal
epoch, frame, and latest-map validation. The destructor signals and joins the
single worker.

Each accepted new complete snapshot increments `map_generation`. A frame change
fully resets Path and Markers. A same-frame update only revalidates the unpassed
path. An invalid path can schedule one fresh-graph automatic request; an external
goal increments `goal_epoch` and has priority.
