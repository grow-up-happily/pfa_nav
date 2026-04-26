# Nav2 Isolated Noise Filter Design

## Goal

Before Nav2 plans a path, ignore isolated and small-cluster obstacle noise that
appears in the custom `pb_nav2_costmap_2d::IntensityVoxelLayer`. The filter must
be configurable from YAML so the minimum obstacle cluster size can be tuned
without rebuilding.

## Current Behavior

Both simulation and reality Nav2 configs use `IntensityVoxelLayer` in local and
global costmaps. Each update resets the layer, reads PointCloud2 observations,
filters only by height, intensity, and range, then immediately writes every
accepted point's 2D cell as `LETHAL_OBSTACLE`. A single noisy point can therefore
enter the costmap and influence SmacPlannerHybrid.

## Design

Add a small connected-component filter inside `IntensityVoxelLayer` before cells
are written to `costmap_`.

The layer will first collect candidate obstacle cells from the current
observations. It will then group adjacent candidate cells into 8-connected 2D
components. A component is kept only when it contains enough candidate cells.
Rejected components are not written as lethal obstacles, so the planner sees them
as absent.

Configurable parameters:

- `noise_filter_enabled`: enable or disable the filter. Default `false` in code,
  set to `true` in this project's Nav2 configs.
- `noise_filter_min_cluster_cells`: minimum number of connected occupied
  candidate cells needed to keep an obstacle component. A value of `5` removes
  single-cell noise and a `2x2` four-cell noise block.

The requested "judgment grid count" is `noise_filter_min_cluster_cells`.

## Scope

Update the custom costmap layer and the two project Nav2 configs:

- `src/pb2025_sentry_nav/pb_nav2_plugins/.../intensity_voxel_layer.*`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`

The filter applies to both local and global costmaps in simulation and reality.

## Testing

Add focused unit tests for the connected-component keep/drop logic so parameter changes
are covered without needing to launch ROS. Then build/test `pb_nav2_plugins`.

Manual tuning can start with:

- `noise_filter_enabled: true`
- `noise_filter_min_cluster_cells: 5`

If real obstacles become too sparse, lower the threshold or disable the filter. If
noise still leaks through, raise `noise_filter_min_cluster_cells`.
