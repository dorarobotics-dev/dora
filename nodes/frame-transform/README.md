# dora-frame-transform

Transforms a `std/media/v1/PointCloud` into `TARGET_FRAME` using the robot
calibration (`DORA_FRAMES`, `DORA_EXTRINSICS`). If the route crosses one dynamic
edge, wire a `std/math/v1/Pose` input (with `child_frame` in metadata).
Unplaceable clouds are counted drops, reported on `diag`. Only x, y, z move.
