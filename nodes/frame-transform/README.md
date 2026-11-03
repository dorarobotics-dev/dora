# dora-frame-transform

Transforms a `std/media/v1/PointCloud` into `TARGET_FRAME` using the robot
calibration (`DORA_FRAMES`, `DORA_EXTRINSICS`). If the route crosses one dynamic
edge, wire a `std/math/v1/Pose` input (with `child_frame` in metadata).
Unplaceable clouds are counted drops, reported on `diag`. Only x, y, z move.

Points are packed float32 in the layout `POINT_FIELDS` names (default `xyzi`,
declared on the ports as `std/media/v1/PointCloud[fields=xyzi]`); a buffer that
does not hold `width*height` such points is refused. Drops are counted by kind:
`bad_frame` (no or unknown `frame_id`, no route), `bad_header`, `malformed`
(a cloud that does not decode), `no_pose`, and `bad_pose` for a pose input.
