# bimanual_move_robotics

Lightweight **bimanual manipulation** stack for humanoid robots, built for
**ROS 2 Humble**. It plugs onto any `robot_description` package, talks directly
to the robot's `FollowJointTrajectory` / `GripperCommand` controllers, and
exposes ROS actions to run a single move or a chain of moves
(*approach → grasp → retreat*, …).

It deliberately **does not use MoveIt's planning pipeline**. Kinematics are done
with **KDL** and collision checking with **FCL**, with simple dense
interpolation in joint / Cartesian space. The result is fast to compute and
fast to execute, while still preventing the robot from colliding **with itself**
or with **collision objects** you add at runtime.

Everything that depends on your robot — move groups, joint names, controller
names, named poses, collision rules, sequences — lives in **YAML files**, not in
the code.

---

## Packages

| Package | Description |
|---|---|
| `bimanual_msgs` | Action / message / service definitions. |
| `bimanual_manipulation` | The C++ engine and the `manipulation_server_node`. |

## Architecture

```
                       ┌──────────────────────────────────────────┐
   /robot_description  │            manipulation_server            │
   (URDF) ───────────► │                                            │
                       │  KDL kinematics  ─┐                        │
   /joint_states ────► │  FCL collision    ├─► trajectory generator │
                       │  (self + world)  ─┘        (joint / cart)  │
   config/*.yaml ────► │                                  │         │
                       └──────────────────────────────────┼─────────┘
   ~/move (action) ───►                                   ▼
   ~/execute_sequence ─►        FollowJointTrajectory / GripperCommand
   ~/manage_collision_object        (left_arm, right_arm, grippers…)
```

* **Kinematics** (`kinematics.cpp`) — one KDL chain per Cartesian group, FK +
  Levenberg-Marquardt IK.
* **Collision** (`collision_model.cpp`) — FCL world built from the URDF
  collision geometry (boxes/spheres/cylinders/meshes). A small URDF forward-
  kinematics walker places every link, so a full configuration is validated in
  `O(num_links)`. Self-collision uses an SRDF-style allowed-pairs list; world
  objects are added/removed at runtime.
* **Trajectory generation** (`trajectory_generator.cpp`) — dense linear
  interpolation in joint space, or straight-line Cartesian motion with per-
  waypoint IK. **Every waypoint is collision-checked**, then the path is
  velocity/acceleration-limited time-parameterized.
* **Move groups** (`move_group.cpp`) — own the controller action clients and
  split a trajectory across several controllers so **both arms move at once**.
* **Server** (`manipulation_server.cpp`) — loads config + URDF and serves the
  ROS interfaces.

---

## Build

Place the two packages in a colcon workspace next to your robot's
`robot_description` package.

```bash
# system deps (Ubuntu 22.04 / Humble)
sudo apt install ros-humble-kdl-parser ros-humble-orocos-kdl-vendor \
                 ros-humble-resource-retriever ros-humble-control-msgs \
                 ros-humble-srdfdom ros-humble-tf2-ros ros-humble-tf2-eigen \
                 libfcl-dev libassimp-dev libyaml-cpp-dev libeigen3-dev

cd ~/ros2_ws
# build bimanual_msgs first (bimanual_manipulation depends on its messages);
# --packages-select keeps the right order automatically.
colcon build --packages-select bimanual_msgs bimanual_manipulation --symlink-install
source install/setup.bash
```

> `--symlink-install` lets you edit the YAML configs without rebuilding. After
> changing a `.msg`/`.action`/`.srv`, rebuild **both** packages.

## Run

The server needs the URDF and the controllers up (`ros2_control` with a
`FollowJointTrajectory` controller per arm and, optionally, a `GripperCommand`
controller per gripper).

```bash
ros2 launch bimanual_manipulation bimanual_manipulation.launch.py \
    move_groups_config:=/path/to/your/move_groups.yaml \
    named_poses_config:=/path/to/your/named_poses.yaml \
    collision_config:=/path/to/your/collision.yaml \
    sequences_config:=/path/to/your/sequences.yaml \
    srdf_config:=/path/to/your/robot.srdf \
    robot_description_file:=/path/to/full_robot.urdf
```

Defaults point at the example configs in `bimanual_manipulation/config/`. Edit
those (or pass your own) to match your robot's link/joint/controller names.

**Where the URDF comes from** (priority order):
1. `robot_description` parameter (a URDF string), else
2. `robot_description_file` — a URDF **file** on disk, else
3. the `/robot_description` topic (default).

Use `robot_description_file` when the live `/robot_description` is a
kinematics-only URDF with no `<collision>`/`<visual>` geometry: collision
checking needs the geometry, so point it at the full description URDF (the one
with meshes). Joint names must match the controllers / `/joint_states`.

**SRDF (`srdf_config`, optional but recommended).** A MoveIt `.srdf`'s
`disable_collisions` pairs are merged into the allowed-collision matrix, so
links that can never collide (or are adjacent) are not checked — exactly like
MoveIt. Pairs referencing links absent from the URDF are ignored.

On startup the server logs a summary you should glance at:

```
Loaded N disabled collision pairs from SRDF ...
Collision model: P shapes, Q link pairs checked.
Meshes: 33 loaded, 0 failed, NNNN collision triangles total.
[WARN] X link(s) have NO collision geometry ...      # if any
```

---

## Configuration (all YAML)

* **`move_groups.yaml`** — groups, their joints, base/tip links and the
  controllers they drive. A group becomes *Cartesian capable* when it has both
  `base_link` and `tip_link`. Aggregate groups (e.g. `both_arms`) list several
  controllers and are driven simultaneously.
* **`named_poses.yaml`** — joint-space targets per group; the first value of a
  gripper pose is used as the gripper command.
* **`collision.yaml`** — collision settings:
  * `enabled` — master on/off.
  * `margin` — separation distance kept from everything (self + world objects,
    **meshes included**); the robot is rejected from getting closer than this.
    `0` = plain contact test (fastest), `0.02`–`0.05` gives a safety gap.
  * `padding` — primitive-only geometry inflation (no effect on meshes; use
    `margin` instead for mesh robots).
  * `mesh_decimation` — grid size [m] for simplifying detailed collision meshes
    at load (huge speedup; `0` keeps full resolution).
  * `resolution` — joint-space step between collision-checked waypoints.
  * `disabled_pairs` — SRDF-style list of link pairs to ignore (in addition to
    the SRDF and to parent/child adjacency).
* **`sequences.yaml`** — named lists of steps (`named` / `joint` / `cartesian` /
  `gripper`), e.g. `pick_above_object`, `both_ready`.

See the heavily commented examples under `bimanual_manipulation/config/`.

---

## ROS interface

### `~/move` — `bimanual_msgs/action/Move`
Run one `MotionStep`.

```bash
# joint-space named pose
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 0, group: left_arm, named_target: ready}}"

# straight-line approach: move the left tool 10 cm down its own Z axis
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 2, group: left_arm, relative: true, offset_in_tip_frame: true,
           offset: {x: 0.0, y: 0.0, z: -0.10}, velocity_scaling: 0.15}}"

# pose target defined relative to a TF frame (MoveIt-like "pose in frame_id"):
# bring the left tip 15 cm above the frame 'target_object', solving IK + joint
# interpolation (cartesian_path:false). Set cartesian_path:true for a straight
# line (precise approach/retreat).
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 2, group: left_arm, reference_frame: target_object,
           pose_target: {position: {x: 0.0, y: 0.0, z: 0.15},
                         orientation: {w: 1.0}}, cartesian_path: false}}"

# close the left gripper
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 3, group: left_gripper, named_target: closed}}"
```

`step.type`: `0` named, `1` joint, `2` cartesian, `3` gripper. For type `2`, the
goal pose is taken in `reference_frame` (any TF frame) when set, else relative to
the current tip when `relative:true`, else in the arm's base frame.

### `~/execute_sequence` — `bimanual_msgs/action/ExecuteSequence`
Run a predefined or inline sequence (approach → grasp → retreat …).

```bash
ros2 action send_goal -f /bimanual_manipulation_server/execute_sequence \
  bimanual_msgs/action/ExecuteSequence "{sequence_name: both_ready}"
```

**Fluid chaining.** Consecutive steps acting on the **same** move group are
concatenated into a **single continuous trajectory** — the arm does not stop at
each step's end, velocities flow through the via points. A gripper step or a
group change is a barrier that closes the current trajectory. Per step,
`blend_radius` (rad of joint space, sequences only) rounds the corner with the
next step for smoother, more human-like motion (re-validated for collisions,
reverted if unsafe). The combined trajectory uses the **slowest** scaling among
its steps. See `wave_left` in `sequences.yaml`.

### `~/manage_collision_object` — `bimanual_msgs/srv/ManageCollisionObject`
Add / remove / attach world collision objects (checked on every motion).

```bash
# add a 20 cm box on the table, 60 cm in front of the robot
ros2 service call /bimanual_manipulation_server/manage_collision_object \
  bimanual_msgs/srv/ManageCollisionObject \
  "{operation: 0, id: table_box,
    primitive: {type: 1, dimensions: [0.2, 0.2, 0.2]},
    pose: {header: {frame_id: ''}, pose: {position: {x: 0.6, y: 0.0, z: 0.1},
           orientation: {w: 1.0}}}}"
```

`operation`: `0` ADD, `1` REMOVE, `2` ATTACH, `3` DETACH, `4` CLEAR. ADD poses
are expressed in the planning (URDF root) frame; ATTACH poses in `attach_link`'s
frame.

### Visualization
World collision objects are published as a `visualization_msgs/MarkerArray` on
`~/collision_objects` (`/bimanual_manipulation_server/collision_objects`,
transient-local). Add a *MarkerArray* display in RViz to see them.

---

## MotionStep fields (type `2`, Cartesian)

| field | meaning |
|---|---|
| `reference_frame` | TF frame the target is expressed in (object, fixture, camera, …). Empty → see below. |
| `pose_target` | target pose; in `reference_frame` if set, else in the arm base frame. |
| `relative` + `offset` (+ `offset_in_tip_frame`) | when no `reference_frame`: offset from the current tip (tip or base frame). |
| `cartesian_path` | `true` = straight line of the tip (precise approach/retreat); `false` = IK once + joint interpolation (robust, fast reach). |
| `velocity_scaling` / `acceleration_scaling` | per-step speed (0 → group default). |
| `blend_radius` | sequences only: round the corner with the next same-group step over ~this many rad of joint space (0 = pass through). |

---

## Notes & limitations

* Motions are **straight lines** (joint or Cartesian). There is no obstacle
  *avoidance* planner — if the direct path is blocked the move is **rejected**
  (reported as a collision), it is not routed around the obstacle. This is the
  intended trade-off for speed; add intermediate waypoints / sequence steps to
  go around clutter.
* `collision.margin` uses an exact distance query, which is heavier than the
  plain contact test; with detailed meshes keep `mesh_decimation` on. Mesh
  decimation clusters vertices and can shrink a shape by ~one voxel — combine a
  small `margin` to stay conservative.
* SRDF `disable_collisions` reflects pairs MoveIt's random sampling never found
  colliding; some left↔right pairs may be disabled even though the two arms can
  physically meet. Drop those from the SRDF (or list them back via the code) if
  you need cross-arm collisions caught.
* World-object poses for ADD must be given in the root frame (no TF lookup is
  performed); ATTACH poses are in the link frame. DETACH simply removes the
  object.
* URDF `mimic` joints are treated as independent (defaulted to 0) for collision
  FK. List the relevant pairs in `disabled_pairs` if that ever matters.
* One goal is executed at a time; additional goals are rejected while busy.
