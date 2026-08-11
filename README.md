# bimanual_move_robotics

Lightweight **bimanual manipulation** stack for humanoid robots on **ROS 2
Humble**. It plugs onto any `robot_description`, talks directly to the robot's
`FollowJointTrajectory` / `GripperCommand` controllers, and exposes ROS actions
to run a single move or a chain of moves (*approach → grasp → retreat*, …).

It deliberately **does not use MoveIt's planning pipeline**: kinematics use
**KDL**, collision checking uses **FCL**, and paths are dense joint/Cartesian
interpolation. The result is fast to compute and fast to execute, while still
preventing the robot from colliding **with itself** or with **collision
objects** you add at runtime.

Everything robot-specific — move groups, joint/controller names, named poses,
collision rules, sequences — lives in **YAML**, not in the code.

---

## Contents
- [Packages](#packages)
- [How it works](#how-it-works)
- [Build](#build)
- [Run](#run)
- [Configuration](#configuration)
- [Motion types](#motion-types) — the seven `MotionStep` primitives
- [ROS interface](#ros-interface) — `~/move`, `~/execute_sequence`, collision objects
- [Sequences](#sequences) — chaining, blending, timing
- [Notes & limitations](#notes--limitations)

---

## Packages

| Package | Description |
|---|---|
| `bimanual_msgs` | Action / message / service definitions. |
| `bimanual_manipulation` | The C++ engine and the `manipulation_server_node`. |

## How it works

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
  Levenberg-Marquardt IK (full-pose or position-only).
* **Collision** (`collision_model.cpp`) — an FCL world built from the URDF
  collision geometry. A URDF FK walker places every link, so a full
  configuration is validated in `O(num_links)`. Self-collision uses an
  SRDF-style allowed-pairs list; world objects are added/removed at runtime.
  Two modes: exact **meshes** (FCL) or fast analytic **spheres**.
* **Trajectory generation** (`trajectory_generator.cpp`) — dense interpolation
  in joint or Cartesian space, **every waypoint collision-checked**, then
  velocity/acceleration-limited time-parameterization.
* **Move groups** (`move_group.cpp`) — own the controller action clients and
  split one trajectory across several controllers so **both arms move at once**.
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
colcon build --packages-select bimanual_msgs bimanual_manipulation --symlink-install
source install/setup.bash
```

> `--symlink-install` lets you edit the YAML configs without rebuilding. After
> changing a `.msg`/`.action`/`.srv`, rebuild **both** packages (messages first;
> `--packages-select` orders them for you).

---

## Run

The server needs the URDF and the controllers up (`ros2_control` with a
`FollowJointTrajectory` controller per arm and, optionally, a `GripperCommand`
controller per gripper).

**One robot = one folder.** Everything robot-specific lives in `config/<robot>/`
with standard file names, so switching robots is a **single argument**:

```bash
ros2 launch bimanual_manipulation bimanual_manipulation.launch.py robot:=walker_s2
ros2 launch bimanual_manipulation bimanual_manipulation.launch.py robot:=genie
```

`robot:=<name>` reads `config/<name>/`:

| File | Role |
|---|---|
| `move_groups.yaml`, `named_poses.yaml`, `collision.yaml`, `sequences.yaml` | required configs |
| `model.srdf` | optional — the SRDF ACM, used if present |
| `model.urdf` | optional — a full-geometry URDF, used if present, else the `/robot_description` topic is used |

To add a robot, copy an existing folder, drop in its `model.urdf`/`model.srdf`,
and edit the four YAMLs. `config/walker_s2/` and `config/genie/` are included as
references. Overrides remain available: `config_dir:=<abs path>` points outside
the package, and any single file can be replaced (`move_groups_config:=...`,
`srdf_config:=...`, `robot_description_file:=...`, `robot_description:=...`).

**Where the URDF comes from** (priority order):
1. `robot_description` parameter (a URDF string), else
2. `robot_description_file` — a URDF **file** on disk, else
3. the `/robot_description` topic (default).

Use `robot_description_file` when the live `/robot_description` is a
kinematics-only URDF with no `<collision>`/`<visual>` geometry — collision
checking needs the geometry, so point it at the full description (with meshes).
Joint names must match the controllers / `/joint_states`.

**SRDF (`srdf_config`).** A MoveIt `.srdf`'s `disable_collisions` pairs are
merged into the allowed-collision matrix, exactly like MoveIt (adjacent /
never-colliding links are skipped). Pairs referencing links absent from the URDF
are ignored. Loaded by default from the packaged SRDF; pass `srdf_config:=""` to
disable.

**Startup log** — glance at the summary:

```
Loaded N disabled collision pairs from SRDF ...
Collision model (spheres): P spheres, Q link pairs checked (R auto-disabled ...)
    auto-disabled: <link_a> <-> <link_b>          # the structural pairs dropped
Meshes: 35 loaded, 0 failed, ...
[WARN] X link(s) have NO collision geometry ...   # if any
```

---

## Configuration

Four YAML files (+ the SRDF) define everything robot-specific. All are heavily
commented under `bimanual_manipulation/config/`.

| File | Defines |
|---|---|
| `move_groups.yaml` | groups: joints, `base_link`/`tip_link`, controllers, and the global `planning:` defaults (speeds, RRT, `acceleration_limiting`). A group is *Cartesian-capable* once it has `base_link`+`tip_link`. Optional `locked_joints:` — see below. |
| `named_poses.yaml` | joint-space targets per group (first value of a gripper pose = the gripper command). |
| `collision.yaml` | collision-checking settings (see below). |
| `sequences.yaml` | named lists of steps, called by name via `~/execute_sequence`. |

### `collision.yaml`

| Key | Meaning |
|---|---|
| `enabled` | master on/off. |
| `mode` | `mesh` (FCL on decimated meshes, exact) or `spheres` (each link approximated by spheres, checked analytically — much faster, slightly conservative). |
| `sphere_voxel` / `sphere_radius_scale` | sphere resolution [m] and inflate/deflate factor (spheres mode). |
| `margin` | separation kept from everything, **meshes included** (`0` = contact test, `0.02`–`0.05` = safety gap). |
| `padding` | primitive-only inflation (no effect on meshes — use `margin`). |
| `mesh_decimation` | grid size [m] simplifying meshes at load (big speedup; `0` = full res). |
| `resolution` | joint-space step between collision-checked waypoints. |
| `auto_disable` (+ `auto_disable_samples`) | at startup, drop link pairs in collision in the **default pose** or **all** sampled configs (permanent structural contacts, like MoveIt's "Default"/"Always"). Listed at startup. |
| `self_chain_distance` | skip self-collision between links within N joints on the same chain. `1` = adjacent only; `2` also skips a link vs its grandparent (elbow↔wrist) — clears sphere false positives along an arm. |
| `disabled_pairs` | extra link pairs to ignore (on top of the SRDF, auto-disable and chain-distance skip). |

**Tuning false positives (spheres mode).** If a legitimate pose is rejected, the
error names the colliding pair. In order: raise `self_chain_distance` to `2`,
lower `sphere_radius_scale` (`0.85`), then add the specific pair to
`disabled_pairs`.

### Locked joints (`move_groups.yaml`)

A joint can be **on** a group's kinematic chain (so it must be listed in
`joints`) yet held **fixed** during Cartesian IK. List it under `locked_joints`:
the IK keeps it exactly at its current value while the other joints reach the
target. This is how you keep, say, the waist *pitch* still while the waist *yaw*
and the arm move (pitch sits between yaw and the arm, so it cannot simply be
dropped from the chain). Locked joints are still commanded — to hold position.

```yaml
  right_arm_with_torso:
    base_link: base_link                 # upstream of the waist
    tip_link: R_industrial_hand_contact_frame
    joints: [waist_yaw_joint, waist_pitch_joint, R_shoulder_pitch_joint, ... , R_wrist_roll_joint]
    locked_joints: [waist_pitch_joint]   # on the chain, but held fixed
    controllers:
      - {name: /right_arm_controller/follow_joint_trajectory, joints: [R_shoulder_pitch_joint, ...]}
      - {name: /waist_controller/follow_joint_trajectory, joints: [waist_yaw_joint, waist_pitch_joint]}
```

**`assist_joints`** — free joints (e.g. a waist *yaw*) that should stay put when
the rest of the chain can reach the target on its own, and only move when it
cannot. Without this, the extra DoF makes the IK pick a worse arm branch and lose
reach; with it, the arm does the work (matching the arm-only group) and the yaw
only kicks in for targets the arm can't reach alone:

```yaml
    joints: [waist_yaw_joint, waist_pitch_joint, R_shoulder_pitch_joint, ... ]
    locked_joints: [waist_pitch_joint]   # never moves
    assist_joints: [waist_yaw_joint]     # moves only when the arm can't reach
```

Under the hood, groups with `locked_joints`/`assist_joints` run KDL's LMA on a
**reduced chain** (the held joints baked in as fixed segments at their current
value); it tries the arm-only solve first, then frees the assist joints, and
honors `free_orientation` and collision-aware IK.

---

## Motion types

A `MotionStep` is one primitive. `~/move` runs a single step; `~/execute_sequence`
runs a list. `type` selects the behavior:

| `type` | Name | Purpose |
|---|---|---|
| `0` | Named | go to a named joint pose of the group |
| `1` | Joint | go to explicit joint values |
| `2` | Cartesian | move the tip to a pose / by an offset (single arm **or** dual-arm coordinated) |
| `3` | Gripper | actuate a gripper group |
| `4` | Follow | continuously track a (moving) TF frame |
| `5` | Hold-tip | drive a joint (e.g. waist) while an arm holds the tip fixed |
| `6` | Collision | add/remove/attach objects or toggle checking (barrier step) |

Joint-space moves (`0`, `1`, and `2` with `cartesian_path:false`) try a straight
line first and **fall back to RRT-Connect** around obstacles if blocked (see
[Notes](#notes--limitations)).

### `0` Named / `1` Joint
`named_target` selects a pose from `named_poses.yaml`; `joint_target` gives
explicit values in the group's joint order.

```bash
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 0, group: left_arm, named_target: ready}}"
```

### `2` Cartesian (single arm)
Moves the group tip. How the goal pose is resolved:

| Field | Meaning |
|---|---|
| `reference_frame` | TF frame the target is expressed in (object, fixture, camera…). Set → `pose_target` is taken in this frame. |
| `pose_target` | target pose (in `reference_frame` if set, else in the arm's base frame). |
| `relative` + `offset` (+ `offset_in_tip_frame`) | if no `reference_frame`: offset from the current tip, in the tip frame (`offset_in_tip_frame:true`) or base frame. |
| `cartesian_path` | `true` = straight line of the tip (precise approach/retreat, never re-routed); `false` = IK once + joint interpolation (robust, fast reach). |
| `free_orientation` | `false` (default) holds the tip orientation; `true` tracks only the **position** and lets orientation drift — extends reach when a constrained arm (e.g. with `locked_joints`) cannot hold the full pose along the whole move. |
| `velocity_scaling` / `acceleration_scaling` | per-step speed (`0` → group default). |

```bash
# approach: move the left tool 10 cm down its own Z axis, in a straight line
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 2, group: left_arm, relative: true, offset_in_tip_frame: true,
           offset: {x: 0.0, y: 0.0, z: -0.10}, cartesian_path: true, velocity_scaling: 0.15}}"

# pose target 15 cm above the TF frame 'target_object' (IK + joint interpolation)
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 2, group: left_arm, reference_frame: target_object,
           pose_target: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {w: 1.0}},
           cartesian_path: false}}"
```

### `2` Cartesian (dual-arm coordinated)
A group with `cartesian_subgroups` (e.g. `both_arms: {cartesian_subgroups:
[left_arm, right_arm]}`) moves **both** tips at once, sampled together so the
arms stay in sync (one trajectory split across both controllers). Orientation is
kept; pure straight-line translation of each tip.

**Frame.** The offset is expressed in each subgroup's **`base_link`** — the
shared arm base (`arm_base_link` on Genie, `torso_link` on Walker) — or in each
subgroup's **tip frame** when `offset_in_tip_frame: true`.

* **`offset: [dx,dy,dz]`** — the *same* vector for both tips → the hands keep
  their relative pose, a two-handed grasp translates **rigidly** (carry).
* **`offsets: [[lx,ly,lz],[rx,ry,rz]]`** — one offset **per subgroup** (same
  order as `cartesian_subgroups`) → the hands move **independently in one
  simultaneous step**, e.g. spread/close.

```bash
# carry: lift an object held in both hands by 15 cm (rigid)
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 2, group: both_arms, offset: {x: 0.0, y: 0.0, z: 0.15}}}"
```
```yaml
# spread the hands apart by 5 cm each, both arms at once (in a sequence):
- {type: cartesian, group: both_arms, offsets: [[0.0, 0.05, 0.0], [0.0, -0.05, 0.0]]}
```

This replaces two separate `left_arm` / `right_arm` steps (which would run
**sequentially** — a group change is a barrier) with one **simultaneous** step.

### `3` Gripper
Actuates every gripper controller of the group. `named_target` (from
`named_poses.yaml`) or `gripper_position`.

```bash
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 3, group: left_gripper, named_target: closed}}"
```

### `4` Follow (track a moving frame)
Continuously servos the tip to `reference_frame * pose_target` (the delta is
`pose_target`, in the frame), tracking it as it moves. Runs as a `~/move` goal
(cancel to stop) or in a sequence, where it is a **barrier** (the sequence
resumes once it stops). Each cycle it looks up TF, solves IK, collision-checks
the target (holds if blocked) and streams a short trajectory. Requires a
Cartesian-capable group. Set at least one stop condition in a sequence:

| Field | Meaning |
|---|---|
| `follow_rate` | servo rate [Hz] (`0` → 20). |
| `follow_timeout` | stop after this long [s] (`0` → none). |
| `follow_position_tolerance` + `follow_settle_time` | stop once the tip stays within the tolerance [m] for that long [s] (target reached/stopped). |
| `follow_stop_topic` | a `std_msgs/Bool` topic; a `true` message stops it. |

```bash
# keep the tip 10 cm above 'moving_target' until canceled (Ctrl-C) or settled
ros2 action send_goal -f /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 4, group: left_arm, reference_frame: moving_target,
           pose_target: {position: {x: 0.0, y: 0.0, z: 0.10}, orientation: {w: 1.0}},
           follow_position_tolerance: 0.03, follow_settle_time: 1.0}}"
```

### `5` Hold-tip (drive a joint, keep the hand fixed)
For a group that declares `driven_joints` and a `compensating_subgroup` (e.g.
`waist_right_arm`: waist joint + right arm, `compensating_subgroup: right_arm`).
`joint_target` gives the target for the driven joints; they are ramped from their
current value while the subgroup is IK-solved every step to keep `tip_link` fixed
in `base_link` — **turn the waist while the hand stays on its object**, the arm
absorbing the motion. Errors clearly if the tip becomes unreachable or collides.

* **Several driven joints** are supported — list them all in `driven_joints`
  (each must be upstream of the subgroup's `base_link`) and give one value each
  in `joint_target`. The whole set is ramped together while the tip is held.
* **Relative vs absolute** — `driven_absolute:false` (default) treats
  `joint_target` as deltas from the current value; `driven_absolute:true` treats
  them as absolute joint positions. In a sequence use `deltas: [...]` or
  `targets: [...]` (targets implies absolute).

* `base_link` = the (fixed) frame the hand is held in; it **must be upstream of
  the driven joints** (use the robot root to hold the hand fixed in the world).
* `tip_link` = the held frame — use the frame actually attached to the object
  (e.g. the hand contact frame).
* `free_orientation` — `false` (default) holds the full pose; `true` holds only
  the **position** and lets orientation drift, giving the arm much more room when
  the full-pose hold hits a limit/singularity.

```bash
# rotate the waist 0.3 rad, holding the right hand pose fixed
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 5, group: waist_right_arm, joint_target: [0.3]}}"

# same but hold position only (bigger reachable range)
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 5, group: waist_right_arm, joint_target: [0.5], free_orientation: true}}"

# several driven joints to ABSOLUTE positions, holding the tip fixed
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 5, group: torso_right_arm, driven_absolute: true,
           joint_target: [0.2, 0.4, -0.3, 0.0, 0.5]}}"
```

### `6` Collision (manage objects / toggle checking)
A group-less **barrier** step: it manages the collision world (or turns checking
on/off) *between* motions of a sequence, so you can e.g. add a table, move to
grasp, attach the grasped box to the hand, then move away with the box now part
of the robot. `collision_op` selects the action:

| `op` | effect | fields used |
|------|--------|-------------|
| `add` | add a free world object at `pose_target` (planning root frame) | `id`, `primitive`, `position`, `orientation` |
| `attach` | attach an object to a link; `pose_target` is in that link frame, and it rides with the link | `id`, `primitive`, `attach_link`, `position`, `orientation` |
| `remove` / `detach` | remove the object named `id` | `id` |
| `clear` | remove every world / attached object | — |
| `enable` / `disable` | turn collision checking on / off globally | — |

The same actions are available at runtime through the
[`~/manage_collision_object`](#manage_collision_object--bimanual_msgssrvmanagecollisionobject)
service; the step type just lets you script them inline in a sequence.

```bash
# add a 20 cm box 60 cm in front of the robot (unitary)
ros2 action send_goal /bimanual_manipulation_server/move bimanual_msgs/action/Move \
  "{step: {type: 6, collision_op: add, collision_id: table,
           collision_primitive: {type: 1, dimensions: [0.2, 0.2, 0.2]},
           pose_target: {position: {x: 0.6, y: 0.0, z: 0.1}, orientation: {w: 1.0}}}}"
```

In a `sequences.yaml` step (`type: collision`), `primitive`, `position` and
`orientation` ([x,y,z,w]) are given as short lists:

```yaml
- {type: collision, op: add, id: table,
   primitive: {type: box, dimensions: [0.2, 0.2, 0.2]}, position: [0.6, 0.0, 0.1]}
- {type: cartesian, group: right_arm, relative: true, offset: {z: -0.1}}   # approach
- {type: gripper, group: right_gripper, gripper_position: 0.0}             # grasp
- {type: collision, op: attach, id: box, attach_link: R_wrist_roll_link,
   primitive: {type: box, dimensions: [0.05, 0.05, 0.15]}, position: [0.0, 0.0, 0.08]}
- {type: cartesian, group: right_arm, relative: true, offset: {z: 0.15}}   # lift with box
- {type: collision, op: disable}      # temporarily skip checking
- {type: collision, op: detach, id: box}
```

`primitive.type` accepts the names `box` / `sphere` / `cylinder` (or the numeric
`1` / `2` / `3`). Box dimensions are `[x, y, z]`, sphere `[radius]`, cylinder
`[height, radius]`.

---

## ROS interface

### `~/move` — `bimanual_msgs/action/Move`
Runs one `MotionStep` (see [Motion types](#motion-types) for the `type` values
and fields). `-f` streams feedback.

### `~/execute_sequence` — `bimanual_msgs/action/ExecuteSequence`
Runs a predefined sequence (by `sequence_name`, from `sequences.yaml`) and/or
inline `steps`. In YAML a step's `type` is the name: `named`, `joint`,
`cartesian`, `gripper`, `follow`, `hold_tip`, `collision`.

```bash
ros2 action send_goal -f /bimanual_manipulation_server/execute_sequence \
  bimanual_msgs/action/ExecuteSequence "{sequence_name: both_ready}"
```

### `~/manage_collision_object` — `bimanual_msgs/srv/ManageCollisionObject`
Add / remove / attach world collision objects, checked on every motion.

```bash
# add a 20 cm box, 60 cm in front of the robot
ros2 service call /bimanual_manipulation_server/manage_collision_object \
  bimanual_msgs/srv/ManageCollisionObject \
  "{operation: 0, id: table_box,
    primitive: {type: 1, dimensions: [0.2, 0.2, 0.2]},
    pose: {header: {frame_id: ''}, pose: {position: {x: 0.6, y: 0.0, z: 0.1},
           orientation: {w: 1.0}}}}"
```

`operation`: `0` ADD, `1` REMOVE, `2` ATTACH, `3` DETACH, `4` CLEAR. ADD poses
are in the planning (URDF root) frame; ATTACH poses in `attach_link`'s frame.

**Visualization.** World objects are published as a
`visualization_msgs/MarkerArray` on `~/collision_objects` (transient-local). Add
a *MarkerArray* display in RViz to see them.

---

## Sequences

Consecutive steps on the **same** move group are concatenated into a **single
continuous trajectory** — the arm does not stop at each step's end, velocities
flow through the via points. A gripper, follow, hold-tip, or group change is a
**barrier** that closes the current trajectory. The combined trajectory uses the
**slowest** scaling among its steps.

Timing and path shape are controlled **independently**:

* **Timing** — `acceleration_limiting` (in `move_groups.yaml` `planning:`).
  `false` (example default) = **constant velocity-limited** speed, no slowdown at
  corners; `true` smooths acceleration (slows near sharp corners, gentler on
  hardware).
* **Path** — per-step `blend_radius` (rad of joint space, `0` = off). Rounds the
  corner with the next same-group step for a more human-like path; re-validated
  for collisions and shrunk/dropped if it would hit something.

So a rounded corner **at constant velocity** is just `blend_radius > 0` with
`acceleration_limiting: false`. See `wave_left`, `carry_up_and_forward` and
`track_object` in `sequences.yaml`.

---

## Notes & limitations

* **Obstacle avoidance.** Joint-space moves try a straight line first and fall
  back to **RRT-Connect** around obstacles (self + world), controlled by
  `planning.avoid_obstacles` (default true), `rrt_step`, `rrt_edge_resolution`
  and `rrt_max_iterations`. You only pay planning time when a detour is needed.
* **Cartesian straight lines** (`cartesian_path:true`, and dual-arm coordinated
  moves) are never re-routed — a blocked one is **rejected**. Use them for
  precise approach/retreat, not for traversing clutter.
* `collision.margin` uses an exact distance query (heavier than the contact
  test); keep `mesh_decimation` on with detailed meshes. Decimation can shrink a
  shape by ~one voxel — combine a small `margin` to stay conservative.
* SRDF `disable_collisions` reflects pairs MoveIt never found colliding; some
  left↔right pairs may be disabled even though the arms can physically meet. Drop
  those from the SRDF if you need cross-arm collisions caught.
* World-object ADD poses are in the root frame (no TF lookup); ATTACH poses are
  in the link frame.
* URDF `mimic` joints are treated as independent (defaulted to 0) for collision
  FK. List affected pairs in `disabled_pairs` if it matters.
* One goal runs at a time; additional goals are rejected while busy.
