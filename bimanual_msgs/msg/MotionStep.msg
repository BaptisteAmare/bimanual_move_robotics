# A single manipulation primitive.
# Several MotionSteps chained together form a sequence such as
# approach -> grasp -> retreat.

# --- step type -------------------------------------------------------------
uint8 TYPE_NAMED=0      # go to a named pose of the group (joint space)
uint8 TYPE_JOINT=1      # go to explicit joint values (group order)
uint8 TYPE_CARTESIAN=2  # straight-line Cartesian move of the group tip
uint8 TYPE_GRIPPER=3    # actuate a gripper group
uint8 TYPE_FOLLOW=4     # continuously track a (possibly moving) TF frame

uint8 type

# Name of the move group this step acts on (see move_groups.yaml).
string group

# --- TYPE_NAMED ------------------------------------------------------------
string named_target

# --- TYPE_JOINT ------------------------------------------------------------
# Ordered exactly as the joints declared for the group.
float64[] joint_target

# --- TYPE_CARTESIAN --------------------------------------------------------
# Target pose of the tip.
#   * If reference_frame is set, pose_target is expressed in that TF frame
#     (a translation+rotation delta from the frame origin), and the server
#     transforms it into the arm's base frame via TF. This is the MoveIt-like
#     "pose target in frame_id" and lets you approach an EEF pose defined
#     relative to any frame (an object, a fixture, ...).
#   * Else if relative is true, pose_target is ignored and "offset" is applied
#     to the current tip pose (handy for approach / retreat moves).
#   * Else pose_target is taken directly in the arm's base frame.
geometry_msgs/Pose pose_target
string reference_frame

bool relative
geometry_msgs/Vector3 offset
# If true, the offset is expressed in the tip frame (e.g. approach along the
# tool's +Z axis); otherwise it is expressed in the arm's base frame.
bool offset_in_tip_frame

# Motion shape for TYPE_CARTESIAN:
#   true  -> straight line of the tip in Cartesian space (IK per waypoint),
#            precise for approach / retreat.
#   false -> solve IK for the goal pose once, then interpolate in joint space
#            (more robust and faster to reach an arbitrary pose).
bool cartesian_path

# --- TYPE_GRIPPER ----------------------------------------------------------
# Target position of the gripper joint(s) (meters or radians, controller
# dependent). A named_target may also be used for grippers.
float64 gripper_position

# --- motion shaping (0 -> use the group's configured default) --------------
float64 velocity_scaling
float64 acceleration_scaling

# Junction blending (sequences only): when > 0, the corner between this step
# and the next step of the SAME move group is rounded over roughly this radius
# in joint space [rad], instead of coming to the exact junction. Larger =
# rounder / more fluid, but deviates more from the via point. 0 = pass through.
float64 blend_radius

# --- TYPE_FOLLOW -----------------------------------------------------------
# Continuously servo the group tip to (reference_frame * pose_target), where
# pose_target is the delta in reference_frame (reuse reference_frame and
# pose_target above). Runs until a stop condition or until the action is
# canceled. At least one stop condition is recommended when used in a sequence.
float64 follow_rate                # servo rate [Hz], 0 -> default (20)
float64 follow_timeout             # stop after this long [s], 0 -> no timeout
float64 follow_position_tolerance  # "settled" distance to target [m], 0 -> off
float64 follow_settle_time         # time spent within tolerance to stop [s]
string follow_stop_topic           # std_msgs/Bool topic; a true msg stops it
