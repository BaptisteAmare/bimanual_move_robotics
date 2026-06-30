# A single manipulation primitive.
# Several MotionSteps chained together form a sequence such as
# approach -> grasp -> retreat.

# --- step type -------------------------------------------------------------
uint8 TYPE_NAMED=0      # go to a named pose of the group (joint space)
uint8 TYPE_JOINT=1      # go to explicit joint values (group order)
uint8 TYPE_CARTESIAN=2  # straight-line Cartesian move of the group tip
uint8 TYPE_GRIPPER=3    # actuate a gripper group

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
