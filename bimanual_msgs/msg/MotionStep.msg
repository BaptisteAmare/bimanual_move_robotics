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
# Absolute target pose of the tip, expressed in the planning frame.
geometry_msgs/Pose pose_target
# If relative is true, pose_target is ignored and "offset" is applied to the
# current tip pose instead (handy for approach / retreat moves).
bool relative
geometry_msgs/Vector3 offset
# If true, the offset is expressed in the tip frame (e.g. approach along the
# tool's +Z axis); otherwise it is expressed in the planning frame.
bool offset_in_tip_frame

# --- TYPE_GRIPPER ----------------------------------------------------------
# Target position of the gripper joint(s) (meters or radians, controller
# dependent). A named_target may also be used for grippers.
float64 gripper_position

# --- motion shaping (0 -> use the group's configured default) --------------
float64 velocity_scaling
float64 acceleration_scaling
