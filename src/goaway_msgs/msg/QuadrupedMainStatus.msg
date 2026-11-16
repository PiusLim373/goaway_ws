# Mirrors /amcl_pose, so this is the MAP frame and is planar (z and roll/pitch are 0).
# Reads as the map origin until amcl first publishes. Bags recorded before 2026-09 carry
# the B2's own odom pose in this field instead.
geometry_msgs/Pose pose
int32 battery_percentage
string robot_mode
string navigation_state
bool is_paused
string localization_status
string current_map
# Mirrors /curr_navplan. 0 means no plan -- ids are epoch ms, so never 0.
int64 current_navplan_id
string current_navplan_status
# Which locomotion drives the robot: Unitree's own or the HTX policies (goaway_locomotion_node).
# Keeps the old value while a switch is running.
string locomotion_policy
# HTX walk (Velocity) only: its terrain scan is missing or stale, so it walks as if on flat ground.
bool height_scan_lost

# Enum navigation_state
string IDLE=idle
string NAVIGATING=navigating
string ERROR=error
string UNLOCALIZED=unlocalized
string CANCELED=canceled
string PAUSED=paused
# Distinct from ERROR: a goal that could not be reached, not a broken system.
string NAVIGATION_FAILED=navigation_failed
string MAP_SWITCHING=map_switching

# Enum robot_mode
string ROBOT_IDLE=idle
string BALANCE_STAND=balance_stand
string POSE=pose
string LOCOMOTION=locomotion
string LIE_DOWN=lie_down
string JOINT_LOCK=joint_lock
string DAMPING=damping
string RECOVERY_STAND=recovery_stand
string SIT=sit
string FRONT_FLIP=front_flip
string FRONT_JUMP=front_jump
string FRONT_POUNCE=front_pounce
# HTX only
string DONKEY_KICK=donkey_kick
string QUICK_BUCK=quick_buck
string SIDE_FLIP_LEFT=side_flip_left
string SIDE_FLIP_RIGHT=side_flip_right
string BACK_FLIP=back_flip
string UPRIGHT_WALK=upright_walk
string SHAKE_HAND_LEFT=shake_hand_left
string SHAKE_HAND_RIGHT=shake_hand_right
string UPRIGHT_RISING=upright_rising
string WAVE_LEFT=wave_left
string WAVE_RIGHT=wave_right
string UPRIGHT_ARM_BACK=upright_arm_back
string UPRIGHT_TIPPED=upright_tipped
string UPRIGHT_COMING_DOWN=upright_coming_down
# HTX sends no motor commands (Unitree's controller has the robot, or nothing does)
string OFF=off

# Enum localization_status
string NOT_STARTED=not_started
string PENDING=pending
string LOCALIZED=localized
string LOCALIZATION_LOST=localization_lost
string FAILED=failed

# Enum locomotion_policy
string UNITREE=unitree
string HTX=htx
