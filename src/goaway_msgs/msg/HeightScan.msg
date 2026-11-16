# Height scan around the robot for a perceptive policy, from goaway_locomotion's height_map_node.
# The grid is the training's terrain scan: nx by ny points, resolution apart, centred on base_link
# and turned with its heading. Index k = iy * nx + ix (x forward fastest, then y to the left).

# Odom frame, stamp of the pose used
std_msgs/Header header
uint16 nx
uint16 ny
float32 resolution

# base_link height above the ground at each point (m), clamped to the training range
float32[] heights

# Where each value came from
uint8 SEEN = 0          # its own cell, seen within fresh_time
uint8 REMEMBERED = 1    # its own cell, seen earlier
uint8 NEIGHBOUR = 2     # the nearest known cell within fill_radius
uint8 FEET = 3          # nothing known nearby: the ground under the planted feet
uint8[] source

# False while odom is paused (skills) and until enough of the grid is known again
bool ready
