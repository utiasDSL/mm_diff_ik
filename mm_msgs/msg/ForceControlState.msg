Header header

# Raw force measurement, with bias removed. (To see biased raw force, just log
# directly from the sensor's topic.)
geometry_msgs/Vector3 force_raw

# Force after exponential smoothing has been applied.
geometry_msgs/Vector3 force_filtered

# (Filtered) force after having been transformed to the world frame.
geometry_msgs/Vector3 force_world

# Position offset output by the force controller.
geometry_msgs/Vector3 position_offset
