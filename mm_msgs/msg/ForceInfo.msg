Header header

# Raw force measurement, with bias removed. (To see biased raw force, just log
# directly from the sensor's topic.)
geometry_msgs/Vector3 force_raw

# Force after exponential smoothing has been applied.
geometry_msgs/Vector3 force_filtered

# (Filtered) force after having been transformed to the world frame.
geometry_msgs/Vector3 force_world

float64 force_desired

# True if contact has been made at any point in the past.
bool first_contact

# True if the EE is currently in contact.
bool contact
