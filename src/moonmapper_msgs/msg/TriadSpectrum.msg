std_msgs/Header header

# Logical sensor name (e.g. "triad_sensor_1")
string sensor_name

# Entity/collision that was hit by the raycast (best-effort; may be empty)
string hit_name

# Discrete material label produced by lookup (e.g. "regolith", "basalt", "unknown")
string material_label

# 0..1 confidence for material_label (Level 1 heuristic)
float32 confidence

# Synthetic spectral channels (arbitrary units). Recommended: 18 channels.
float32[] channels

# Optional normalized channels in [0..1] (same length as channels when provided)
float32[] channels_normalized

