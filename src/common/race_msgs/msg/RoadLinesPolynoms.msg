builtin_interfaces/Time stamp

# The ID of the equipped vehicle
int16 ego_vhl_id

# Unique identifier of the line road markings sensor
int32 global_id

# Boolean value, true if the sent polynoms are noisy, false if not
uint8 is_noisy

int16 reference_frame

# Array of structures describing each line polynom,
# from left to the right of the ego-vehicle
RoadLinePolynom[<=3] road_lines_polynoms_array

# Time of update
float64 time_of_update
