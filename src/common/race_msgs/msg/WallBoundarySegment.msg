# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
std_msgs/Header header

# Number inliers in point cloud of the wall boundary segment.
uint32 inliers_size 

# Wall segmentation plane coefficients.
float64 coefficient_a
float64 coefficient_b
float64 coefficient_c
float64 coefficient_d