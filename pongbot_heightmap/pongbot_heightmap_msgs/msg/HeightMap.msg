# Header (time and frame)
std_msgs/Header header

# map layer names.
string[] layers

# Resolution of the grid [m/cell].
float64 resolution

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] data 

# Value of Color
float32[] color