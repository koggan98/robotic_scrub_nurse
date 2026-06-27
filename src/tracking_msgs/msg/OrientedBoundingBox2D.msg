# 2D Oriented Bounding Box from OBB detection model
# All coordinates in pixel space

geometry_msgs/Point32[4] corners   # Four corner points (x,y in pixels, z unused)
float32 center_x                   # Center x in pixels
float32 center_y                   # Center y in pixels
float32 width                      # Box width in pixels
float32 height                     # Box height in pixels
float32 angle_rad                  # Rotation angle in radians
