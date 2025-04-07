std_msgs/Header header
geometry_msgs/Vector3 position_ecef
geometry_msgs/Vector3 position_ecef_std
geometry_msgs/Vector3 orientation_hrp
geometry_msgs/Vector3 orientation_hrp_std
geometry_msgs/Vector3 velocity_ecef
geometry_msgs/Vector3 velocity_ecef_std

# Number_of_satellites
std_msgs/Int8 number_of_satellites

# Quality number of Inertial Explorer.
# Main quality indicator of the solution. 1 best, 6 worst.
# https://docs.novatel.com/Waypoint/Content/PDFs/Waypoint_Software_User_Manual_D100513.pdf (Page 72)
std_msgs/Int8 quality_indicator

# PDOP
# https://docs.novatel.com/Waypoint/Content/PDFs/Waypoint_Software_User_Manual_D100513.pdf (Page 112)
# Low PDOP (e.g., < 4): Indicates a strong satellite geometry, where satellites are well-distributed, leading to higher accuracy and faster position fixes. 
# High PDOP (e.g., > 7): Suggests weak geometry, with satellites clustered in the sky, resulting in lower accuracy and potentially slower fixes. 
std_msgs/Float32 position_dilution_of_precision

# Course Over Ground
# https://docs.novatel.com/Waypoint/Content/PDFs/Waypoint_Software_User_Manual_D100513.pdf (Page 86)
std_msgs/Float32 course_over_ground

# Fixed or Float ambiguity_status
# https://docs.novatel.com/Waypoint/Content/PDFs/Waypoint_Software_User_Manual_D100513.pdf (Page 112)
uint16 FIXED = 1
uint16 FLOAT = 2
std_msgs/UInt16 ambiguity_status