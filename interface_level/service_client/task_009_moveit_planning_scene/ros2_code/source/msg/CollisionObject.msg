# Simplified version of moveit_msgs/CollisionObject
std_msgs/Header header
string id

# The object type (for database)
# object_recognition_msgs/ObjectType type

# Solid primitives and their poses
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# Meshes and their poses
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# Planes and their poses
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses

# Subframe names and poses
string[] subframe_names
geometry_msgs/Pose[] subframe_poses

# Operation to perform
uint8 ADD=0
uint8 REMOVE=1
uint8 APPEND=2
uint8 MOVE=3
uint8 operation

# Pose of the object
geometry_msgs/Pose pose