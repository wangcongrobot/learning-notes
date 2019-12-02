 # _mjtCamera
CAMERA_FREE = 0
CAMERA_TRACKING = 1
CAMERA_FIXED = 2
CAMERA_USER = 3

    ctypedef enum mjtCamera:             # abstract camera type
        mjCAMERA_FREE        = 0,       # free camera
        mjCAMERA_TRACKING,              # tracking camera; uses trackbodyid
        mjCAMERA_FIXED,                 # fixed camera; uses fixedcamid
        mjCAMERA_USER                   # user is responsible for setting OpenGL camera
