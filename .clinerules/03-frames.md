# Frame convention

The pose contract is always:

    X_camera = R * X_world + t

`Rotation` is OpenCV's Rodrigues vector. `Translation` is `t`; it is not camera
position. Camera center in world coordinates is:

    C_world = -R^T * t

Never rename or document `t` as camera location.
