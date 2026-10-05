#include "opencv_calib3d_shim.h"
#include <stddef.h>
_Static_assert(sizeof(opencv_calib3d_camera_intrinsics) == 4*sizeof(double), "intrinsics size");
_Static_assert(sizeof(opencv_calib3d_distortion5) == 5*sizeof(double), "distortion size");
_Static_assert(sizeof(opencv_calib3d_pose) == 6*sizeof(double), "pose size");
_Static_assert(offsetof(opencv_calib3d_pose, tx) == 3*sizeof(double), "pose ordering");
_Static_assert(sizeof(opencv_calib3d_rotation_matrix) == 9*sizeof(double), "rotation matrix size");
_Static_assert(offsetof(opencv_calib3d_rotation_matrix, m22) == 8*sizeof(double), "rotation ordering");
int main(void) { return OPENCV_CALIB3D_OK; }
