#include "../../cpp/opencv_calib3d_shim.h"
#include <stddef.h>
#include <stdint.h>
#include <math.h>

double calib3d_test_nonfinite(int32_t kind) {
    return kind == 0 ? NAN : (kind == 1 ? INFINITY : -INFINITY);
}

int32_t calib3d_test_intrinsics_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_camera_intrinsics), _Alignof(opencv_calib3d_camera_intrinsics),
        offsetof(opencv_calib3d_camera_intrinsics, focal_x),
        offsetof(opencv_calib3d_camera_intrinsics, focal_y),
        offsetof(opencv_calib3d_camera_intrinsics, center_x),
        offsetof(opencv_calib3d_camera_intrinsics, center_y)
    };
    return field >= 0 && field < 6 ? (int32_t)values[field] : -1;
}
int32_t calib3d_test_distortion_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_distortion5), _Alignof(opencv_calib3d_distortion5),
        offsetof(opencv_calib3d_distortion5, k1), offsetof(opencv_calib3d_distortion5, k2),
        offsetof(opencv_calib3d_distortion5, p1), offsetof(opencv_calib3d_distortion5, p2),
        offsetof(opencv_calib3d_distortion5, k3)
    };
    return field >= 0 && field < 7 ? (int32_t)values[field] : -1;
}
int32_t calib3d_test_pose_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_pose), _Alignof(opencv_calib3d_pose),
        offsetof(opencv_calib3d_pose, rx), offsetof(opencv_calib3d_pose, ry),
        offsetof(opencv_calib3d_pose, rz), offsetof(opencv_calib3d_pose, tx),
        offsetof(opencv_calib3d_pose, ty), offsetof(opencv_calib3d_pose, tz)
    };
    return field >= 0 && field < 8 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_intrinsics(opencv_calib3d_camera_intrinsics *v) {
    *v = (opencv_calib3d_camera_intrinsics){800.0, 820.0, 320.0, 240.0};
}
void calib3d_test_fill_distortion(opencv_calib3d_distortion5 *v) {
    *v = (opencv_calib3d_distortion5){0.1, -0.2, 0.01, -0.02, 0.03};
}
void calib3d_test_fill_pose(opencv_calib3d_pose *v) {
    *v = (opencv_calib3d_pose){0.1, 0.2, 0.3, 4.0, 5.0, 6.0};
}
