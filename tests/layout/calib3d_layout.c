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

int32_t calib3d_test_rotation_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_rotation_matrix), _Alignof(opencv_calib3d_rotation_matrix),
        offsetof(opencv_calib3d_rotation_matrix, m00),
        offsetof(opencv_calib3d_rotation_matrix, m01),
        offsetof(opencv_calib3d_rotation_matrix, m02),
        offsetof(opencv_calib3d_rotation_matrix, m10),
        offsetof(opencv_calib3d_rotation_matrix, m11),
        offsetof(opencv_calib3d_rotation_matrix, m12),
        offsetof(opencv_calib3d_rotation_matrix, m20),
        offsetof(opencv_calib3d_rotation_matrix, m21),
        offsetof(opencv_calib3d_rotation_matrix, m22)
    };
    return field >= 0 && field < 11 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_rotation(opencv_calib3d_rotation_matrix *v) {
    *v = (opencv_calib3d_rotation_matrix){1,2,3,4,5,6,7,8,9};
}

int32_t calib3d_test_homography_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_homography), _Alignof(opencv_calib3d_homography),
        offsetof(opencv_calib3d_homography, h00), offsetof(opencv_calib3d_homography, h01),
        offsetof(opencv_calib3d_homography, h02), offsetof(opencv_calib3d_homography, h10),
        offsetof(opencv_calib3d_homography, h11), offsetof(opencv_calib3d_homography, h12),
        offsetof(opencv_calib3d_homography, h20), offsetof(opencv_calib3d_homography, h21),
        offsetof(opencv_calib3d_homography, h22)
    };
    return field >= 0 && field < 11 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_homography(opencv_calib3d_homography *v) {
    *v = (opencv_calib3d_homography){1,2,3,4,5,6,7,8,9};
}
int32_t calib3d_test_homography_options_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_homography_options), _Alignof(opencv_calib3d_homography_options),
        offsetof(opencv_calib3d_homography_options, maximum_iterations),
        offsetof(opencv_calib3d_homography_options, reprojection_threshold_pixels),
        offsetof(opencv_calib3d_homography_options, confidence)
    };
    return field >= 0 && field < 5 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_homography_options(opencv_calib3d_homography_options *v) {
    *v = (opencv_calib3d_homography_options){2000,3.0,0.995};
}

int32_t calib3d_test_fundamental_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_fundamental), _Alignof(opencv_calib3d_fundamental),
        offsetof(opencv_calib3d_fundamental, f00), offsetof(opencv_calib3d_fundamental, f01),
        offsetof(opencv_calib3d_fundamental, f02), offsetof(opencv_calib3d_fundamental, f10),
        offsetof(opencv_calib3d_fundamental, f11), offsetof(opencv_calib3d_fundamental, f12),
        offsetof(opencv_calib3d_fundamental, f20), offsetof(opencv_calib3d_fundamental, f21),
        offsetof(opencv_calib3d_fundamental, f22)
    };
    return field >= 0 && field < 11 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_fundamental(opencv_calib3d_fundamental *v) {
    *v = (opencv_calib3d_fundamental){1,2,3,4,5,6,7,8,9};
}
int32_t calib3d_test_fundamental_options_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_fundamental_options), _Alignof(opencv_calib3d_fundamental_options),
        offsetof(opencv_calib3d_fundamental_options, epipolar_threshold_pixels),
        offsetof(opencv_calib3d_fundamental_options, confidence)
    };
    return field >= 0 && field < 4 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_fundamental_options(opencv_calib3d_fundamental_options *v) {
    *v = (opencv_calib3d_fundamental_options){3.0,0.99};
}

int32_t calib3d_test_essential_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_essential), _Alignof(opencv_calib3d_essential),
        offsetof(opencv_calib3d_essential,e00), offsetof(opencv_calib3d_essential,e01),
        offsetof(opencv_calib3d_essential,e02), offsetof(opencv_calib3d_essential,e10),
        offsetof(opencv_calib3d_essential,e11), offsetof(opencv_calib3d_essential,e12),
        offsetof(opencv_calib3d_essential,e20), offsetof(opencv_calib3d_essential,e21),
        offsetof(opencv_calib3d_essential,e22)
    };
    return field >= 0 && field < 11 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_essential(opencv_calib3d_essential *v) {
    *v = (opencv_calib3d_essential){1,2,3,4,5,6,7,8,9};
}
int32_t calib3d_test_relative_pose_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_relative_pose), _Alignof(opencv_calib3d_relative_pose),
        offsetof(opencv_calib3d_relative_pose,r00), offsetof(opencv_calib3d_relative_pose,r01),
        offsetof(opencv_calib3d_relative_pose,r02), offsetof(opencv_calib3d_relative_pose,r10),
        offsetof(opencv_calib3d_relative_pose,r11), offsetof(opencv_calib3d_relative_pose,r12),
        offsetof(opencv_calib3d_relative_pose,r20), offsetof(opencv_calib3d_relative_pose,r21),
        offsetof(opencv_calib3d_relative_pose,r22), offsetof(opencv_calib3d_relative_pose,tx),
        offsetof(opencv_calib3d_relative_pose,ty), offsetof(opencv_calib3d_relative_pose,tz)
    };
    return field >= 0 && field < 14 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_relative_pose(opencv_calib3d_relative_pose *v) {
    *v = (opencv_calib3d_relative_pose){1,2,3,4,5,6,7,8,9,10,11,12};
}
int32_t calib3d_test_essential_options_layout(int32_t field) {
    const size_t values[] = {
        sizeof(opencv_calib3d_essential_options), _Alignof(opencv_calib3d_essential_options),
        offsetof(opencv_calib3d_essential_options,normalized_epipolar_threshold),
        offsetof(opencv_calib3d_essential_options,confidence)
    };
    return field >= 0 && field < 4 ? (int32_t)values[field] : -1;
}
void calib3d_test_fill_essential_options(opencv_calib3d_essential_options *v) {
    *v = (opencv_calib3d_essential_options){0.001,0.999};
}
