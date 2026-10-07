#ifndef OPENCV_CALIB3D_SHIM_H
#define OPENCV_CALIB3D_SHIM_H

#include <stdint.h>

#if defined(_WIN32) && defined(OPENCV_CALIB3D_BUILDING)
# define OPENCV_CALIB3D_API __declspec(dllexport)
#elif defined(_WIN32)
# define OPENCV_CALIB3D_API __declspec(dllimport)
#else
# define OPENCV_CALIB3D_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct opencv_core_mat_handle opencv_core_mat_handle;
typedef struct opencv_calib3d_pose_result_handle opencv_calib3d_pose_result_handle;
typedef struct opencv_calib3d_homography_result_handle opencv_calib3d_homography_result_handle;
typedef struct opencv_calib3d_fundamental_result_handle opencv_calib3d_fundamental_result_handle;
typedef int32_t opencv_calib3d_status;

enum {
    OPENCV_CALIB3D_OK = 0,
    OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT = 1,
    OPENCV_CALIB3D_ERROR_OPENCV = 2,
    OPENCV_CALIB3D_ERROR_ALLOCATION = 3,
    OPENCV_CALIB3D_ERROR_STD = 4
};

typedef struct opencv_calib3d_camera_intrinsics {
    double focal_x, focal_y, center_x, center_y;
} opencv_calib3d_camera_intrinsics;

typedef struct opencv_calib3d_distortion5 {
    double k1, k2, p1, p2, k3;
} opencv_calib3d_distortion5;

typedef struct opencv_calib3d_pose {
    double rx, ry, rz, tx, ty, tz;
} opencv_calib3d_pose;

typedef struct opencv_calib3d_point3 {
    double x, y, z;
} opencv_calib3d_point3;

typedef struct opencv_calib3d_rotation_matrix {
    double m00, m01, m02;
    double m10, m11, m12;
    double m20, m21, m22;
} opencv_calib3d_rotation_matrix;

typedef struct opencv_calib3d_homography {
    double h00, h01, h02;
    double h10, h11, h12;
    double h20, h21, h22;
} opencv_calib3d_homography;

typedef struct opencv_calib3d_homography_options {
    int32_t maximum_iterations;
    double reprojection_threshold_pixels;
    double confidence;
} opencv_calib3d_homography_options;

typedef struct opencv_calib3d_fundamental {
    double f00, f01, f02;
    double f10, f11, f12;
    double f20, f21, f22;
} opencv_calib3d_fundamental;

typedef struct opencv_calib3d_fundamental_options {
    double epipolar_threshold_pixels;
    double confidence;
} opencv_calib3d_fundamental_options;

/* Shared exact native arithmetic profile, also used for Ada preflight. */
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_validate_fundamental_options(
    const opencv_calib3d_fundamental_options *options);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_find_fundamental_ransac(
    const opencv_core_mat_handle *first_points,
    const opencv_core_mat_handle *second_points,
    const opencv_calib3d_fundamental_options *options,
    opencv_calib3d_fundamental_result_handle **result);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_fundamental_result_found(
    const opencv_calib3d_fundamental_result_handle *result, uint8_t *found);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_fundamental_result_matrix(
    const opencv_calib3d_fundamental_result_handle *result, opencv_calib3d_fundamental *matrix);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_fundamental_result_inlier_count(
    const opencv_calib3d_fundamental_result_handle *result, int32_t *count);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_fundamental_result_inlier(
    const opencv_calib3d_fundamental_result_handle *result, int32_t index, int32_t *correspondence_index);
OPENCV_CALIB3D_API void opencv_calib3d_fundamental_result_destroy(
    opencv_calib3d_fundamental_result_handle *result);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_find_homography_ransac(
    const opencv_core_mat_handle *source_points,
    const opencv_core_mat_handle *destination_points,
    const opencv_calib3d_homography_options *options,
    opencv_calib3d_homography_result_handle **result);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_homography_result_found(
    const opencv_calib3d_homography_result_handle *result, uint8_t *found);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_homography_result_matrix(
    const opencv_calib3d_homography_result_handle *result, opencv_calib3d_homography *matrix);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_homography_result_inlier_count(
    const opencv_calib3d_homography_result_handle *result, int32_t *count);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_homography_result_inlier(
    const opencv_calib3d_homography_result_handle *result, int32_t index, int32_t *correspondence_index);
OPENCV_CALIB3D_API void opencv_calib3d_homography_result_destroy(
    opencv_calib3d_homography_result_handle *result);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_rotation_matrix_of(
    const opencv_calib3d_pose *pose, opencv_calib3d_rotation_matrix *matrix);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_undistort_normalized(
    const opencv_core_mat_handle *image_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    opencv_core_mat_handle *normalized_points);

OPENCV_CALIB3D_API const char *opencv_calib3d_last_error(void);
OPENCV_CALIB3D_API const char *opencv_calib3d_native_version(void);
OPENCV_CALIB3D_API const char *opencv_calib3d_native_backend(void);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_project_points(
    const opencv_core_mat_handle *object_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    const opencv_calib3d_pose *pose,
    opencv_core_mat_handle *image_points);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_camera_center(
    const opencv_calib3d_pose *pose,
    opencv_calib3d_point3 *center);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_solve_pnp_ransac(
    const opencv_core_mat_handle *object_points,
    const opencv_core_mat_handle *image_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    int32_t maximum_iterations,
    double reprojection_error_pixels,
    double confidence,
    opencv_calib3d_pose_result_handle **result);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_refine_pose_iterative(
    const opencv_core_mat_handle *object_points,
    const opencv_core_mat_handle *image_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    const opencv_calib3d_pose *initial_pose,
    uint8_t *refined,
    opencv_calib3d_pose *refined_pose);

OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_pose_result_found(
    const opencv_calib3d_pose_result_handle *result, uint8_t *found);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_pose_result_pose(
    const opencv_calib3d_pose_result_handle *result, opencv_calib3d_pose *pose);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_pose_result_inlier_count(
    const opencv_calib3d_pose_result_handle *result, int32_t *count);
OPENCV_CALIB3D_API opencv_calib3d_status opencv_calib3d_pose_result_inlier(
    const opencv_calib3d_pose_result_handle *result, int32_t index, int32_t *correspondence_index);
OPENCV_CALIB3D_API void opencv_calib3d_pose_result_destroy(
    opencv_calib3d_pose_result_handle *result);

#ifdef __cplusplus
}
#endif
#endif
