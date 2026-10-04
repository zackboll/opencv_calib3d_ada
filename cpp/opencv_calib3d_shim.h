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
