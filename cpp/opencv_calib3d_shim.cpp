#include "opencv_calib3d_shim.h"
#include "pnp_profile.hpp"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR == 4
# include <opencv2/calib3d.hpp>
#elif CV_VERSION_MAJOR == 5
# include <opencv2/geometry/3d.hpp>
#else
# error "Review Calib3D compatibility before using another OpenCV major version"
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#if (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 1) || \
    (CV_VERSION_MAJOR == 5 && CV_VERSION_MINOR != 0)
# error "Review Calib3D compatibility outside OpenCV 4.1-4.x and 5.0.x"
#endif

static_assert(sizeof(int) == 4, "OpenCV native integer contract requires 32-bit int");
static_assert(std::is_standard_layout<opencv_calib3d_camera_intrinsics>::value,
              "camera intrinsics must have standard C layout");
static_assert(std::is_standard_layout<opencv_calib3d_distortion5>::value,
              "distortion must have standard C layout");
static_assert(std::is_standard_layout<opencv_calib3d_pose>::value,
              "pose must have standard C layout");

struct opencv_calib3d_pose_result_handle {
    bool found = false;
    opencv_calib3d_pose pose{};
    std::vector<int32_t> inliers;
};

namespace {
thread_local char error_text[1024] = "";

#ifdef OPENCV_CALIB3D_TEST_HOOKS
thread_local int failure_stage = 0;
thread_local int failure_kind = 0;
void checkpoint(int stage) {
    if (failure_stage != stage) return;
    const int kind = failure_kind;
    failure_stage = failure_kind = 0;
    switch (kind) {
    case 1: throw std::invalid_argument("injected invalid argument");
    case 2: throw cv::Exception(cv::Error::StsError, "injected OpenCV exception",
                                "checkpoint", __FILE__, __LINE__);
    case 3: throw std::bad_alloc();
    case 4: throw std::runtime_error("injected standard exception");
    default: throw 0;
    }
}
#else
void checkpoint(int) noexcept {}
#endif

void set_error(const char *message) noexcept {
    if (message == nullptr) message = "unknown error";
    std::snprintf(error_text, sizeof(error_text), "%s", message);
}

void require(bool condition, const char *message) {
    if (!condition) throw std::invalid_argument(message);
}

template <typename F>
opencv_calib3d_status guarded(F &&operation) noexcept {
    error_text[0] = '\0';
    try {
        operation();
        return OPENCV_CALIB3D_OK;
    } catch (const std::invalid_argument &error) {
        set_error(error.what());
        return OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT;
    } catch (const cv::Exception &error) {
        set_error(error.what());
        return OPENCV_CALIB3D_ERROR_OPENCV;
    } catch (const std::bad_alloc &error) {
        set_error(error.what());
        return OPENCV_CALIB3D_ERROR_ALLOCATION;
    } catch (const std::exception &error) {
        set_error(error.what());
        return OPENCV_CALIB3D_ERROR_STD;
    } catch (...) {
        set_error("unknown native exception");
        return OPENCV_CALIB3D_ERROR_STD;
    }
}

bool finite_pose(const opencv_calib3d_pose &pose) noexcept {
    return opencv_calib3d_detail::finite(pose.rx) && opencv_calib3d_detail::finite(pose.ry) &&
           opencv_calib3d_detail::finite(pose.rz) && opencv_calib3d_detail::finite(pose.tx) &&
           opencv_calib3d_detail::finite(pose.ty) && opencv_calib3d_detail::finite(pose.tz);
}

void validate_intrinsics(const opencv_calib3d_camera_intrinsics *value) {
    require(value != nullptr, "null camera intrinsics");
    require(opencv_calib3d_detail::intrinsics_fit(value->focal_x, value->focal_y,
                                                  value->center_x, value->center_y),
            "camera intrinsics must be finite with positive focal lengths");
}

void validate_distortion(const opencv_calib3d_distortion5 *value) {
    require(value != nullptr, "null distortion coefficients");
    require(opencv_calib3d_detail::distortion5_fit(value->k1, value->k2, value->p1,
                                                   value->p2, value->k3),
            "distortion coefficients must be finite");
}

void validate_pose(const opencv_calib3d_pose *value) {
    require(value != nullptr && finite_pose(*value), "pose must contain six finite components");
}

cv::Mat camera_matrix(const opencv_calib3d_camera_intrinsics &value) {
    return (cv::Mat_<double>(3, 3) << value.focal_x, 0.0, value.center_x,
                                     0.0, value.focal_y, value.center_y,
                                     0.0, 0.0, 1.0);
}

cv::Mat distortion_vector(const opencv_calib3d_distortion5 &value) {
    return (cv::Mat_<double>(5, 1) << value.k1, value.k2, value.p1, value.p2, value.k3);
}

cv::Mat rotation_vector(const opencv_calib3d_pose &value) {
    return (cv::Mat_<double>(3, 1) << value.rx, value.ry, value.rz);
}

cv::Mat translation_vector(const opencv_calib3d_pose &value) {
    return (cv::Mat_<double>(3, 1) << value.tx, value.ty, value.tz);
}

const cv::Mat &resolve_input(const opencv_core_mat_handle *handle) {
    require(handle != nullptr, "null Core input handle");
    const cv::Mat *mat = nullptr;
    require(opencv_core_module_input_mat(handle, &mat) == OPENCV_CORE_OK && mat != nullptr,
            "Core rejected input Mat handle");
    return *mat;
}

cv::Mat &resolve_output(opencv_core_mat_handle *handle) {
    require(handle != nullptr, "null Core output handle");
    cv::Mat *mat = nullptr;
    require(opencv_core_module_output_mat(handle, &mat) == OPENCV_CORE_OK && mat != nullptr,
            "Core rejected output Mat handle");
    return *mat;
}

int validate_object_points(const cv::Mat &mat, bool allow_empty) {
    if (mat.empty()) {
        require(allow_empty, "object points must be nonempty");
        return 0;
    }
    require(mat.dims == 2 && mat.type() == CV_64FC3 && mat.cols == 1 && mat.rows > 0,
            "object points must be Nx1 Float64 C3");
    return mat.rows;
}

int validate_image_points(const cv::Mat &mat, bool allow_empty) {
    if (mat.empty()) {
        require(allow_empty, "image points must be nonempty");
        return 0;
    }
    require(mat.dims == 2 && mat.type() == CV_64FC2 && mat.cols == 1 && mat.rows > 0,
            "image points must be Nx1 Float64 C2");
    return mat.rows;
}

void validate_finite_object_points(const cv::Mat &mat) {
    for (int i = 0; i < mat.rows; ++i) {
        const cv::Vec3d value = mat.at<cv::Vec3d>(i, 0);
        require(std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]),
                "object points must be finite");
    }
}

void validate_finite_image_points(const cv::Mat &mat) {
    for (int i = 0; i < mat.rows; ++i) {
        const cv::Vec2d value = mat.at<cv::Vec2d>(i, 0);
        require(std::isfinite(value[0]) && std::isfinite(value[1]),
                "image points must be finite");
    }
}

opencv_calib3d_pose pose_from_mats(const cv::Mat &rvec, const cv::Mat &tvec) {
    require(rvec.checkVector(1, CV_64F) == 3 && tvec.checkVector(1, CV_64F) == 3,
            "native pose vectors must contain three Float64 scalars");
    cv::Mat r = rvec.reshape(1, 3);
    cv::Mat t = tvec.reshape(1, 3);
    opencv_calib3d_pose result{r.at<double>(0), r.at<double>(1), r.at<double>(2),
                               t.at<double>(0), t.at<double>(1), t.at<double>(2)};
    require(finite_pose(result), "native pose contains nonfinite values");
    return result;
}
} // namespace

extern "C" {
#ifdef OPENCV_CALIB3D_TEST_HOOKS
void opencv_calib3d_test_fail(int stage, int kind) {
    failure_stage = stage;
    failure_kind = kind;
}
#endif

const char *opencv_calib3d_last_error(void) { return error_text; }
const char *opencv_calib3d_native_version(void) { return CV_VERSION; }
const char *opencv_calib3d_native_backend(void) {
#if CV_VERSION_MAJOR == 4
    return "calib3d";
#else
    return "geometry";
#endif
}

opencv_calib3d_status opencv_calib3d_project_points(
    const opencv_core_mat_handle *object_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    const opencv_calib3d_pose *pose,
    opencv_core_mat_handle *image_points) {
    return guarded([&] {
        validate_intrinsics(intrinsics);
        validate_distortion(distortion);
        validate_pose(pose);
        const cv::Mat &objects = resolve_input(object_points);
        const int count = validate_object_points(objects, true);
        if (count > 0) validate_finite_object_points(objects);
        cv::Mat &output = resolve_output(image_points);
        cv::Mat projected;
        checkpoint(1);
        if (count > 0) {
            cv::projectPoints(objects, rotation_vector(*pose), translation_vector(*pose),
                              camera_matrix(*intrinsics), distortion_vector(*distortion), projected);
            require(projected.dims == 2 && projected.type() == CV_64FC2 && projected.cols == 1 &&
                    projected.rows == count, "native projectPoints output schema differs");
            validate_finite_image_points(projected);
        }
        checkpoint(2);
        output = std::move(projected);
    });
}

opencv_calib3d_status opencv_calib3d_camera_center(
    const opencv_calib3d_pose *pose, opencv_calib3d_point3 *center) {
    if (center != nullptr) *center = {};
    return guarded([&] {
        require(center != nullptr, "null camera-center output");
        validate_pose(pose);
        checkpoint(3);
        cv::Mat rotation;
        cv::Rodrigues(rotation_vector(*pose), rotation);
        cv::Mat center_mat = -rotation.t() * translation_vector(*pose);
        require(center_mat.rows == 3 && center_mat.cols == 1 && center_mat.type() == CV_64FC1,
                "native camera-center schema differs");
        const opencv_calib3d_point3 value{center_mat.at<double>(0), center_mat.at<double>(1),
                                          center_mat.at<double>(2)};
        require(std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z),
                "native camera center is nonfinite");
        checkpoint(4);
        *center = value;
    });
}

opencv_calib3d_status opencv_calib3d_solve_pnp_ransac(
    const opencv_core_mat_handle *object_points,
    const opencv_core_mat_handle *image_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    int32_t maximum_iterations, double reprojection_error_pixels, double confidence,
    opencv_calib3d_pose_result_handle **out_result) {
    if (out_result != nullptr) *out_result = nullptr;
    return guarded([&] {
        require(out_result != nullptr, "null pose-result output");
        validate_intrinsics(intrinsics);
        validate_distortion(distortion);
        require(opencv_calib3d_detail::ransac_options_fit(maximum_iterations,
                    reprojection_error_pixels, confidence), "invalid RANSAC options");
        const cv::Mat &objects = resolve_input(object_points);
        const cv::Mat &images = resolve_input(image_points);
        const int count = validate_object_points(objects, false);
        require(count == validate_image_points(images, false), "point correspondence counts differ");
        require(count >= 4, "solvePnPRansac requires at least four correspondences");
        validate_finite_object_points(objects);
        validate_finite_image_points(images);
        checkpoint(5);
        cv::Mat rvec, tvec, inliers;
        const bool found = cv::solvePnPRansac(
            objects, images, camera_matrix(*intrinsics), distortion_vector(*distortion),
            rvec, tvec, false, maximum_iterations,
            static_cast<float>(reprojection_error_pixels), confidence, inliers, cv::SOLVEPNP_EPNP);
        checkpoint(6);
        auto result = std::make_unique<opencv_calib3d_pose_result_handle>();
        result->found = found;
        if (found) {
            result->pose = pose_from_mats(rvec, tvec);
            require(!inliers.empty() && inliers.type() == CV_32SC1,
                    "successful native pose has invalid inlier output");
            cv::Mat flat = inliers.reshape(1, 1);
            require(flat.cols >= 4 && flat.cols <= count, "successful pose has invalid inlier count");
            result->inliers.reserve(static_cast<std::size_t>(flat.cols));
            for (int i = 0; i < flat.cols; ++i) {
                const int value = flat.at<int>(0, i);
                require(value >= 0 && value < count, "native inlier index out of range");
                result->inliers.push_back(static_cast<int32_t>(value));
            }
            std::sort(result->inliers.begin(), result->inliers.end());
            require(std::adjacent_find(result->inliers.begin(), result->inliers.end()) == result->inliers.end(),
                    "native inlier indices contain duplicates");
        } else {
            result->pose = {};
            result->inliers.clear();
        }
        checkpoint(7);
        *out_result = result.release();
    });
}

opencv_calib3d_status opencv_calib3d_pose_result_found(
    const opencv_calib3d_pose_result_handle *result, uint8_t *found) {
    if (found != nullptr) *found = 0;
    return guarded([&] {
        require(result != nullptr && found != nullptr, "null pose-result found argument");
        *found = result->found ? 1U : 0U;
    });
}

opencv_calib3d_status opencv_calib3d_pose_result_pose(
    const opencv_calib3d_pose_result_handle *result, opencv_calib3d_pose *pose) {
    if (pose != nullptr) *pose = {};
    return guarded([&] {
        require(result != nullptr && pose != nullptr, "null pose-result pose argument");
        require(result->found, "pose requested from a no-pose result");
        checkpoint(8);
        *pose = result->pose;
    });
}

opencv_calib3d_status opencv_calib3d_pose_result_inlier_count(
    const opencv_calib3d_pose_result_handle *result, int32_t *count) {
    if (count != nullptr) *count = 0;
    return guarded([&] {
        require(result != nullptr && count != nullptr, "null pose-result count argument");
        require(result->inliers.size() <= static_cast<std::size_t>(INT32_MAX),
                "inlier count is not representable");
        *count = static_cast<int32_t>(result->inliers.size());
    });
}

opencv_calib3d_status opencv_calib3d_pose_result_inlier(
    const opencv_calib3d_pose_result_handle *result, int32_t index, int32_t *correspondence_index) {
    if (correspondence_index != nullptr) *correspondence_index = 0;
    return guarded([&] {
        require(result != nullptr && correspondence_index != nullptr,
                "null pose-result inlier argument");
        require(index >= 0 && static_cast<std::size_t>(index) < result->inliers.size(),
                "inlier index out of range");
        checkpoint(9);
        *correspondence_index = result->inliers[static_cast<std::size_t>(index)];
    });
}

void opencv_calib3d_pose_result_destroy(opencv_calib3d_pose_result_handle *result) {
    try { delete result; } catch (...) {}
}
} // extern "C"
