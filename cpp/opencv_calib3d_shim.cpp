#include "opencv_calib3d_shim.h"
#include "pnp_profile.hpp"
#include "homography_profile.hpp"
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
#include <initializer_list>
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
static_assert(std::is_standard_layout<opencv_calib3d_rotation_matrix>::value,
              "rotation matrix must have standard C layout");

struct opencv_calib3d_pose_result_handle {
    bool found = false;
    opencv_calib3d_pose pose{};
    std::vector<int32_t> inliers;
};

static_assert(std::is_standard_layout<opencv_calib3d_homography>::value,
              "homography must have standard C layout");
struct opencv_calib3d_homography_result_handle {
    bool found = false;
    opencv_calib3d_homography matrix{};
    std::vector<int32_t> inliers;
};

namespace {
thread_local char error_text[1024] = "";

#ifdef OPENCV_CALIB3D_TEST_HOOKS
thread_local int failure_stage = 0;
thread_local int failure_kind = 0;
thread_local bool refinement_false = false;
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

cv::Mat double_matrix(int rows, int columns, std::initializer_list<double> values) {
    require(values.size() == static_cast<std::size_t>(rows * columns),
            "internal matrix initializer size differs");
    cv::Mat result(rows, columns, CV_64FC1);
    std::copy(values.begin(), values.end(), result.ptr<double>());
    return result;
}

cv::Mat camera_matrix(const opencv_calib3d_camera_intrinsics &value) {
    return double_matrix(3, 3, {value.focal_x, 0.0, value.center_x,
                               0.0, value.focal_y, value.center_y,
                               0.0, 0.0, 1.0});
}

cv::Mat distortion_vector(const opencv_calib3d_distortion5 &value) {
    return double_matrix(5, 1, {value.k1, value.k2, value.p1, value.p2, value.k3});
}

cv::Mat rotation_vector(const opencv_calib3d_pose &value) {
    return double_matrix(3, 1, {value.rx, value.ry, value.rz});
}

cv::Mat translation_vector(const opencv_calib3d_pose &value) {
    return double_matrix(3, 1, {value.tx, value.ty, value.tz});
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
void opencv_calib3d_test_refinement_false(void) {
    refinement_false = true;
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

opencv_calib3d_status opencv_calib3d_find_homography_ransac(
    const opencv_core_mat_handle *source_points,
    const opencv_core_mat_handle *destination_points,
    const opencv_calib3d_homography_options *options,
    opencv_calib3d_homography_result_handle **result) {
    if (result != nullptr) *result = nullptr;
    return guarded([&] {
        require(result != nullptr && options != nullptr, "null homography options/output");
        require(opencv_calib3d_detail::ransac_options_fit(options->maximum_iterations,
                options->reprojection_threshold_pixels, options->confidence),
                "invalid homography RANSAC options");
        const cv::Mat &original_source = resolve_input(source_points);
        const cv::Mat &original_destination = resolve_input(destination_points);
        const int count = validate_image_points(original_source, false);
        require(count == validate_image_points(original_destination, false),
                "homography correspondence counts differ");
        require(count >= 5, "robust homography requires at least five correspondences");
        validate_finite_image_points(original_source);
        validate_finite_image_points(original_destination);
        const cv::Mat source = original_source.isContinuous() ? original_source : original_source.clone();
        const cv::Mat destination = original_destination.isContinuous() ? original_destination : original_destination.clone();
        cv::Mat native_mask;
        checkpoint(17);
        const cv::Mat h = cv::findHomography(source, destination, cv::RANSAC,
            options->reprojection_threshold_pixels, native_mask,
            options->maximum_iterations, options->confidence);
        checkpoint(18);
        auto value = std::make_unique<opencv_calib3d_homography_result_handle>();
        if (!h.empty()) {
            if (h.dims != 2 || h.rows != 3 || h.cols != 3 || h.type() != CV_64FC1)
                throw std::runtime_error("native homography schema is invalid");
            bool nonzero = false;
            for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
                const double coefficient = h.at<double>(row, col);
                if (!std::isfinite(coefficient))
                    throw std::runtime_error("native homography is nonfinite");
                nonzero = nonzero || coefficient != 0;
            }
            if (!nonzero) throw std::runtime_error("native homography is all zero");
            value->matrix = {h.at<double>(0,0),h.at<double>(0,1),h.at<double>(0,2),
                             h.at<double>(1,0),h.at<double>(1,1),h.at<double>(1,2),
                             h.at<double>(2,0),h.at<double>(2,1),h.at<double>(2,2)};
            checkpoint(19);
            // Never publish native_mask: 4.x and 5.0 differ after refinement.
            // Classify the original Float64 values in correspondence order.
            for (int i = 0; i < count; ++i) {
                const auto s = original_source.at<cv::Vec2d>(i,0);
                const auto d = original_destination.at<cv::Vec2d>(i,0);
                if (opencv_calib3d_detail::final_homography_inlier(value->matrix,
                    s[0],s[1],d[0],d[1],options->reprojection_threshold_pixels))
                    value->inliers.push_back(i);
            }
            value->found = value->inliers.size() >= 4;
            if (!value->found) {
                value->matrix = {};
                value->inliers.clear();
            }
        }
        checkpoint(20);
        *result = value.release();
    });
}

opencv_calib3d_status opencv_calib3d_homography_result_found(
    const opencv_calib3d_homography_result_handle *result, uint8_t *found) {
    if (found != nullptr) *found = 0;
    return guarded([&] {
        require(result != nullptr && found != nullptr, "null homography found argument");
        *found = result->found ? 1U : 0U;
    });
}
opencv_calib3d_status opencv_calib3d_homography_result_matrix(
    const opencv_calib3d_homography_result_handle *result, opencv_calib3d_homography *matrix) {
    if (matrix != nullptr) *matrix = {};
    return guarded([&] {
        require(result != nullptr && matrix != nullptr, "null homography matrix argument");
        require(result->found, "matrix requested from a no-model result");
        checkpoint(21);
        *matrix = result->matrix;
    });
}
opencv_calib3d_status opencv_calib3d_homography_result_inlier_count(
    const opencv_calib3d_homography_result_handle *result, int32_t *count) {
    if (count != nullptr) *count = 0;
    return guarded([&] {
        require(result != nullptr && count != nullptr, "null homography count argument");
        require(result->inliers.size() <= static_cast<std::size_t>(INT32_MAX),
                "homography count is not representable");
        *count = static_cast<int32_t>(result->inliers.size());
    });
}
opencv_calib3d_status opencv_calib3d_homography_result_inlier(
    const opencv_calib3d_homography_result_handle *result, int32_t index, int32_t *correspondence_index) {
    if (correspondence_index != nullptr) *correspondence_index = 0;
    return guarded([&] {
        require(result != nullptr && correspondence_index != nullptr, "null homography inlier argument");
        require(index >= 0 && static_cast<std::size_t>(index) < result->inliers.size(),
                "homography inlier index out of range");
        checkpoint(22);
        *correspondence_index = result->inliers[static_cast<std::size_t>(index)];
    });
}
void opencv_calib3d_homography_result_destroy(opencv_calib3d_homography_result_handle *result) {
    try { delete result; } catch (...) {}
}

opencv_calib3d_status opencv_calib3d_undistort_normalized(
    const opencv_core_mat_handle *image_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    opencv_core_mat_handle *normalized_points) {
    return guarded([&] {
        validate_intrinsics(intrinsics);
        validate_distortion(distortion);
        const cv::Mat &images = resolve_input(image_points);
        const int count = validate_image_points(images, true);
        if (count > 0) validate_finite_image_points(images);
        cv::Mat &output = resolve_output(normalized_points);
        cv::Mat normalized;
        if (count > 0) {
            // All reviewed versions require continuous input. Snapshot a valid
            // strided Core Region; never retain the borrowed header or handle.
            const cv::Mat input = images.isContinuous() ? images : images.clone();
            // Empty coefficients are the documented zero-distortion model.
            // Avoid meaningless 0*r6 overflow for large finite ideal points.
            const bool zero_distortion = distortion->k1 == 0 && distortion->k2 == 0 &&
                distortion->p1 == 0 && distortion->p2 == 0 && distortion->k3 == 0;
            const cv::Mat coefficients = zero_distortion ? cv::Mat() : distortion_vector(*distortion);
            checkpoint(13);
            cv::undistortPoints(input, normalized, camera_matrix(*intrinsics),
                                coefficients, cv::noArray(), cv::noArray(),
                                cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                                 20, 1.0e-12));
            checkpoint(14);
            require(normalized.dims == 2 && normalized.type() == CV_64FC2 &&
                    normalized.cols == 1 && normalized.rows == count,
                    "native undistortPoints output schema differs");
            validate_finite_image_points(normalized);
        }
        output = std::move(normalized);
    });
}

opencv_calib3d_status opencv_calib3d_rotation_matrix_of(
    const opencv_calib3d_pose *pose, opencv_calib3d_rotation_matrix *matrix) {
    if (matrix != nullptr) *matrix = {};
    return guarded([&] {
        require(matrix != nullptr, "null rotation-matrix output");
        validate_pose(pose);
        checkpoint(15);
        cv::Mat rotation;
        cv::Rodrigues(rotation_vector(*pose), rotation);
        checkpoint(16);
        require(rotation.dims == 2 && rotation.rows == 3 && rotation.cols == 3 &&
                rotation.type() == CV_64FC1, "native Rodrigues output schema differs");
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                require(std::isfinite(rotation.at<double>(row,col)),
                        "native Rodrigues output is nonfinite");
        const opencv_calib3d_rotation_matrix value{
            rotation.at<double>(0,0), rotation.at<double>(0,1), rotation.at<double>(0,2),
            rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2),
            rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2)};
        *matrix = value;
    });
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

opencv_calib3d_status opencv_calib3d_refine_pose_iterative(
    const opencv_core_mat_handle *object_points,
    const opencv_core_mat_handle *image_points,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    const opencv_calib3d_distortion5 *distortion,
    const opencv_calib3d_pose *initial_pose,
    uint8_t *refined, opencv_calib3d_pose *refined_pose) {
    if (refined != nullptr) *refined = 0;
    if (refined_pose != nullptr) *refined_pose = {};
    return guarded([&] {
        require(refined != nullptr && refined_pose != nullptr, "null refinement output");
        validate_intrinsics(intrinsics);
        validate_distortion(distortion);
        validate_pose(initial_pose);
        const cv::Mat &objects = resolve_input(object_points);
        const cv::Mat &images = resolve_input(image_points);
        const int count = validate_object_points(objects, false);
        require(count == validate_image_points(images, false), "point correspondence counts differ");
        require(count >= 4, "iterative refinement binding requires at least four correspondences");
        validate_finite_object_points(objects);
        validate_finite_image_points(images);
        checkpoint(10);
        cv::Mat rvec = rotation_vector(*initial_pose);
        cv::Mat tvec = translation_vector(*initial_pose);
        bool solved = cv::solvePnP(objects, images, camera_matrix(*intrinsics),
                                  distortion_vector(*distortion), rvec, tvec,
                                  true, cv::SOLVEPNP_ITERATIVE);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
        if (refinement_false) {
            refinement_false = false;
            solved = false;
        }
#endif
        checkpoint(11);
        if (!solved) return;
        const opencv_calib3d_pose value = pose_from_mats(rvec, tvec);
        checkpoint(12);
        *refined_pose = value;
        *refined = 1;
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
