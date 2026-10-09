#include "opencv_calib3d_shim.h"
#include "pnp_profile.hpp"
#include "homography_profile.hpp"
#include "fundamental_profile.hpp"
#include "essential_profile.hpp"
#include "triangulation_profile.hpp"
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
#include <limits>
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

static_assert(std::is_standard_layout<opencv_calib3d_fundamental>::value,
              "fundamental must have standard C layout");
struct opencv_calib3d_fundamental_result_handle {
    bool found = false;
    opencv_calib3d_fundamental matrix{};
    std::vector<int32_t> inliers;
};

static_assert(std::is_standard_layout<opencv_calib3d_essential>::value,
              "essential must have standard C layout");
static_assert(std::is_standard_layout<opencv_calib3d_relative_pose>::value,
              "relative pose must have standard C layout");
static_assert(std::is_standard_layout<opencv_calib3d_essential_options>::value,
              "essential options must have standard C layout");
struct opencv_calib3d_essential_result_handle {
    bool found = false, pose_found = false;
    opencv_calib3d_essential matrix{};
    opencv_calib3d_relative_pose pose{};
    std::vector<int32_t> inliers, pose_inliers;
};

struct opencv_calib3d_triangulation_result_handle {
    std::vector<opencv_calib3d_triangulated_point> points;
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    static thread_local int live;
    opencv_calib3d_triangulation_result_handle() { ++live; }
    ~opencv_calib3d_triangulation_result_handle() { --live; }
#endif
};
#ifdef OPENCV_CALIB3D_TEST_HOOKS
thread_local int opencv_calib3d_triangulation_result_handle::live = 0;
#endif

namespace {
thread_local char error_text[1024] = "";

#ifdef OPENCV_CALIB3D_TEST_HOOKS
thread_local int failure_stage = 0;
thread_local int failure_kind = 0;
thread_local bool refinement_false = false;
thread_local bool essential_no_model = false;
thread_local bool essential_no_pose = false;
thread_local bool triangulation_override = false, triangulation_unknown = false;
thread_local double triangulation_h[4]{};
thread_local int planar_control = 0;
thread_local int visibility_control = 0;
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
#ifdef OPENCV_CALIB3D_TEST_HOOKS
void opencv_calib3d_test_planar_control(int mode) { planar_control = mode; }
void opencv_calib3d_test_visibility_control(int mode) { visibility_control = mode; }
#endif
const char *opencv_calib3d_native_version(void) { return CV_VERSION; }
const char *opencv_calib3d_native_backend(void) {
#if CV_VERSION_MAJOR == 4
    return "calib3d";
#else
    return "geometry";
#endif
}

opencv_calib3d_status opencv_calib3d_decompose_homography(
    const opencv_calib3d_homography *matrix,
    const opencv_calib3d_camera_intrinsics *intrinsics,
    opencv_calib3d_planar_decomposition *result) {
    if (result) *result = {};
    return guarded([&] {
        require(matrix && result, "null homography decomposition argument");
        validate_intrinsics(intrinsics);
        const double entries[] = {matrix->h00,matrix->h01,matrix->h02,
            matrix->h10,matrix->h11,matrix->h12,matrix->h20,matrix->h21,matrix->h22};
        double scale = 0;
        for (double v : entries) {
            require(std::isfinite(v), "nonfinite homography");
            scale = std::max(scale,std::abs(v));
        }
        require(scale > 0, "zero homography");
        cv::Mat h(3,3,CV_64FC1);
        for (int i=0;i<9;++i) {
            const double v = entries[i]/scale;
            require(std::isfinite(v) && (entries[i]==0 || v!=0),
                    "unrepresentable homography canonicalization");
            h.at<double>(i/3,i%3) = v;
        }
        // Determinant orientation removes negative projective scale without
        // relying on h22. Long double avoids double product underflow here.
        const long double a=h.at<double>(0,0),b=h.at<double>(0,1),c=h.at<double>(0,2);
        const long double d=h.at<double>(1,0),e=h.at<double>(1,1),f=h.at<double>(1,2);
        const long double g=h.at<double>(2,0),j=h.at<double>(2,1),k=h.at<double>(2,2);
        const long double determinant=a*(e*k-f*j)-b*(d*k-f*g)+c*(d*j-e*g);
        require(std::isfinite(determinant) && determinant!=0, "singular homography");
        if (determinant<0) h *= -1;
        const cv::Mat camera = camera_matrix(*intrinsics);
        const cv::Mat normalized = camera.inv()*h*camera;
        require(cv::checkRange(normalized), "unrepresentable calibrated homography");
        cv::SVD singular(normalized,cv::SVD::NO_UV);
        require(singular.w.at<double>(1)>0 && singular.w.at<double>(2)>0,
                "singular calibrated homography");
        std::vector<cv::Mat> rotations, translations, normals;
        checkpoint(42);
        int count=cv::decomposeHomographyMat(h,camera,rotations,translations,normals);
        checkpoint(43);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
        // Explicit controls AFTER a real native call; not degenerate-input behavior.
        const int control=planar_control;
        planar_control=0;
        if (control==1) { count=0; rotations.clear(); translations.clear(); normals.clear(); }
        if (control==2) rotations[0]=cv::Mat::zeros(2,2,CV_32F);
        if (control==3) count=5;
        if (control==4) translations[0].at<double>(0)=std::numeric_limits<double>::infinity();
#endif
        if (count<0 || count>4 || rotations.size()!=static_cast<std::size_t>(count) ||
            translations.size()!=rotations.size() || normals.size()!=rotations.size())
            throw std::runtime_error("malformed decomposition count");
        opencv_calib3d_planar_decomposition local{};
        local.count=count;
        for (int i=0;i<count;++i) {
            checkpoint(44);
            const auto &r=rotations[i], &t=translations[i], &n=normals[i];
            if (r.rows!=3 || r.cols!=3 || r.type()!=CV_64FC1 ||
                t.rows!=3 || t.cols!=1 || t.type()!=CV_64FC1 ||
                n.rows!=3 || n.cols!=1 || n.type()!=CV_64FC1 ||
                !cv::checkRange(r) || !cv::checkRange(t) || !cv::checkRange(n))
                throw std::runtime_error("malformed decomposition candidate");
            // Native near-rotation branch returns Hnorm directly, not an SO(3)
            // projection. Preserve its documented tolerance, without modifying R.
            const bool pure=cv::norm(t)==0 && cv::norm(n)==0;
            const double tolerance=pure ? 0.001 : 1e-9;
            if ((pure && count!=1) || (!pure && count!=4) ||
                cv::norm(r.t()*r-cv::Mat::eye(3,3,CV_64F),cv::NORM_INF)>tolerance ||
                std::abs(cv::determinant(r)-1)>tolerance ||
                (!pure && (cv::norm(t)==0 || std::abs(cv::norm(n)-1)>1e-9)))
                throw std::runtime_error("invalid decomposition geometry");
            local.candidates[i]={r.at<double>(0,0),r.at<double>(0,1),r.at<double>(0,2),
                r.at<double>(1,0),r.at<double>(1,1),r.at<double>(1,2),
                r.at<double>(2,0),r.at<double>(2,1),r.at<double>(2,2),
                t.at<double>(0),t.at<double>(1),t.at<double>(2),
                n.at<double>(0),n.at<double>(1),n.at<double>(2)};
        }
        checkpoint(45);
        *result=local;
    });
}

opencv_calib3d_status opencv_calib3d_filter_planar_visibility(
    const opencv_calib3d_planar_decomposition *hypotheses,
    const opencv_core_mat_handle *first, const opencv_core_mat_handle *second,
    opencv_calib3d_planar_visibility *result) {
    if (result) *result = {};
    return guarded([&] {
        require(hypotheses && result, "null visibility argument");
        require(hypotheses->count > 0 && hypotheses->count <= 4,
                "visibility requires one to four general candidates");
        std::vector<cv::Mat> rotations, normals;
        for (int i=0; i<hypotheses->count; ++i) {
            const auto &v=hypotheses->candidates[i];
            const double fields[]={v.r00,v.r01,v.r02,v.r10,v.r11,v.r12,
                v.r20,v.r21,v.r22,v.tx,v.ty,v.tz,v.nx,v.ny,v.nz};
            for (double x:fields) require(std::isfinite(x), "nonfinite visibility candidate");
            cv::Mat r=double_matrix(3,3,{v.r00,v.r01,v.r02,v.r10,v.r11,v.r12,v.r20,v.r21,v.r22});
            cv::Mat n=double_matrix(3,1,{v.nx,v.ny,v.nz});
            require((v.tx!=0 || v.ty!=0 || v.tz!=0) &&
                std::abs(std::hypot(v.nx,v.ny,v.nz)-1)<=1e-9 &&
                cv::norm(r.t()*r-cv::Mat::eye(3,3,CV_64F),cv::NORM_INF)<=1e-9 &&
                std::abs(cv::determinant(r)-1)<=1e-9, "invalid visibility candidate geometry");
            rotations.push_back(r); normals.push_back(n);
        }
        const auto &a=resolve_input(first), &b=resolve_input(second);
        const int count=validate_image_points(a,false);
        require(count==validate_image_points(b,false), "visibility point counts differ");
        auto snapshot=[&](const cv::Mat &source) {
            cv::Mat target(count,1,CV_32FC2);
            for (int i=0;i<count;++i) for (int j=0;j<2;++j) {
                const double x=source.at<cv::Vec2d>(i,0)[j];
                require(std::isfinite(x) && std::abs(x)<=std::numeric_limits<float>::max(),
                        "visibility Float32 observation overflow/nonfinite");
                const float y=static_cast<float>(x);
                require(std::isfinite(y) && (x==0 || y!=0), "visibility Float32 observation underflow");
                target.at<cv::Vec2f>(i,0)[j]=y;
            }
            return target;
        };
        const cv::Mat af=snapshot(a), bf=snapshot(b);
        cv::Mat possible;
        checkpoint(46);
        cv::filterHomographyDecompByVisibleRefpoints(rotations,normals,af,bf,possible,cv::noArray());
        checkpoint(47);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
        const int control=visibility_control;
        visibility_control=0;
        if (control==1) possible=cv::Mat::ones(1,1,CV_64F);
        if (control==2) possible=(cv::Mat_<int>(2,1)<<0,0);
        if (control==3) possible=(cv::Mat_<int>(2,1)<<1,0);
        if (control==4) possible=(cv::Mat_<int>(1,1)<<hypotheses->count);
        if (control==5) possible=(cv::Mat_<int>(1,1)<<-1);
#endif
        opencv_calib3d_planar_visibility local{};
        local.count=hypotheses->count;
        checkpoint(48);
        if (!possible.empty()) {
            const int survivors=possible.checkVector(1,CV_32S);
            require(possible.type()==CV_32SC1 && survivors>0 && survivors<=local.count,
                    "invalid native visibility index schema");
            const cv::Mat flat=possible.reshape(1,survivors);
            int previous=-1;
            for (int i=0;i<survivors;++i) {
                const int index=flat.at<int>(i,0);
                require(index>previous && index<local.count, "invalid native visibility index");
                local.accepted[index]=1; previous=index;
            }
        }
        checkpoint(49);
        *result=local;
    });
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

opencv_calib3d_status opencv_calib3d_validate_fundamental_options(
    const opencv_calib3d_fundamental_options *options) {
    return guarded([&] {
        require(options != nullptr && opencv_calib3d_detail::fundamental_options_fit(
            options->epipolar_threshold_pixels, options->confidence),
            "invalid fundamental RANSAC options");
    });
}

opencv_calib3d_status opencv_calib3d_find_fundamental_ransac(
    const opencv_core_mat_handle *first_points,
    const opencv_core_mat_handle *second_points,
    const opencv_calib3d_fundamental_options *options,
    opencv_calib3d_fundamental_result_handle **result) {
    if (result != nullptr) *result = nullptr;
    return guarded([&] {
        require(result != nullptr && options != nullptr, "null fundamental options/output");
        require(opencv_calib3d_detail::fundamental_options_fit(
                options->epipolar_threshold_pixels, options->confidence),
                "invalid fundamental RANSAC options");
        const cv::Mat &original_first = resolve_input(first_points);
        const cv::Mat &original_second = resolve_input(second_points);
        const int count = validate_image_points(original_first, false);
        require(count == validate_image_points(original_second, false),
                "fundamental correspondence counts differ");
        require(count >= 15, "robust fundamental requires at least fifteen correspondences");
        validate_finite_image_points(original_first);
        validate_finite_image_points(original_second);
        const cv::Mat first = original_first.isContinuous() ? original_first : original_first.clone();
        const cv::Mat second = original_second.isContinuous() ? original_second : original_second.clone();
        cv::Mat native_mask;
        checkpoint(23);
        const cv::Mat h = cv::findFundamentalMat(first, second, cv::FM_RANSAC,
            options->epipolar_threshold_pixels, options->confidence, native_mask);
        checkpoint(24);
        auto value = std::make_unique<opencv_calib3d_fundamental_result_handle>();
        if (!h.empty()) {
            if (h.dims != 2 || h.rows != 3 || h.cols != 3 || h.type() != CV_64FC1)
                throw std::runtime_error("native fundamental schema is invalid");
            bool nonzero = false;
            for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
                const double coefficient = h.at<double>(row, col);
                if (!std::isfinite(coefficient))
                    throw std::runtime_error("native fundamental is nonfinite");
                nonzero = nonzero || coefficient != 0;
            }
            if (!nonzero) throw std::runtime_error("native fundamental is all zero");
            value->matrix = {h.at<double>(0,0),h.at<double>(0,1),h.at<double>(0,2),
                             h.at<double>(1,0),h.at<double>(1,1),h.at<double>(1,2),
                             h.at<double>(2,0),h.at<double>(2,1),h.at<double>(2,2)};
            checkpoint(25);
            // Never publish the upstream Float32 mask; classify against final F.
            // Classify the original Float64 values in correspondence order.
            for (int i = 0; i < count; ++i) {
                const auto s = original_first.at<cv::Vec2d>(i,0);
                const auto d = original_second.at<cv::Vec2d>(i,0);
                if (opencv_calib3d_detail::final_fundamental_inlier(value->matrix,
                    s[0],s[1],d[0],d[1],options->epipolar_threshold_pixels))
                    value->inliers.push_back(i);
            }
            value->found = value->inliers.size() >= 7;
            if (!value->found) {
                value->matrix = {};
                value->inliers.clear();
            }
        }
        checkpoint(26);
        *result = value.release();
    });
}

opencv_calib3d_status opencv_calib3d_fundamental_result_found(
    const opencv_calib3d_fundamental_result_handle *result, uint8_t *found) {
    if (found != nullptr) *found = 0;
    return guarded([&] {
        require(result != nullptr && found != nullptr, "null fundamental found argument");
        *found = result->found ? 1U : 0U;
    });
}
opencv_calib3d_status opencv_calib3d_fundamental_result_matrix(
    const opencv_calib3d_fundamental_result_handle *result, opencv_calib3d_fundamental *matrix) {
    if (matrix != nullptr) *matrix = {};
    return guarded([&] {
        require(result != nullptr && matrix != nullptr, "null fundamental matrix argument");
        require(result->found, "matrix requested from a no-model result");
        checkpoint(27);
        *matrix = result->matrix;
    });
}
opencv_calib3d_status opencv_calib3d_fundamental_result_inlier_count(
    const opencv_calib3d_fundamental_result_handle *result, int32_t *count) {
    if (count != nullptr) *count = 0;
    return guarded([&] {
        require(result != nullptr && count != nullptr, "null fundamental count argument");
        require(result->inliers.size() <= static_cast<std::size_t>(INT32_MAX),
                "fundamental count is not representable");
        *count = static_cast<int32_t>(result->inliers.size());
    });
}
opencv_calib3d_status opencv_calib3d_fundamental_result_inlier(
    const opencv_calib3d_fundamental_result_handle *result, int32_t index, int32_t *correspondence_index) {
    if (correspondence_index != nullptr) *correspondence_index = 0;
    return guarded([&] {
        require(result != nullptr && correspondence_index != nullptr, "null fundamental inlier argument");
        require(index >= 0 && static_cast<std::size_t>(index) < result->inliers.size(),
                "fundamental inlier index out of range");
        checkpoint(28);
        *correspondence_index = result->inliers[static_cast<std::size_t>(index)];
    });
}
void opencv_calib3d_fundamental_result_destroy(opencv_calib3d_fundamental_result_handle *result) {
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
opencv_calib3d_status opencv_calib3d_validate_essential_options(
    const opencv_calib3d_essential_options *options) {
    return guarded([&] {
        require(options && opencv_calib3d_detail::essential_options_fit(
            options->normalized_epipolar_threshold,options->confidence),
            "invalid essential threshold/confidence profile");
    });
}

opencv_calib3d_status opencv_calib3d_normalized_sampson_error(
    const opencv_calib3d_essential *matrix, double x1, double y1, double x2, double y2,
    uint8_t *defined, double *error) {
    if (defined) *defined = 0;
    if (error) *error = 0;
    return guarded([&] {
        require(matrix && defined && error, "null Sampson argument");
        require(opencv_calib3d_detail::essential_finite(*matrix,x1,y1,x2,y2),
                "Sampson inputs must be finite");
        double value = 0;
        if (opencv_calib3d_detail::normalized_sampson_error(*matrix,x1,y1,x2,y2,value)) {
            *error = value;
            *defined = 1;
        }
    });
}

#ifdef OPENCV_CALIB3D_TEST_HOOKS
void opencv_calib3d_test_essential_no_model(void) { essential_no_model = true; }
void opencv_calib3d_test_essential_no_pose(void) { essential_no_pose = true; }
#endif

opencv_calib3d_status opencv_calib3d_find_essential_ransac(
    const opencv_core_mat_handle *first_points, const opencv_core_mat_handle *second_points,
    const opencv_calib3d_essential_options *options,
    opencv_calib3d_essential_result_handle **result) {
    if (result) *result = nullptr;
    return guarded([&] {
        require(result, "null essential result output");
        require(options && opencv_calib3d_detail::essential_options_fit(
            options->normalized_epipolar_threshold,options->confidence),
            "invalid essential options");
        const auto &first_input = resolve_input(first_points);
        const auto &second_input = resolve_input(second_points);
        const int count = validate_image_points(first_input, true);
        require(count == validate_image_points(second_input, true) && count >= 6,
                "essential RANSAC requires equal counts >=6");
        validate_finite_image_points(first_input);
        validate_finite_image_points(second_input);
        // Snapshot both original CV_64F observations, including strided Regions.
        const cv::Mat first = first_input.clone(), second = second_input.clone();
        const cv::Mat identity = cv::Mat::eye(3,3,CV_64F);
        cv::Mat native_mask;
        checkpoint(29);
#if CV_VERSION_MAJOR >= 5
        // 5.0 removed the compatibility overload. The required argument is
        // fixed, not caller policy; 4.x retains the 4.1-compatible call below.
        cv::Mat e = cv::findEssentialMat(first,second,identity,cv::RANSAC,
            options->confidence,options->normalized_epipolar_threshold,1000,native_mask);
#else
        cv::Mat e = cv::findEssentialMat(first,second,identity,cv::RANSAC,
            options->confidence,options->normalized_epipolar_threshold,native_mask);
#endif
        checkpoint(30);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
        if (essential_no_model) { essential_no_model = false; e.release(); }
#endif
        auto value = std::make_unique<opencv_calib3d_essential_result_handle>();
        if (!e.empty()) {
            if (e.dims != 2 || e.rows != 3 || e.cols != 3 || e.type() != CV_64FC1)
                throw std::runtime_error("native essential schema is invalid");
            bool nonzero = false;
            for (int row=0;row<3;++row) for (int col=0;col<3;++col) {
                const double coefficient = e.at<double>(row,col);
                if (!std::isfinite(coefficient))
                    throw std::runtime_error("native essential is nonfinite");
                nonzero = nonzero || coefficient != 0;
            }
            if (!nonzero) throw std::runtime_error("native essential is all zero");
            value->matrix = {e.at<double>(0,0),e.at<double>(0,1),e.at<double>(0,2),
                e.at<double>(1,0),e.at<double>(1,1),e.at<double>(1,2),
                e.at<double>(2,0),e.at<double>(2,1),e.at<double>(2,2)};
            cv::Mat mask = cv::Mat::zeros(count,1,CV_8U);
            for (int i=0;i<count;++i) {
                const auto p = first.at<cv::Vec2d>(i,0), q = second.at<cv::Vec2d>(i,0);
                if (opencv_calib3d_detail::final_essential_inlier(value->matrix,
                    p[0],p[1],q[0],q[1],options->normalized_epipolar_threshold)) {
                    value->inliers.push_back(i);
                    mask.at<uint8_t>(i,0) = 255;
                }
            }
            value->found = value->inliers.size() >= 5;
            checkpoint(31);
            if (value->found) {
                cv::Mat rotation, translation;
                const int support = cv::recoverPose(e,first,second,identity,rotation,translation,
                    std::numeric_limits<double>::max(),mask,cv::noArray());
                checkpoint(32);
                if (rotation.dims != 2 || rotation.rows != 3 || rotation.cols != 3 ||
                    rotation.type() != CV_64FC1 || translation.dims != 2 ||
                    translation.rows != 3 || translation.cols != 1 ||
                    translation.type() != CV_64FC1 || mask.dims != 2 ||
                    mask.rows != count || mask.cols != 1 || mask.type() != CV_8UC1)
                    throw std::runtime_error("native relative pose schema is invalid");
                for (int row=0;row<3;++row) {
                    for (int col=0;col<3;++col)
                        if (!std::isfinite(rotation.at<double>(row,col)))
                            throw std::runtime_error("native rotation is nonfinite");
                    if (!std::isfinite(translation.at<double>(row,0)))
                        throw std::runtime_error("native translation direction is nonfinite");
                }
                double tx=translation.at<double>(0), ty=translation.at<double>(1),
                       tz=translation.at<double>(2);
                const double scale=std::max({std::abs(tx),std::abs(ty),std::abs(tz)});
                if (scale == 0) throw std::runtime_error("native translation direction is zero");
                tx/=scale; ty/=scale; tz/=scale;
                const double norm=std::hypot(std::hypot(tx,ty),tz);
                tx/=norm; ty/=norm; tz/=norm;
                if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz))
                    throw std::runtime_error("invalid normalized translation direction");
                for (int i=0;i<count;++i) if (mask.at<uint8_t>(i,0)) {
                    if (!std::binary_search(value->inliers.begin(),value->inliers.end(),i))
                        throw std::runtime_error("native pose mask is not a final-E subset");
                    value->pose_inliers.push_back(i);
                }
                if (support < 0 || static_cast<std::size_t>(support) != value->pose_inliers.size())
                    throw std::runtime_error("native pose support/mask mismatch");
#ifdef OPENCV_CALIB3D_TEST_HOOKS
                if (essential_no_pose) { essential_no_pose = false; value->pose_inliers.clear(); }
#endif
                value->pose_found = value->pose_inliers.size() >= 5;
                if (value->pose_found)
                    value->pose = {rotation.at<double>(0,0),rotation.at<double>(0,1),rotation.at<double>(0,2),
                        rotation.at<double>(1,0),rotation.at<double>(1,1),rotation.at<double>(1,2),
                        rotation.at<double>(2,0),rotation.at<double>(2,1),rotation.at<double>(2,2),tx,ty,tz};
                else value->pose_inliers.clear();
            } else {
                value->matrix = {};
                value->inliers.clear();
            }
        }
        *result = value.release();
    });
}

opencv_calib3d_status opencv_calib3d_essential_result_found(
    const opencv_calib3d_essential_result_handle *result, uint8_t *found) {
    if (found) *found = 0;
    return guarded([&] {
        require(result && found, "null essential found argument");
        *found = result->found ? 1 : 0;
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_matrix(
    const opencv_calib3d_essential_result_handle *result, opencv_calib3d_essential *matrix) {
    if (matrix) *matrix = {};
    return guarded([&] {
        require(result && matrix, "null essential matrix argument");
        require(result->found, "matrix requested from a no-E result");
        checkpoint(33);
        *matrix = result->matrix;
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_inlier_count(
    const opencv_calib3d_essential_result_handle *result, int32_t *count) {
    if (count) *count = 0;
    return guarded([&] {
        require(result && count, "null essential count argument");
        *count = static_cast<int32_t>(result->inliers.size());
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_inlier(
    const opencv_calib3d_essential_result_handle *result, int32_t index, int32_t *correspondence_index) {
    if (correspondence_index) *correspondence_index = 0;
    return guarded([&] {
        require(result && correspondence_index, "null essential inlier argument");
        require(index >= 0 && static_cast<std::size_t>(index) < result->inliers.size(),
                "essential inlier index out of range");
        checkpoint(34);
        *correspondence_index = result->inliers[static_cast<std::size_t>(index)];
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_pose_found(
    const opencv_calib3d_essential_result_handle *result, uint8_t *found) {
    if (found) *found = 0;
    return guarded([&] {
        require(result && found, "null relative pose found argument");
        *found = result->pose_found ? 1 : 0;
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_pose(
    const opencv_calib3d_essential_result_handle *result, opencv_calib3d_relative_pose *pose) {
    if (pose) *pose = {};
    return guarded([&] {
        require(result && pose, "null relative pose argument");
        require(result->pose_found, "pose requested from a no-pose result");
        checkpoint(35);
        *pose = result->pose;
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_pose_inlier_count(
    const opencv_calib3d_essential_result_handle *result, int32_t *count) {
    if (count) *count = 0;
    return guarded([&] {
        require(result && count, "null relative pose count argument");
        *count = static_cast<int32_t>(result->pose_inliers.size());
    });
}
opencv_calib3d_status opencv_calib3d_essential_result_pose_inlier(
    const opencv_calib3d_essential_result_handle *result, int32_t index, int32_t *correspondence_index) {
    if (correspondence_index) *correspondence_index = 0;
    return guarded([&] {
        require(result && correspondence_index, "null relative pose inlier argument");
        require(index >= 0 && static_cast<std::size_t>(index) < result->pose_inliers.size(),
                "relative pose inlier index out of range");
        checkpoint(36);
        *correspondence_index = result->pose_inliers[static_cast<std::size_t>(index)];
    });
}
void opencv_calib3d_essential_result_destroy(opencv_calib3d_essential_result_handle *result) {
    try { delete result; } catch (...) {}
}
#ifdef OPENCV_CALIB3D_TEST_HOOKS
void opencv_calib3d_test_triangulation_h(double x, double y, double z, double w) {
    triangulation_h[0]=x; triangulation_h[1]=y; triangulation_h[2]=z; triangulation_h[3]=w;
    triangulation_override=true;
}
void opencv_calib3d_test_triangulation_unknown(void) { triangulation_unknown=true; }
int opencv_calib3d_test_triangulation_live(void) {
    return opencv_calib3d_triangulation_result_handle::live;
}
#endif
opencv_calib3d_status opencv_calib3d_triangulate_normalized(
    const opencv_core_mat_handle *first_handle, const opencv_core_mat_handle *second_handle,
    const opencv_calib3d_relative_pose *pose,
    opencv_calib3d_triangulation_result_handle **result) {
    if (result) *result=nullptr;
    return guarded([&] {
        require(result && pose, "null triangulation result/pose");
        double r[3][3], t[3];
        require(opencv_calib3d_detail::triangulation_pose(*pose,r,t), "invalid relative pose");
        const cv::Mat &a=resolve_input(first_handle), &b=resolve_input(second_handle);
        const auto schema=[](const cv::Mat &m) {
            require(m.dims==2 && m.type()==CV_64FC2 && m.cols==1 && m.rows>=0,
                    "triangulation points must be typed Nx1 Float64 C2");
        };
        schema(a); schema(b);
        require(a.rows==b.rows, "triangulation counts differ");
        validate_finite_image_points(a); validate_finite_image_points(b);
        auto owned=std::make_unique<opencv_calib3d_triangulation_result_handle>();
        if (a.rows==0) { *result=owned.release(); return; }
        const cv::Mat first=a.clone(), second=b.clone();
        cv::Mat p1(3,4,CV_64FC1), p2(3,4,CV_64FC1);
        for (int i=0;i<3;++i) for (int j=0;j<4;++j) {
            p1.at<double>(i,j)=i==j ? 1.0 : 0.0;
            p2.at<double>(i,j)=j==3 ? t[i] : r[i][j];
        }
        cv::Mat points1(2,a.rows,CV_64FC1), points2(2,a.rows,CV_64FC1), h;
        for (int i=0;i<a.rows;++i) for (int j=0;j<2;++j) {
            points1.at<double>(j,i)=first.at<cv::Vec2d>(i,0)[j];
            points2.at<double>(j,i)=second.at<cv::Vec2d>(i,0)[j];
        }
        checkpoint(37);
        cv::triangulatePoints(p1,p2,points1,points2,h);
        checkpoint(38);
        if (h.dims!=2 || h.rows!=4 || h.cols!=a.rows || h.type()!=CV_64FC1)
            throw std::runtime_error("malformed native homogeneous triangulation output");
        checkpoint(39);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
        // Only replace the first column, after a real native call and schema check.
        if (triangulation_override) {
            triangulation_override=false;
            for (int j=0;j<4;++j) h.at<double>(j,0)=triangulation_h[j];
        }
#endif
        owned->points.reserve(static_cast<std::size_t>(a.rows));
        for (int i=0;i<a.rows;++i) {
            const double v[4]={h.at<double>(0,i),h.at<double>(1,i),h.at<double>(2,i),h.at<double>(3,i)};
            owned->points.push_back(opencv_calib3d_detail::classify_triangulation(v,r,t,
                points1.at<double>(0,i),points1.at<double>(1,i),
                points2.at<double>(0,i),points2.at<double>(1,i)));
        }
        checkpoint(40);
        *result=owned.release();
    });
}
opencv_calib3d_status opencv_calib3d_triangulation_result_count(
    const opencv_calib3d_triangulation_result_handle *result, int32_t *count) {
    if (count) *count=0;
    return guarded([&] {
        require(result && count,"null triangulation count argument");
        *count=static_cast<int32_t>(result->points.size());
    });
}
opencv_calib3d_status opencv_calib3d_triangulation_result_point(
    const opencv_calib3d_triangulation_result_handle *result, int32_t index,
    opencv_calib3d_triangulated_point *point) {
    if (point) *point={};
    return guarded([&] {
        require(result && point,"null triangulation point argument");
        require(index>=0 && static_cast<std::size_t>(index)<result->points.size(),
                "triangulation index out of range");
        checkpoint(41);
        *point=result->points[static_cast<std::size_t>(index)];
#ifdef OPENCV_CALIB3D_TEST_HOOKS
        if (triangulation_unknown) {
            triangulation_unknown=false;
            point->status=867;
        }
#endif
    });
}
void opencv_calib3d_triangulation_result_destroy(opencv_calib3d_triangulation_result_handle *result) {
    try { delete result; } catch (...) {}
}
} // extern "C"
