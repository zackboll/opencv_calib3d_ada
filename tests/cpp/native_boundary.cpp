#include "opencv_calib3d_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"

#include <opencv2/core.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <limits>
#include <stdexcept>

#ifdef OPENCV_CALIB3D_TEST_HOOKS
extern "C" void opencv_calib3d_test_fail(int stage, int kind);
extern "C" void opencv_calib3d_test_refinement_false(void);
#endif

namespace {
void check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
using Mat = std::unique_ptr<opencv_core_mat_handle, decltype(&opencv_core_mat_destroy)>;
using Result = std::unique_ptr<opencv_calib3d_pose_result_handle,
                               decltype(&opencv_calib3d_pose_result_destroy)>;

Mat matrix(int rows, int columns, int depth, int channels) {
    opencv_core_mat_handle *raw = nullptr;
    check(opencv_core_mat_create_2d(rows, columns, depth, channels, &raw) == OPENCV_CORE_OK,
          "Core matrix factory failed");
    return Mat(raw, opencv_core_mat_destroy);
}

cv::Mat &output(opencv_core_mat_handle *handle) {
    cv::Mat *mat = nullptr;
    check(opencv_core_module_output_mat(handle, &mat) == OPENCV_CORE_OK && mat,
          "Core output resolver failed");
    return *mat;
}

void fill_world(opencv_core_mat_handle *handle) {
    cv::Mat &m = output(handle);
    for (int i = 0; i < m.rows; ++i) {
        const double x = (i % 5 - 2) * 0.6;
        const double y = ((i / 5) % 4 - 1) * 0.7;
        const double z = 0.15 * (i % 7) + 0.05 * i;
        m.at<cv::Vec3d>(i,0) = cv::Vec3d(x,y,z);
    }
}

bool zero_pose(const opencv_calib3d_pose &p) {
    return p.rx == 0 && p.ry == 0 && p.rz == 0 && p.tx == 0 && p.ty == 0 && p.tz == 0;
}

void refinement_boundary() {
    auto world = matrix(20,1,OPENCV_CORE_DEPTH_FLOAT64,3);
    auto image = matrix(20,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto projected = matrix(20,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    fill_world(world.get());
    const opencv_calib3d_camera_intrinsics k{800,820,320,240};
    const opencv_calib3d_distortion5 none{};
    const opencv_calib3d_distortion5 distorted{0.1,-0.04,0.003,-0.002,0.01};
    const opencv_calib3d_pose truth{0.10,-0.05,0.08,0.20,-0.10,6.00};
    const opencv_calib3d_pose initial{0.16,-0.09,0.12,0.45,-0.30,6.40};
    uint8_t refined = 9;
    opencv_calib3d_pose pose{9,9,9,9,9,9};
    for (const auto &d : {none,distorted}) {
        check(opencv_calib3d_project_points(world.get(),&k,&d,&truth,image.get()) == 0,
              "refinement truth projection");
        check(opencv_calib3d_refine_pose_iterative(world.get(),image.get(),&k,&d,&initial,
              &refined,&pose) == 0 && refined == 1, "raw iterative refinement");
        check(opencv_calib3d_project_points(world.get(),&k,&d,&pose,projected.get()) == 0 &&
              cv::norm(output(image.get()),output(projected.get()),cv::NORM_INF) < 1e-7,
              "raw iterative refinement final pixel error");
        check(initial.rx == 0.16 && initial.ty == -0.30, "native changed initial pose");
    }
    auto invalid = [&](const opencv_core_mat_handle *o, const opencv_core_mat_handle *i,
                       const opencv_calib3d_camera_intrinsics *intrinsics,
                       const opencv_calib3d_distortion5 *d, const opencv_calib3d_pose *p,
                       bool flag = true, bool out_pose = true) {
        refined = 9; pose = {9,9,9,9,9,9};
        check(opencv_calib3d_refine_pose_iterative(o,i,intrinsics,d,p,
              flag ? &refined : nullptr,out_pose ? &pose : nullptr) ==
              OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT, "invalid refinement accepted");
        check((!flag || refined == 0) && (!out_pose || zero_pose(pose)),
              "refinement failure outputs not cleared");
    };
    invalid(nullptr,image.get(),&k,&none,&initial);
    invalid(world.get(),nullptr,&k,&none,&initial);
    invalid(world.get(),image.get(),nullptr,&none,&initial);
    invalid(world.get(),image.get(),&k,nullptr,&initial);
    invalid(world.get(),image.get(),&k,&none,nullptr);
    invalid(world.get(),image.get(),&k,&none,&initial,false,true);
    invalid(world.get(),image.get(),&k,&none,&initial,true,false);
    for (auto schema : {CV_32FC3,CV_64FC2}) {
        auto bad = matrix(20,1,CV_MAT_DEPTH(schema),CV_MAT_CN(schema));
        invalid(bad.get(),image.get(),&k,&none,&initial);
    }
    auto bad_object_shape = matrix(1,20,OPENCV_CORE_DEPTH_FLOAT64,3);
    invalid(bad_object_shape.get(),image.get(),&k,&none,&initial);
    for (auto schema : {CV_32FC2,CV_64FC3}) {
        auto bad = matrix(20,1,CV_MAT_DEPTH(schema),CV_MAT_CN(schema));
        invalid(world.get(),bad.get(),&k,&none,&initial);
    }
    auto bad_image_shape = matrix(1,20,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto mismatch = matrix(19,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto short_world = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,3);
    auto short_image = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    invalid(world.get(),bad_image_shape.get(),&k,&none,&initial);
    invalid(world.get(),mismatch.get(),&k,&none,&initial);
    invalid(short_world.get(),short_image.get(),&k,&none,&initial);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int component=0; component<6; ++component) {
        auto bad = initial;
        switch (component) {
        case 0: bad.rx=nan; break; case 1: bad.ry=nan; break; case 2: bad.rz=nan; break;
        case 3: bad.tx=nan; break; case 4: bad.ty=nan; break; default: bad.tz=nan;
        }
        invalid(world.get(),image.get(),&k,&none,&bad);
    }
    auto bad_k = k; bad_k.focal_x = 0;
    auto bad_d = none; bad_d.k3 = nan;
    invalid(world.get(),image.get(),&bad_k,&none,&initial);
    invalid(world.get(),image.get(),&k,&bad_d,&initial);
    const cv::Vec3d saved_object = output(world.get()).at<cv::Vec3d>(0,0);
    output(world.get()).at<cv::Vec3d>(0,0)[0] = nan;
    invalid(world.get(),image.get(),&k,&none,&initial);
    output(world.get()).at<cv::Vec3d>(0,0) = saved_object;
    const cv::Vec2d saved_image = output(image.get()).at<cv::Vec2d>(0,0);
    output(image.get()).at<cv::Vec2d>(0,0)[1] = nan;
    invalid(world.get(),image.get(),&k,&none,&initial);
    output(image.get()).at<cv::Vec2d>(0,0) = saved_image;
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    opencv_calib3d_test_refinement_false();
    refined=9; pose={9,9,9,9,9,9};
    check(opencv_calib3d_refine_pose_iterative(world.get(),image.get(),&k,&distorted,&initial,
          &refined,&pose) == 0 && refined == 0 && zero_pose(pose),
          "native refinement false result publication");
    const int expected[] = {0,1,2,3,4,4};
    for (int stage : {10,11,12}) for (int kind=1; kind<=5; ++kind) {
        refined=9; pose={9,9,9,9,9,9};
        opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_refine_pose_iterative(world.get(),image.get(),&k,&distorted,&initial,
              &refined,&pose) == expected[kind] && refined == 0 && zero_pose(pose),
              "refinement exception/publication atomicity");
    }
    std::cout << "PASS: refinement false-result and 15 exception checkpoints\n";
#endif
    std::cout << "PASS: iterative refinement/distortion/schema/output initialization raw ABI\n";
}

void run() {
    refinement_boundary();
    constexpr int N = 20;
    auto world = matrix(N, 1, OPENCV_CORE_DEPTH_FLOAT64, 3);
    auto image = matrix(N, 1, OPENCV_CORE_DEPTH_FLOAT64, 2);
    fill_world(world.get());

    const opencv_calib3d_camera_intrinsics k{800.0,820.0,320.0,240.0};
    const opencv_calib3d_distortion5 d{0.0,0.0,0.0,0.0,0.0};
    const opencv_calib3d_pose truth{0.10,-0.05,0.08,0.20,-0.10,6.00};

    check(opencv_calib3d_project_points(world.get(),&k,&d,&truth,image.get()) == 0,
          "projectPoints boundary failed");
    const cv::Mat &pixels = output(image.get());
    check(pixels.rows == N && pixels.cols == 1 && pixels.type() == CV_64FC2,
          "projectPoints output schema");

    opencv_calib3d_point3 center{};
    check(opencv_calib3d_camera_center(&truth,&center) == 0 &&
          std::isfinite(center.x) && std::isfinite(center.y) && std::isfinite(center.z),
          "camera center failed");
    check(opencv_calib3d_camera_center(&truth,nullptr) == OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT,
          "null camera-center output accepted");
    center = {9,9,9};
    check(opencv_calib3d_camera_center(nullptr,&center) == OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT &&
          center.x == 0 && center.y == 0 && center.z == 0, "null pose clearing");

    for (int i : {1,6,14}) output(image.get()).at<cv::Vec2d>(i,0) += cv::Vec2d(5000,-4000);

    opencv_calib3d_pose_result_handle *raw = nullptr;
    check(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,500,1.0,0.999,&raw) == 0 && raw,
          "solvePnPRansac boundary failed");
    Result result(raw, opencv_calib3d_pose_result_destroy);
    uint8_t found = 0;
    check(opencv_calib3d_pose_result_found(raw,&found) == 0 && found == 1,
          "synthetic pose not found");
    opencv_calib3d_pose pose{};
    check(opencv_calib3d_pose_result_pose(raw,&pose) == 0 && std::isfinite(pose.tz),
          "pose accessor failed");
    int32_t count = 0;
    check(opencv_calib3d_pose_result_inlier_count(raw,&count) == 0 && count >= 4 && count <= N,
          "inlier count failed");
    int32_t previous = -1;
    for (int32_t i = 0; i < count; ++i) {
        int32_t index = -1;
        check(opencv_calib3d_pose_result_inlier(raw,i,&index) == 0 && index > previous && index < N,
              "inlier accessor/order failed");
        check(index != 1 && index != 6 && index != 14, "gross outlier accepted");
        previous = index;
    }

    int32_t cleared = 99;
    check(opencv_calib3d_pose_result_inlier(raw,-1,&cleared) == OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT && cleared == 0,
          "negative inlier index clearing");
    cleared = 99;
    check(opencv_calib3d_pose_result_inlier(raw,count,&cleared) == OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT && cleared == 0,
          "end inlier index clearing");
    pose = {1,1,1,1,1,1};
    check(opencv_calib3d_pose_result_pose(nullptr,&pose) == OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT &&
          pose.rx == 0 && pose.tz == 0, "null result pose clearing");
    found = 7;
    check(opencv_calib3d_pose_result_found(nullptr,&found) == OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT && found == 0,
          "null result found clearing");
    count = 99;
    check(opencv_calib3d_pose_result_inlier_count(nullptr,&count) == 1 && count == 0,
          "null result count clearing");
    cleared = 99;
    check(opencv_calib3d_pose_result_inlier(nullptr,0,&cleared) == 1 && cleared == 0,
          "null result index clearing");
    check(opencv_calib3d_pose_result_found(raw,nullptr) == 1 &&
          opencv_calib3d_pose_result_pose(raw,nullptr) == 1 &&
          opencv_calib3d_pose_result_inlier_count(raw,nullptr) == 1 &&
          opencv_calib3d_pose_result_inlier(raw,0,nullptr) == 1,
          "null accessor outputs accepted");

    opencv_calib3d_pose_result_handle *sentinel = raw;
    auto fail = [&](opencv_calib3d_status status) {
        check(status != 0 && sentinel == nullptr, "solve failure did not clear result output");
        sentinel = raw;
    };
    fail(opencv_calib3d_solve_pnp_ransac(nullptr,image.get(),&k,&d,100,1.0,0.99,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),nullptr,&k,&d,100,1.0,0.99,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,0,1.0,0.99,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,0.0,0.99,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,1.0,1.0,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,-1,1.0,0.99,&sentinel));
    for (double threshold : {1e-300, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN(), 1e100})
        fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,threshold,0.99,&sentinel));
    for (double confidence : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN()})
        fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,1.0,confidence,&sentinel));
    check(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,1.0,0.99,nullptr) ==
          OPENCV_CALIB3D_ERROR_INVALID_ARGUMENT, "null solve output accepted");

    auto wrong_object = matrix(N,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    sentinel = raw;
    fail(opencv_calib3d_solve_pnp_ransac(wrong_object.get(),image.get(),&k,&d,100,1.0,0.99,&sentinel));

    auto wrong_depth = matrix(N,1,OPENCV_CORE_DEPTH_FLOAT32,3);
    auto wrong_shape = matrix(1,N,OPENCV_CORE_DEPTH_FLOAT64,3);
    auto wrong_image = matrix(N,1,OPENCV_CORE_DEPTH_FLOAT64,3);
    auto wrong_image_depth = matrix(N,1,OPENCV_CORE_DEPTH_FLOAT32,2);
    auto wrong_image_shape = matrix(1,N,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto mismatch = matrix(N-1,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto short_world = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,3);
    auto short_image = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    for (auto *bad : {wrong_depth.get(), wrong_shape.get()})
        fail(opencv_calib3d_solve_pnp_ransac(bad,image.get(),&k,&d,100,1.0,0.99,&sentinel));
    for (auto *bad : {wrong_image.get(),wrong_image_depth.get(),wrong_image_shape.get(),mismatch.get()})
        fail(opencv_calib3d_solve_pnp_ransac(world.get(),bad,&k,&d,100,1.0,0.99,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(short_world.get(),short_image.get(),&k,&d,100,1.0,0.99,&sentinel));
    for (double invalid : {0.0,-1.0,std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        auto bad = k;
        bad.focal_x = invalid;
        fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&bad,&d,100,1.0,0.99,&sentinel));
        bad = k; bad.focal_y = invalid;
        fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&bad,&d,100,1.0,0.99,&sentinel));
    }
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),nullptr,&d,100,1.0,0.99,&sentinel));
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,nullptr,100,1.0,0.99,&sentinel));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int member=0; member<5; ++member) {
        auto bad = d;
        switch (member) { case 0: bad.k1=nan; break; case 1: bad.k2=nan; break;
            case 2: bad.p1=nan; break; case 3: bad.p2=nan; break; default: bad.k3=nan; }
        fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&bad,100,1.0,0.99,&sentinel));
    }
    check(opencv_calib3d_project_points(nullptr,&k,&d,&truth,image.get()) == 1 &&
          opencv_calib3d_project_points(world.get(),&k,&d,&truth,nullptr) == 1 &&
          opencv_calib3d_project_points(world.get(),&k,&d,nullptr,image.get()) == 1,
          "null project arguments accepted");
    for (auto *bad : {wrong_object.get(),wrong_depth.get(),wrong_shape.get()})
        check(opencv_calib3d_project_points(bad,&k,&d,&truth,image.get()) == 1,
              "invalid project schema accepted");
    const auto saved = output(world.get()).at<cv::Vec3d>(0,0);
    output(world.get()).at<cv::Vec3d>(0,0)[0] = nan;
    fail(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,1.0,0.99,&sentinel));
    output(world.get()).at<cv::Vec3d>(0,0) = saved;

    auto inconsistent_image = matrix(N,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    for (int i=0;i<N;++i)
        output(inconsistent_image.get()).at<cv::Vec2d>(i,0) =
            cv::Vec2d(((i+1)*7919)%997,((i+1)*104729)%991);
    opencv_calib3d_pose_result_handle *absent = nullptr;
    check(opencv_calib3d_solve_pnp_ransac(world.get(),inconsistent_image.get(),
          &k,&d,100,1e-6,0.99,&absent) == 0 && absent, "native false became ABI error");
    Result no_pose(absent,opencv_calib3d_pose_result_destroy);
    found=9; count=99; pose={9,9,9,9,9,9};
    check(opencv_calib3d_pose_result_found(absent,&found) == 0 && found == 0 &&
          opencv_calib3d_pose_result_inlier_count(absent,&count) == 0 && count == 0 &&
          opencv_calib3d_pose_result_pose(absent,&pose) == 1 && pose.rx == 0 && pose.tz == 0,
          "not-found result contract");

#ifdef OPENCV_CALIB3D_TEST_HOOKS
    const int expected[] = {0,1,2,3,4,4};
    for (int stage : {1,2}) for (int kind=1; kind<=5; ++kind) {
        const cv::Mat before = output(image.get()).clone();
        opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_project_points(world.get(),&k,&d,&truth,image.get()) == expected[kind],
              "project exception barrier");
        check(cv::norm(before,output(image.get()),cv::NORM_INF) == 0,
              "failed projection modified caller output");
    }
    for (int stage : {3,4}) for (int kind=1; kind<=5; ++kind) {
        center = {9,9,9};
        opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_camera_center(&truth,&center) == expected[kind] &&
              center.x == 0 && center.y == 0 && center.z == 0,
              "camera-center exception clearing");
    }
    for (int stage : {5,6,7}) for (int kind=1; kind<=5; ++kind) {
        sentinel = raw;
        opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_solve_pnp_ransac(world.get(),image.get(),&k,&d,100,1.0,0.99,&sentinel) == expected[kind] &&
              sentinel == nullptr, "solve exception/publication atomicity");
    }
    pose = {9,9,9,9,9,9};
    opencv_calib3d_test_fail(8,3);
    check(opencv_calib3d_pose_result_pose(raw,&pose) == OPENCV_CALIB3D_ERROR_ALLOCATION && pose.rx == 0,
          "pose getter exception clearing");
    cleared = 99;
    opencv_calib3d_test_fail(9,3);
    check(opencv_calib3d_pose_result_inlier(raw,0,&cleared) == OPENCV_CALIB3D_ERROR_ALLOCATION && cleared == 0,
          "inlier getter exception clearing");
#endif

    opencv_calib3d_pose_result_destroy(nullptr);
    std::cout << "PASS: Calib3D projectPoints/camera-center/solvePnPRansac raw boundary on "
              << opencv_calib3d_native_version() << " / " << opencv_calib3d_native_backend() << '\n';
}
} // namespace

int main() {
    try {
        run();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
