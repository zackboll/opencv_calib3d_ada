#include "opencv_calib3d_shim.h"
#include "opencv_core_shim.h"
#include "opencv_core_module_bridge.hpp"
#include "homography_profile.hpp"
#include "fundamental_profile.hpp"
#include "essential_profile.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <limits>
#include <stdexcept>

#ifdef OPENCV_CALIB3D_TEST_HOOKS
extern "C" void opencv_calib3d_test_fail(int stage, int kind);
extern "C" void opencv_calib3d_test_refinement_false(void);
extern "C" void opencv_calib3d_test_essential_no_model(void);
extern "C" void opencv_calib3d_test_essential_no_pose(void);
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
              cv::norm(output(image.get()),output(projected.get()),cv::NORM_INF) < 1e-5,
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

void camera_geometry_boundary() {
    const opencv_calib3d_camera_intrinsics k{100,200,10,20};
    const opencv_calib3d_distortion5 none{};
    const opencv_calib3d_distortion5 d{0.1,-0.04,0.003,-0.002,0.01};
    const opencv_calib3d_pose identity{};
    const opencv_calib3d_pose quarter{0,0,std::acos(-1.0)/2,1,2,3};
    auto input = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto destination = matrix(2,2,OPENCV_CORE_DEPTH_FLOAT64,1);
    const cv::Vec2d known[] = {{0,0},{1,0},{0.3,-0.2}};
    for (const auto &dist : {none,d}) {
        for (int i=0;i<3;++i) {
            const double x=known[i][0], y=known[i][1];
            const double r2=x*x+y*y, radial=1+dist.k1*r2+dist.k2*r2*r2+dist.k3*r2*r2*r2;
            output(input.get()).at<cv::Vec2d>(i,0) = cv::Vec2d(
                k.focal_x*(x*radial+2*dist.p1*x*y+dist.p2*(r2+2*x*x))+k.center_x,
                k.focal_y*(y*radial+dist.p1*(r2+2*y*y)+2*dist.p2*x*y)+k.center_y);
        }
        check(opencv_calib3d_undistort_normalized(input.get(),&k,&dist,destination.get()) == 0,
              "raw undistortion");
        const auto &m=output(destination.get());
        check(m.rows==3 && m.cols==1 && m.type()==CV_64FC2, "raw normalized schema");
        for (int i=0;i<3;++i)
            check(cv::norm(m.at<cv::Vec2d>(i,0)-known[i]) < 1e-10, "independent Brown inversion");
    }
    auto region = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    cv::Mat parent(3,2,CV_64FC2);
    output(input.get()).copyTo(parent.col(0));
    output(region.get()) = parent.col(0);
    check(!output(region.get()).isContinuous(), "fixture must be strided");
    check(opencv_calib3d_undistort_normalized(region.get(),&k,&d,destination.get())==0 &&
          cv::norm(output(destination.get()).at<cv::Vec2d>(2,0)-known[2])<1e-10,
          "strided Core region undistortion");
    auto alias = matrix(3,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    output(input.get()).copyTo(output(alias.get()));
    check(opencv_calib3d_undistort_normalized(alias.get(),&k,&d,alias.get())==0 &&
          cv::norm(output(alias.get()).at<cv::Vec2d>(2,0)-known[2])<1e-10,
          "aliased Core input/output publication");
    auto empty = matrix(1,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    output(empty.get()).release();
    check(opencv_calib3d_undistort_normalized(empty.get(),&k,&d,destination.get())==0 &&
          output(destination.get()).empty(), "raw empty undistortion");

    auto invalid = [&](const opencv_core_mat_handle *in,
                       const opencv_calib3d_camera_intrinsics *intrinsics,
                       const opencv_calib3d_distortion5 *dist, bool have_output=true) {
        cv::Mat &target=output(destination.get());
        target=cv::Mat(2,3,CV_64FC1,cv::Scalar(42));
        const cv::Mat saved=target.clone();
        const auto *data=target.data;
        check(opencv_calib3d_undistort_normalized(in,intrinsics,dist,
              have_output ? destination.get() : nullptr)==1, "invalid undistort accepted");
        check(target.data==data && target.size()==saved.size() && target.type()==saved.type() &&
              cv::norm(target,saved,cv::NORM_INF)==0, "failed undistort published output");
    };
    invalid(nullptr,&k,&d); invalid(input.get(),nullptr,&d);
    invalid(input.get(),&k,nullptr); invalid(input.get(),&k,&d,false);
    for (int channels : {1,2,3}) {
        auto bad=matrix(3,1,channels==2 ? OPENCV_CORE_DEPTH_FLOAT32 : OPENCV_CORE_DEPTH_FLOAT64,channels);
        invalid(bad.get(),&k,&d);
    }
    auto bad_shape=matrix(2,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    invalid(bad_shape.get(),&k,&d);
    const int sizes[]={2,2,2};
    output(bad_shape.get())=cv::Mat(3,sizes,CV_64FC2,cv::Scalar(0,0));
    invalid(bad_shape.get(),&k,&d);
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for (double nonfinite : {nan,std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()}) {
        const auto saved=output(input.get()).at<cv::Vec2d>(0,0);
        output(input.get()).at<cv::Vec2d>(0,0)[0]=nonfinite;
        invalid(input.get(),&k,&d);
        output(input.get()).at<cv::Vec2d>(0,0)=saved;
        auto bad_k=k; bad_k.center_y=nonfinite; invalid(input.get(),&bad_k,&d);
        auto bad_d=d; bad_d.k3=nonfinite; invalid(input.get(),&k,&bad_d);
    }
    auto bad_k=k; bad_k.focal_x=0; invalid(input.get(),&bad_k,&d);
    bad_k=k; bad_k.focal_y=-1; invalid(input.get(),&bad_k,&d);
    // Finite input that makes native output nonfinite must not publish either.
    bad_k=k; bad_k.focal_x=std::numeric_limits<double>::min();
    const auto saved_pixel=output(input.get()).at<cv::Vec2d>(0,0);
    output(input.get()).at<cv::Vec2d>(0,0)[0]=std::numeric_limits<double>::max();
    invalid(input.get(),&bad_k,&none);
    output(input.get()).at<cv::Vec2d>(0,0)=saved_pixel;

    opencv_calib3d_rotation_matrix rotation{};
    check(opencv_calib3d_rotation_matrix_of(&identity,&rotation)==0 &&
          rotation.m00==1 && rotation.m11==1 && rotation.m22==1 && rotation.m01==0,
          "raw Rodrigues identity");
    check(opencv_calib3d_rotation_matrix_of(&quarter,&rotation)==0 &&
          std::abs(rotation.m00)<1e-14 && std::abs(rotation.m01+1)<1e-14 &&
          std::abs(rotation.m10-1)<1e-14 && rotation.m22==1, "raw Rodrigues Rz(pi/2)");
    auto zero_rotation = [&] {
        return rotation.m00==0 && rotation.m01==0 && rotation.m02==0 &&
               rotation.m10==0 && rotation.m11==0 && rotation.m12==0 &&
               rotation.m20==0 && rotation.m21==0 && rotation.m22==0;
    };
    auto invalid_rotation = [&](const opencv_calib3d_pose *pose) {
        rotation={9,9,9,9,9,9,9,9,9};
        check(opencv_calib3d_rotation_matrix_of(pose,&rotation)==1 && zero_rotation(),
              "invalid Rodrigues clearing");
    };
    invalid_rotation(nullptr);
    check(opencv_calib3d_rotation_matrix_of(&quarter,nullptr)==1, "null Rodrigues output");
    for (double nonfinite : {nan,std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()}) {
        auto p=quarter; p.rx=nonfinite; invalid_rotation(&p);
        p=quarter; p.tz=nonfinite; invalid_rotation(&p);
    }
    auto huge_rotation=quarter; huge_rotation.rx=std::numeric_limits<double>::max();
    invalid_rotation(&huge_rotation);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    const int expected[]={0,1,2,3,4,4};
    for (int stage : {13,14}) for (int kind=1;kind<=5;++kind) {
        auto &target=output(destination.get());
        target=cv::Mat(2,3,CV_64FC1,cv::Scalar(42));
        const cv::Mat before=target.clone();
        const auto *data=target.data;
        opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_undistort_normalized(region.get(),&k,&d,destination.get())==expected[kind],
              "undistortion exception mapping");
        check(target.data==data && target.size()==before.size() && target.type()==before.type() &&
              cv::norm(target,before,cv::NORM_INF)==0, "undistort exception publication");
    }
    for (int stage : {15,16}) for (int kind=1;kind<=5;++kind) {
        rotation={9,9,9,9,9,9,9,9,9};
        opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_rotation_matrix_of(&quarter,&rotation)==expected[kind] && zero_rotation(),
              "Rodrigues exception clearing");
    }
    std::cout << "PASS: camera geometry 20 exception checkpoints; output failure-atomic\n";
#endif
    std::cout << "PASS: normalized zero/five-coefficient inversion, strided ROI, Rodrigues identity/Rz; raw negatives\n";
}

void homography_boundary() {
    using HResult = std::unique_ptr<opencv_calib3d_homography_result_handle,
                                    decltype(&opencv_calib3d_homography_result_destroy)>;
    constexpr int n=24;
    auto source_parent=matrix(n,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto destination_parent=matrix(n,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    opencv_core_mat_handle *sraw=nullptr, *draw=nullptr;
    check(opencv_core_mat_region(source_parent.get(),0,0,1,n,&sraw)==0 &&
          opencv_core_mat_region(destination_parent.get(),0,0,1,n,&draw)==0,
          "real Core homography Regions");
    Mat source(sraw,opencv_core_mat_destroy), destination(draw,opencv_core_mat_destroy);
    check(!output(sraw).isContinuous() && !output(draw).isContinuous(), "homography Regions strided");
    for (int i=0;i<n;++i) {
        const double x=(i%6)*40.0-100, y=(i/6)*35.0-50;
        const double w=.001*x-.002*y+1;
        output(sraw).at<cv::Vec2d>(i,0)={x,y};
        output(draw).at<cv::Vec2d>(i,0)={(1.2*x+.1*y+20)/w,(-.05*x+.9*y+10)/w};
    }
    const opencv_calib3d_homography_options options{2000,.1,.999};
    auto estimate=[&](const opencv_core_mat_handle *s,const opencv_core_mat_handle *d) {
        opencv_calib3d_homography_result_handle *raw=nullptr;
        check(opencv_calib3d_find_homography_ransac(s,d,&options,&raw)==0 && raw,
              "raw homography estimation");
        return HResult(raw,opencv_calib3d_homography_result_destroy);
    };
    auto verify=[&](HResult &r,bool outliers) {
        uint8_t found=0; int32_t count=0;
        opencv_calib3d_homography h{};
        check(opencv_calib3d_homography_result_found(r.get(),&found)==0 && found==1 &&
              opencv_calib3d_homography_result_matrix(r.get(),&h)==0 &&
              opencv_calib3d_homography_result_inlier_count(r.get(),&count)==0 && count>=4,
              "raw homography accessors");
        bool published[n]={}; int32_t previous=-1;
        for (int i=0;i<count;++i) {
            int32_t index=-1;
            check(opencv_calib3d_homography_result_inlier(r.get(),i,&index)==0 &&
                  index>previous && index<n, "homography indices valid unique ascending");
            published[index]=true; previous=index;
        }
        double maximum_error=0;
        for (int i=0;i<n;++i) {
            const auto s=output(sraw).at<cv::Vec2d>(i,0), d=output(draw).at<cv::Vec2d>(i,0);
            const double w=h.h20*s[0]+h.h21*s[1]+h.h22;
            const double xp=(h.h00*s[0]+h.h01*s[1]+h.h02)/w;
            const double yp=(h.h10*s[0]+h.h11*s[1]+h.h12)/w;
            const double error=std::hypot(xp-d[0],yp-d[1]);
            check(published[i]==(std::isfinite(xp) && std::isfinite(yp) && error<=.1),
                  "every public inclusion/exclusion independently matches FINAL H Float64 threshold");
            if (!outliers) maximum_error=std::max(maximum_error,error);
        }
        if (outliers) check(!published[1] && !published[6] && !published[14], "gross outliers survived");
        else check(count==n && maximum_error<1e-3, "clean homography mapping oracle");
        int32_t index=99;
        check(opencv_calib3d_homography_result_inlier(r.get(),-1,&index)==1 && index==0 &&
              opencv_calib3d_homography_result_inlier(r.get(),count,&index)==1 && index==0,
              "homography invalid index clearing");
        std::cout << "PASS: raw " << (outliers?"outlier":"clean") << " homography final inliers="
                  << count << " clean maximum error=" << maximum_error << "; FINAL H threshold verified\n";
    };
    auto clean=estimate(sraw,draw); verify(clean,false);
    for (int i : {1,6,14}) output(draw).at<cv::Vec2d>(i,0)+=cv::Vec2d(5000,-4000);
    auto robust=estimate(sraw,draw); verify(robust,true);
    auto invalid=[&](const opencv_core_mat_handle *s,const opencv_core_mat_handle *d,
                     const opencv_calib3d_homography_options *o) {
        auto *raw=clean.get(); // live result, not a fabricated opaque handle
        check(opencv_calib3d_find_homography_ransac(s,d,o,&raw)==1 && raw==nullptr,
              "invalid homography arguments/output clearing");
    };
    invalid(nullptr,draw,&options); invalid(sraw,nullptr,&options); invalid(sraw,draw,nullptr);
    check(opencv_calib3d_find_homography_ransac(sraw,draw,&options,nullptr)==1,"null homography output");
    for (int count=0;count<=4;++count) {
        auto bad=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT64,2);
        if (count==0) output(bad.get()).release();
        else output(bad.get())=output(sraw).rowRange(0,count);
        invalid(bad.get(),bad.get(),&options);
    }
    auto five_s=matrix(5,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto five_d=matrix(5,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    for (int i=0;i<5;++i) {
        const int j[]={0,5,18,23,10};
        output(five_s.get()).at<cv::Vec2d>(i,0)=output(sraw).at<cv::Vec2d>(j[i],0);
        output(five_d.get()).at<cv::Vec2d>(i,0)=output(draw).at<cv::Vec2d>(j[i],0);
    }
    auto five=estimate(five_s.get(),five_d.get());
    uint8_t found=0;
    check(opencv_calib3d_homography_result_found(five.get(),&found)==0 && found==1,"five robust permitted");
    invalid(five_s.get(),draw,&options);
    for (int channels : {1,2,3}) {
        auto bad=matrix(n,1,channels==2?OPENCV_CORE_DEPTH_FLOAT32:OPENCV_CORE_DEPTH_FLOAT64,channels);
        invalid(bad.get(),draw,&options); invalid(sraw,bad.get(),&options);
    }
    auto shape=matrix(2,3,OPENCV_CORE_DEPTH_FLOAT64,2);
    invalid(shape.get(),draw,&options); invalid(sraw,shape.get(),&options);
    const int sizes[]={2,2,2}; output(shape.get())=cv::Mat(3,sizes,CV_64FC2);
    invalid(shape.get(),draw,&options); invalid(sraw,shape.get(),&options);
    auto bad=options;
    for (int iterations : {0,-1}) {bad=options;bad.maximum_iterations=iterations;invalid(sraw,draw,&bad);}
    for (double v : {0.,-1.,1e-300,std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()}) {
        bad=options;bad.reprojection_threshold_pixels=v;invalid(sraw,draw,&bad);
    }
    for (double v : {0.,-1.,1.,2.,std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
        bad=options;bad.confidence=v;invalid(sraw,draw,&bad);
    }
    for (auto *handle : {sraw,draw}) for (double v : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
        const auto saved=output(handle).at<cv::Vec2d>(0,0);
        output(handle).at<cv::Vec2d>(0,0)[0]=v; invalid(sraw,draw,&options);
        output(handle).at<cv::Vec2d>(0,0)=saved;
    }
    opencv_calib3d_homography h{9,9,9,9,9,9,9,9,9};
    auto zero=[&] {return h.h00==0 && h.h01==0 && h.h02==0 && h.h10==0 && h.h11==0 &&
                         h.h12==0 && h.h20==0 && h.h21==0 && h.h22==0;};
    int32_t count=99,index=99; found=9;
    check(opencv_calib3d_homography_result_found(nullptr,&found)==1 && found==0 &&
          opencv_calib3d_homography_result_matrix(nullptr,&h)==1 && zero() &&
          opencv_calib3d_homography_result_inlier_count(nullptr,&count)==1 && count==0 &&
          opencv_calib3d_homography_result_inlier(nullptr,0,&index)==1 && index==0,
          "homography null handle output clearing");
    check(opencv_calib3d_homography_result_found(clean.get(),nullptr)==1 &&
          opencv_calib3d_homography_result_matrix(clean.get(),nullptr)==1 &&
          opencv_calib3d_homography_result_inlier_count(clean.get(),nullptr)==1 &&
          opencv_calib3d_homography_result_inlier(clean.get(),0,nullptr)==1,"null homography accessor outputs");
    auto line_s=matrix(n,1,OPENCV_CORE_DEPTH_FLOAT64,2), line_d=matrix(n,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    for (int i=0;i<n;++i) {
        output(line_s.get()).at<cv::Vec2d>(i,0)={double(i),2.*i};
        output(line_d.get()).at<cv::Vec2d>(i,0)={3.*i,4.*i};
    }
    auto absent=estimate(line_s.get(),line_d.get()); found=9;count=99;h={9,9,9,9,9,9,9,9,9};
    check(opencv_calib3d_homography_result_found(absent.get(),&found)==0 && found==0 &&
          opencv_calib3d_homography_result_inlier_count(absent.get(),&count)==0 && count==0 &&
          opencv_calib3d_homography_result_matrix(absent.get(),&h)==1 && zero(),"collinear no-model contract");
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    const int expected[]={0,1,2,3,4,4};
    for (int stage : {17,18,19,20}) for (int kind=1;kind<=5;++kind) {
        auto *raw=clean.get(); opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_find_homography_ransac(sraw,draw,&options,&raw)==expected[kind] && !raw,
              "homography exception/publication atomicity");
    }
    for (int kind=1;kind<=5;++kind) {
        h={9,9,9,9,9,9,9,9,9};opencv_calib3d_test_fail(21,kind);
        check(opencv_calib3d_homography_result_matrix(clean.get(),&h)==expected[kind] && zero(),
              "homography matrix fault clearing");
        index=99;opencv_calib3d_test_fail(22,kind);
        check(opencv_calib3d_homography_result_inlier(clean.get(),0,&index)==expected[kind] && index==0,
              "homography inlier fault clearing");
    }
    std::cout << "PASS: homography 30 fault scenarios, null output/no partial publication\n";
#endif
    opencv_calib3d_homography_result_destroy(nullptr);
    std::cout << "PASS: homography real strided Core Regions, four rejected/five robust, collinear no-model, raw negatives\n";
}

void fundamental_boundary() {
    using FResult = std::unique_ptr<opencv_calib3d_fundamental_result_handle,
                                    decltype(&opencv_calib3d_fundamental_result_destroy)>;
    constexpr int n=32;
    auto source_parent=matrix(n,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto destination_parent=matrix(n,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    opencv_core_mat_handle *sraw=nullptr, *draw=nullptr;
    check(opencv_core_mat_region(source_parent.get(),0,0,1,n,&sraw)==0 &&
          opencv_core_mat_region(destination_parent.get(),0,0,1,n,&draw)==0,
          "real Core fundamental Regions");
    Mat source(sraw,opencv_core_mat_destroy), destination(draw,opencv_core_mat_destroy);
    check(!output(sraw).isContinuous() && !output(draw).isContinuous(), "fundamental Regions strided");
    for (int i=0;i<n;++i) {
        const double x=((i*7)%17-8)*.24, y=((i*11)%19-9)*.19;
        const double z=4.+((i*13)%23)*.17;
        output(sraw).at<cv::Vec2d>(i,0)={800*x/z+320,820*y/z+240};
        output(draw).at<cv::Vec2d>(i,0)={800*(x-.75)/z+320,820*y/z+240};
    }
    const opencv_calib3d_fundamental_options options{.1,.999};
    auto estimate=[&](const opencv_core_mat_handle *s,const opencv_core_mat_handle *d) {
        opencv_calib3d_fundamental_result_handle *raw=nullptr;
        check(opencv_calib3d_find_fundamental_ransac(s,d,&options,&raw)==0 && raw,
              "raw fundamental estimation");
        return FResult(raw,opencv_calib3d_fundamental_result_destroy);
    };
    auto verify=[&](FResult &r,bool outliers) {
        uint8_t found=0; int32_t count=0;
        opencv_calib3d_fundamental h{};
        check(opencv_calib3d_fundamental_result_found(r.get(),&found)==0 && found==1 &&
              opencv_calib3d_fundamental_result_matrix(r.get(),&h)==0 &&
              opencv_calib3d_fundamental_result_inlier_count(r.get(),&count)==0 && count>=7,
              "raw fundamental accessors");
        bool published[n]={}; int32_t previous=-1;
        for (int i=0;i<count;++i) {
            int32_t index=-1;
            check(opencv_calib3d_fundamental_result_inlier(r.get(),i,&index)==0 &&
                  index>previous && index<n, "fundamental indices valid unique ascending");
            published[index]=true; previous=index;
        }
        double maximum_error=0;
        for (int i=0;i<n;++i) {
            const auto s=output(sraw).at<cv::Vec2d>(i,0), d=output(draw).at<cv::Vec2d>(i,0);
            // Independent scalar oracle for this bounded fixture.
            const double a2=h.f00*s[0]+h.f01*s[1]+h.f02;
            const double b2=h.f10*s[0]+h.f11*s[1]+h.f12;
            const double c2=h.f20*s[0]+h.f21*s[1]+h.f22;
            const double a1=h.f00*d[0]+h.f10*d[1]+h.f20;
            const double b1=h.f01*d[0]+h.f11*d[1]+h.f21;
            const double c1=h.f02*d[0]+h.f12*d[1]+h.f22;
            const double error=std::max(std::abs(a1*s[0]+b1*s[1]+c1)/std::hypot(a1,b1),
                                       std::abs(a2*d[0]+b2*d[1]+c2)/std::hypot(a2,b2));
            check(published[i]==(std::isfinite(error) && error<=.1),
                  "every inclusion/exclusion independently matches FINAL F Float64 threshold");
            if (!outliers) maximum_error=std::max(maximum_error,error);
        }
        if (outliers) check(!published[1] && !published[6] && !published[14], "gross outliers survived");
        else check(count>=n-1 && maximum_error<.1, "clean fundamental mapping oracle");
        int32_t index=99;
        check(opencv_calib3d_fundamental_result_inlier(r.get(),-1,&index)==1 && index==0 &&
              opencv_calib3d_fundamental_result_inlier(r.get(),count,&index)==1 && index==0,
              "fundamental invalid index clearing");
        std::cout << "PASS: raw " << (outliers?"outlier":"clean") << " fundamental final inliers="
                  << count << " clean maximum error=" << maximum_error << "; FINAL F threshold verified\n";
    };
    auto clean=estimate(sraw,draw); verify(clean,false);
    for (int i : {1,6,14}) output(draw).at<cv::Vec2d>(i,0)+=cv::Vec2d(50,500);
    auto robust=estimate(sraw,draw); verify(robust,true);
    auto invalid=[&](const opencv_core_mat_handle *s,const opencv_core_mat_handle *d,
                     const opencv_calib3d_fundamental_options *o) {
        auto *raw=clean.get(); // live result, not a fabricated opaque handle
        check(opencv_calib3d_find_fundamental_ransac(s,d,o,&raw)==1 && raw==nullptr,
              "invalid fundamental arguments/output clearing");
    };
    invalid(nullptr,draw,&options); invalid(sraw,nullptr,&options); invalid(sraw,draw,nullptr);
    check(opencv_calib3d_find_fundamental_ransac(sraw,draw,&options,nullptr)==1,"null fundamental output");
    for (int count=0;count<=14;++count) {
        auto bad=matrix(1,1,OPENCV_CORE_DEPTH_FLOAT64,2);
        if (count==0) output(bad.get()).release();
        else output(bad.get())=output(sraw).rowRange(0,count);
        invalid(bad.get(),bad.get(),&options);
    }
    auto fifteen_s=matrix(15,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto fifteen_d=matrix(15,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    output(sraw).rowRange(0,15).copyTo(output(fifteen_s.get()));
    output(draw).rowRange(0,15).copyTo(output(fifteen_d.get()));
    auto fifteen=estimate(fifteen_s.get(),fifteen_d.get());
    uint8_t found=0;
    check(opencv_calib3d_fundamental_result_found(fifteen.get(),&found)==0,
          "fifteen native RANSAC permitted");
    invalid(fifteen_s.get(),draw,&options);
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    auto fourteen=matrix(14,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    opencv_calib3d_test_fail(23,3);
    invalid(fourteen.get(),fourteen.get(),&options);
    auto *pending=clean.get();
    check(opencv_calib3d_find_fundamental_ransac(fifteen_s.get(),fifteen_d.get(),&options,&pending)==3 && !pending,
          "14 preflight preserved checkpoint; 15 consumed native-RANSAC checkpoint");
    std::cout << "PASS: raw 14 before native entry / 15 true-RANSAC checkpoint consumed\n";
#endif
    for (int channels : {1,2,3}) {
        auto bad=matrix(n,1,channels==2?OPENCV_CORE_DEPTH_FLOAT32:OPENCV_CORE_DEPTH_FLOAT64,channels);
        invalid(bad.get(),draw,&options); invalid(sraw,bad.get(),&options);
    }
    auto shape=matrix(2,3,OPENCV_CORE_DEPTH_FLOAT64,2);
    invalid(shape.get(),draw,&options); invalid(sraw,shape.get(),&options);
    const int sizes[]={2,2,2}; output(shape.get())=cv::Mat(3,sizes,CV_64FC2);
    invalid(shape.get(),draw,&options); invalid(sraw,shape.get(),&options);
    auto bad=options;
    for (double v : {0.,-1.,1e-30,1e20,1e-300,std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()}) {
        bad=options;bad.epipolar_threshold_pixels=v;invalid(sraw,draw,&bad);
    }
    for (double v : {0.,-1.,1.,2.,std::nextafter(DBL_EPSILON,0.),
                     std::nextafter(1-DBL_EPSILON,1.),std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
        bad=options;bad.confidence=v;invalid(sraw,draw,&bad);
    }
    for (auto *handle : {sraw,draw}) for (double v : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
        const auto saved=output(handle).at<cv::Vec2d>(0,0);
        output(handle).at<cv::Vec2d>(0,0)[0]=v; invalid(sraw,draw,&options);
        output(handle).at<cv::Vec2d>(0,0)=saved;
    }
    opencv_calib3d_fundamental h{9,9,9,9,9,9,9,9,9};
    auto zero=[&] {return h.f00==0 && h.f01==0 && h.f02==0 && h.f10==0 && h.f11==0 &&
                         h.f12==0 && h.f20==0 && h.f21==0 && h.f22==0;};
    int32_t count=99,index=99; found=9;
    check(opencv_calib3d_fundamental_result_found(nullptr,&found)==1 && found==0 &&
          opencv_calib3d_fundamental_result_matrix(nullptr,&h)==1 && zero() &&
          opencv_calib3d_fundamental_result_inlier_count(nullptr,&count)==1 && count==0 &&
          opencv_calib3d_fundamental_result_inlier(nullptr,0,&index)==1 && index==0,
          "fundamental null handle output clearing");
    check(opencv_calib3d_fundamental_result_found(clean.get(),nullptr)==1 &&
          opencv_calib3d_fundamental_result_matrix(clean.get(),nullptr)==1 &&
          opencv_calib3d_fundamental_result_inlier_count(clean.get(),nullptr)==1 &&
          opencv_calib3d_fundamental_result_inlier(clean.get(),0,nullptr)==1,"null fundamental accessor outputs");
    auto line_s=matrix(n,1,OPENCV_CORE_DEPTH_FLOAT64,2), line_d=matrix(n,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    for (int i=0;i<n;++i) {
        output(line_s.get()).at<cv::Vec2d>(i,0)={double(i),2.*i};
        output(line_d.get()).at<cv::Vec2d>(i,0)={3.*i,4.*i};
    }
    auto absent=estimate(line_s.get(),line_d.get()); found=9;count=99;h={9,9,9,9,9,9,9,9,9};
    check(opencv_calib3d_fundamental_result_found(absent.get(),&found)==0 && found==0 &&
          opencv_calib3d_fundamental_result_inlier_count(absent.get(),&count)==0 && count==0 &&
          opencv_calib3d_fundamental_result_matrix(absent.get(),&h)==1 && zero(),"collinear no-model contract");
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    const int expected[]={0,1,2,3,4,4};
    for (int stage : {23,24,25,26}) for (int kind=1;kind<=5;++kind) {
        auto *raw=clean.get(); opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_find_fundamental_ransac(sraw,draw,&options,&raw)==expected[kind] && !raw,
              "fundamental exception/publication atomicity");
    }
    for (int kind=1;kind<=5;++kind) {
        h={9,9,9,9,9,9,9,9,9};opencv_calib3d_test_fail(27,kind);
        check(opencv_calib3d_fundamental_result_matrix(clean.get(),&h)==expected[kind] && zero(),
              "fundamental matrix fault clearing");
        index=99;opencv_calib3d_test_fail(28,kind);
        check(opencv_calib3d_fundamental_result_inlier(clean.get(),0,&index)==expected[kind] && index==0,
              "fundamental inlier fault clearing");
    }
    std::cout << "PASS: fundamental 30 fault scenarios, null output/no partial publication\n";
#endif
    opencv_calib3d_fundamental_result_destroy(nullptr);
    std::cout << "PASS: fundamental real strided Core Regions, 14 rejected/15 true RANSAC, collinear no-model, raw negatives\n";
}

void essential_boundary() {
#if CV_VERSION_MAJOR >= 5
    std::cout << "Essential compiled profile: " << CV_VERSION <<
        " / geometry; required maxIters overload with fixed literal 1000\n";
#else
    std::cout << "Essential compiled profile: " << CV_VERSION <<
        " / calib3d; legacy no-maxIters overload, native ceiling 1000\n";
#endif
    using EResult=std::unique_ptr<opencv_calib3d_essential_result_handle,
        decltype(&opencv_calib3d_essential_result_destroy)>;
    constexpr int n=40;
    auto first_parent=matrix(n,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto second_parent=matrix(n,2,OPENCV_CORE_DEPTH_FLOAT64,2);
    opencv_core_mat_handle *fraw=nullptr,*sraw=nullptr;
    check(opencv_core_mat_region(first_parent.get(),0,0,1,n,&fraw)==0 &&
          opencv_core_mat_region(second_parent.get(),0,0,1,n,&sraw)==0,"Essential Core Regions");
    Mat first(fraw,opencv_core_mat_destroy),second(sraw,opencv_core_mat_destroy);
    check(!output(fraw).isContinuous() && !output(sraw).isContinuous(),"both Essential arrays strided");
    const double c=std::cos(.08),s=std::sin(.08);
    const cv::Matx33d truth(c,0,s,0,1,0,-s,0,c);
    const cv::Vec3d t(-.75,.08,.12),unit=t/cv::norm(t);
    const cv::Matx33d cross(0,-unit[2],unit[1],unit[2],0,-unit[0],-unit[1],unit[0],0);
    const cv::Matx33d known=cross*truth;
    const opencv_calib3d_essential oracle{known(0,0),known(0,1),known(0,2),known(1,0),known(1,1),
        known(1,2),known(2,0),known(2,1),known(2,2)};
    for (int i=0;i<n;++i) {
        const cv::Vec3d x(((i*7)%17-8)*.24,((i*11)%19-9)*.19,4.+((i*13)%23)*.17);
        const cv::Vec3d q=truth*x+t;
        check(x[2]>0 && q[2]>0,"positive fixture depths");
        output(fraw).at<cv::Vec2d>(i,0)={x[0]/x[2],x[1]/x[2]};
        output(sraw).at<cv::Vec2d>(i,0)={q[0]/q[2],q[1]/q[2]};
        double error=99;
        check(opencv_calib3d_detail::normalized_sampson_error(oracle,x[0]/x[2],x[1]/x[2],
            q[0]/q[2],q[1]/q[2],error) && error<1e-15,"independent raw known E oracle");
    }
    const opencv_calib3d_essential_options options{1e-6,.999};
    auto estimate=[&](const opencv_core_mat_handle *a,const opencv_core_mat_handle *b) {
        opencv_calib3d_essential_result_handle *raw=nullptr;
        check(opencv_calib3d_find_essential_ransac(a,b,&options,&raw)==0 && raw,
              "raw Essential estimate");
        return EResult(raw,opencv_calib3d_essential_result_destroy);
    };
    auto validate=[&](const EResult &result) {
        uint8_t found=0,pose_found=0;
        int32_t count=0,pose_count=0,index=0;
        opencv_calib3d_essential e{};
        opencv_calib3d_relative_pose p{};
        check(opencv_calib3d_essential_result_found(result.get(),&found)==0 && found &&
            opencv_calib3d_essential_result_pose_found(result.get(),&pose_found)==0 && pose_found &&
            opencv_calib3d_essential_result_matrix(result.get(),&e)==0 &&
            opencv_calib3d_essential_result_pose(result.get(),&p)==0 &&
            opencv_calib3d_essential_result_inlier_count(result.get(),&count)==0 && count>=5 &&
            opencv_calib3d_essential_result_pose_inlier_count(result.get(),&pose_count)==0 && pose_count>=5,
            "raw Essential/pose support and accessors");
        std::vector<int32_t> ei,pi;
        double maximum=0;
        for (int i=0;i<count;++i) {
            check(opencv_calib3d_essential_result_inlier(result.get(),i,&index)==0 &&
                index>=0 && index<n && (ei.empty() || index>ei.back()),"Essential indices ordered/valid");
            ei.push_back(index);
        }
        for (int i=0;i<pose_count;++i) {
            check(opencv_calib3d_essential_result_pose_inlier(result.get(),i,&index)==0 &&
                std::binary_search(ei.begin(),ei.end(),index) && (pi.empty() || index>pi.back()),"pose subset/order");
            pi.push_back(index);
        }
        for (int i=0;i<n;++i) {
            const auto a=output(fraw).at<cv::Vec2d>(i,0),b=output(sraw).at<cv::Vec2d>(i,0);
            double error=0;
            const bool accepted=opencv_calib3d_detail::normalized_sampson_error(e,a[0],a[1],b[0],b[1],error) &&
                error<=options.normalized_epipolar_threshold;
            check(accepted==std::binary_search(ei.begin(),ei.end(),i),"raw final-E Float64 classification");
            if (accepted) maximum=std::max(maximum,error);
        }
        const cv::Matx33d r(p.r00,p.r01,p.r02,p.r10,p.r11,p.r12,p.r20,p.r21,p.r22);
        const cv::Vec3d direction(p.tx,p.ty,p.tz);
        const cv::Matx33d delta=r*truth.t();
        const double angle=std::acos(std::clamp((cv::trace(delta)-1)/2,-1.,1.));
        check(angle<1e-5 && direction.dot(unit)>1-1e-8 && std::abs(cv::norm(direction)-1)<1e-12 &&
            cv::norm(cv::Mat(r.t()*r-cv::Matx33d::eye()),cv::NORM_INF)<1e-12 &&
            std::abs(cv::determinant(r)-1)<1e-12 && cv::norm(-r.t()*direction+truth.t()*unit)<1e-5,
            "raw rotation/sign/unit/camera-center frame oracle");
        std::cout << "Essential raw support=" << count << " pose support=" << pose_count <<
            " max Sampson=" << maximum << " rotation angle=" << angle << " signed t dot=" << direction.dot(unit) << '\n';
        return ei;
    };
    auto clean=estimate(fraw,sraw); validate(clean);
    auto invalid=[&](const opencv_core_mat_handle *a,const opencv_core_mat_handle *b,
                     const opencv_calib3d_essential_options *o) {
        auto *raw=clean.get(); // Real live result used only as an output sentinel.
        check(opencv_calib3d_find_essential_ransac(a,b,o,&raw)==1 && !raw,"Essential preflight/result clearing");
    };
    invalid(nullptr,sraw,&options); invalid(fraw,nullptr,&options); invalid(fraw,sraw,nullptr);
    check(opencv_calib3d_find_essential_ransac(fraw,sraw,&options,nullptr)==1,"null Essential result output");
    check(opencv_calib3d_validate_essential_options(nullptr)==1,"null Essential options validation");
    for (int count=0;count<=5;++count) {
        auto a=matrix(count,1,OPENCV_CORE_DEPTH_FLOAT64,2),b=matrix(count,1,OPENCV_CORE_DEPTH_FLOAT64,2);
        invalid(a.get(),b.get(),&options);
    }
    auto mismatch=matrix(n-1,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    invalid(fraw,mismatch.get(),&options);
    for (const auto &schema : {cv::Vec3i(n,1,CV_32FC2),cv::Vec3i(n,1,CV_64FC1),
                               cv::Vec3i(n,1,CV_64FC3),cv::Vec3i(1,n,CV_64FC2)}) {
        auto bad=matrix(schema[0],schema[1],CV_MAT_DEPTH(schema[2]),CV_MAT_CN(schema[2]));
        invalid(bad.get(),sraw,&options); invalid(fraw,bad.get(),&options);
    }
    auto nd=matrix(n,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    const int sizes[]={2,3,7}; output(nd.get())=cv::Mat(3,sizes,CV_64FC2,cv::Scalar(0,0));
    invalid(nd.get(),sraw,&options); invalid(fraw,nd.get(),&options);
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    for (double v : {nan,inf,-inf}) {
        for (auto *handle : {fraw,sraw}) for (int axis=0;axis<2;++axis) {
            const auto saved=output(handle).at<cv::Vec2d>(0,0);
            output(handle).at<cv::Vec2d>(0,0)[axis]=v;
            invalid(fraw,sraw,&options); output(handle).at<cv::Vec2d>(0,0)=saved;
        }
    }
    for (double v : {0.,-1.,nan,inf,-inf,1e-30,1e20,1e300}) {
        auto bad=options;bad.normalized_epipolar_threshold=v;invalid(fraw,sraw,&bad);
    }
    for (double v : {0.,-1.,1.,2.,nan,inf,-inf}) {
        auto bad=options;bad.confidence=v;invalid(fraw,sraw,&bad);
    }
    opencv_calib3d_essential e{};
    opencv_calib3d_relative_pose p{};
    auto dirty_e=[&] {e={9,9,9,9,9,9,9,9,9};};
    auto dirty_p=[&] {p={9,9,9,9,9,9,9,9,9,9,9,9};};
    auto zero_e=[&] {return e.e00==0 && e.e01==0 && e.e02==0 && e.e10==0 && e.e11==0 &&
        e.e12==0 && e.e20==0 && e.e21==0 && e.e22==0;};
    auto zero_p=[&] {return p.r00==0 && p.r01==0 && p.r02==0 && p.r10==0 && p.r11==0 &&
        p.r12==0 && p.r20==0 && p.r21==0 && p.r22==0 && p.tx==0 && p.ty==0 && p.tz==0;};
    uint8_t found=9;int32_t count=99,index=99;
    dirty_e(); dirty_p();
    check(opencv_calib3d_essential_result_found(nullptr,&found)==1 && found==0 &&
        opencv_calib3d_essential_result_matrix(nullptr,&e)==1 && zero_e() &&
        opencv_calib3d_essential_result_inlier_count(nullptr,&count)==1 && count==0 &&
        opencv_calib3d_essential_result_inlier(nullptr,0,&index)==1 && index==0,"null E accessors clearing");
    found=9;count=99;index=99;
    check(opencv_calib3d_essential_result_pose_found(nullptr,&found)==1 && found==0 &&
        opencv_calib3d_essential_result_pose(nullptr,&p)==1 && zero_p() &&
        opencv_calib3d_essential_result_pose_inlier_count(nullptr,&count)==1 && count==0 &&
        opencv_calib3d_essential_result_pose_inlier(nullptr,0,&index)==1 && index==0,"null pose accessors clearing");
    check(opencv_calib3d_essential_result_found(clean.get(),nullptr)==1 &&
        opencv_calib3d_essential_result_matrix(clean.get(),nullptr)==1 &&
        opencv_calib3d_essential_result_inlier_count(clean.get(),nullptr)==1 &&
        opencv_calib3d_essential_result_inlier(clean.get(),0,nullptr)==1 &&
        opencv_calib3d_essential_result_pose_found(clean.get(),nullptr)==1 &&
        opencv_calib3d_essential_result_pose(clean.get(),nullptr)==1 &&
        opencv_calib3d_essential_result_pose_inlier_count(clean.get(),nullptr)==1 &&
        opencv_calib3d_essential_result_pose_inlier(clean.get(),0,nullptr)==1,"null Essential/pose outputs");
    opencv_calib3d_essential_result_inlier_count(clean.get(),&count);
    for (int bad : {-1,count}) {
        index=99;check(opencv_calib3d_essential_result_inlier(clean.get(),bad,&index)==1 && index==0,"bad E index clearing");
    }
    opencv_calib3d_essential_result_pose_inlier_count(clean.get(),&count);
    for (int bad : {-1,count}) {
        index=99;check(opencv_calib3d_essential_result_pose_inlier(clean.get(),bad,&index)==1 && index==0,"bad pose index clearing");
    }
    double error=99;found=9;
    check(opencv_calib3d_normalized_sampson_error(nullptr,0,0,0,0,&found,&error)==1 && found==0 && error==0,
        "null Sampson matrix clearing");
    check(opencv_calib3d_normalized_sampson_error(&oracle,0,0,0,0,nullptr,&error)==1 &&
        opencv_calib3d_normalized_sampson_error(&oracle,0,0,0,0,&found,nullptr)==1,"null Sampson outputs");
    error=99;found=9;
    check(opencv_calib3d_normalized_sampson_error(&oracle,nan,0,0,0,&found,&error)==1 && found==0 && error==0,
        "nonfinite raw Sampson clearing");
    for (int i : {1,6,14}) output(sraw).at<cv::Vec2d>(i,0)+=cv::Vec2d(2,-1.5);
    auto robust=estimate(fraw,sraw);const auto ei=validate(robust);
    for (int i : {1,6,14}) check(!std::binary_search(ei.begin(),ei.end(),i),"raw Essential 2/7/15 absent");
    for (int i : {1,6,14}) output(sraw).at<cv::Vec2d>(i,0)-=cv::Vec2d(2,-1.5);
    auto five_a=matrix(5,1,OPENCV_CORE_DEPTH_FLOAT64,2),five_b=matrix(5,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    auto six_a=matrix(6,1,OPENCV_CORE_DEPTH_FLOAT64,2),six_b=matrix(6,1,OPENCV_CORE_DEPTH_FLOAT64,2);
    output(fraw).rowRange(0,6).copyTo(output(six_a.get()));
    output(sraw).rowRange(0,6).copyTo(output(six_b.get()));
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    opencv_calib3d_test_fail(29,3);
    invalid(five_a.get(),five_b.get(),&options);
    auto *raw=clean.get();
    check(opencv_calib3d_find_essential_ransac(six_a.get(),six_b.get(),&options,&raw)==3 && !raw,
        "five preserves armed checkpoint / six consumes pre-native checkpoint");
#endif
    auto six=estimate(six_a.get(),six_b.get());
    check(opencv_calib3d_essential_result_found(six.get(),&found)==0 && found,"six real native call after checkpoint");
#ifdef OPENCV_CALIB3D_TEST_HOOKS
    opencv_calib3d_test_essential_no_model();
    auto absent=estimate(fraw,sraw);dirty_e();dirty_p();found=9;count=99;
    check(opencv_calib3d_essential_result_found(absent.get(),&found)==0 && !found &&
        opencv_calib3d_essential_result_matrix(absent.get(),&e)==1 && zero_e() &&
        opencv_calib3d_essential_result_inlier_count(absent.get(),&count)==0 && count==0 &&
        opencv_calib3d_essential_result_pose_found(absent.get(),&found)==0 && !found &&
        opencv_calib3d_essential_result_pose(absent.get(),&p)==1 && zero_p() &&
        opencv_calib3d_essential_result_pose_inlier_count(absent.get(),&count)==0 && count==0,"forced no-E after real native call");
    index=99;check(opencv_calib3d_essential_result_inlier(absent.get(),0,&index)==1 && index==0,"no-E index clearing");
    opencv_calib3d_test_essential_no_pose();
    auto ambiguous=estimate(fraw,sraw);dirty_p();found=9;count=99;
    check(opencv_calib3d_essential_result_found(ambiguous.get(),&found)==0 && found &&
        opencv_calib3d_essential_result_matrix(ambiguous.get(),&e)==0 &&
        opencv_calib3d_essential_result_inlier_count(ambiguous.get(),&count)==0 && count>=5 &&
        opencv_calib3d_essential_result_pose_found(ambiguous.get(),&found)==0 && !found &&
        opencv_calib3d_essential_result_pose(ambiguous.get(),&p)==1 && zero_p() &&
        opencv_calib3d_essential_result_pose_inlier_count(ambiguous.get(),&count)==0 && count==0,
        "E retained when pose support suppressed after real recoverPose");
    index=99;check(opencv_calib3d_essential_result_pose_inlier(ambiguous.get(),0,&index)==1 && index==0,"no-pose index clearing");
    const int expected[]={0,1,2,3,4,4};
    for (int stage=29;stage<=32;++stage) for (int kind=1;kind<=5;++kind) {
        auto *raw=clean.get();opencv_calib3d_test_fail(stage,kind);
        check(opencv_calib3d_find_essential_ransac(fraw,sraw,&options,&raw)==expected[kind] && !raw,
            "Essential construction exception atomicity");
    }
    for (int kind=1;kind<=5;++kind) {
        dirty_e();opencv_calib3d_test_fail(33,kind);
        check(opencv_calib3d_essential_result_matrix(clean.get(),&e)==expected[kind] && zero_e(),"E fault clearing");
        index=99;opencv_calib3d_test_fail(34,kind);
        check(opencv_calib3d_essential_result_inlier(clean.get(),0,&index)==expected[kind] && index==0,"E index fault clearing");
        dirty_p();opencv_calib3d_test_fail(35,kind);
        check(opencv_calib3d_essential_result_pose(clean.get(),&p)==expected[kind] && zero_p(),"pose fault clearing");
        index=99;opencv_calib3d_test_fail(36,kind);
        check(opencv_calib3d_essential_result_pose_inlier(clean.get(),0,&index)==expected[kind] && index==0,"pose index fault clearing");
    }
    std::cout << "PASS: Essential 40 fault scenarios, armed 5/6, forced no-E / E-found-no-pose after real native calls\n";
#endif
    opencv_calib3d_essential_result_destroy(nullptr);
    std::cout << "PASS: Essential clean/outliers, both strided Core Regions, 0..5 rejection/6 RANSAC, raw schema/options/accessors\n";
}

void run() {
    essential_boundary();
    fundamental_boundary();
    homography_boundary();
    camera_geometry_boundary();
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
