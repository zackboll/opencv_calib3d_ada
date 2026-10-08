#ifndef OPENCV_CALIB3D_TRIANGULATION_PROFILE_HPP
#define OPENCV_CALIB3D_TRIANGULATION_PROFILE_HPP
#include "opencv_calib3d_shim.h"
#include <algorithm>
#include <cmath>

namespace opencv_calib3d_detail {
// Conservative absolute Float64 SO(3) tolerance; never repair rotations.
constexpr double triangulation_rotation_tolerance = 1e-9;
inline bool triangulation_pose(const opencv_calib3d_relative_pose &p,
                               double (&r)[3][3], double (&t)[3]) noexcept {
    const double values[3][3] = {{p.r00,p.r01,p.r02},
                                {p.r10,p.r11,p.r12}, {p.r20,p.r21,p.r22}};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        if (!std::isfinite(values[i][j]) || std::abs(values[i][j]) > 1.000000001) return false;
        r[i][j]=values[i][j];
    }
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        double dot=0;
        for (int k=0;k<3;++k) dot+=r[k][i]*r[k][j];
        if (std::abs(dot-(i==j ? 1.0 : 0.0)) > triangulation_rotation_tolerance) return false;
    }
    const double det=r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
                    -r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
                    +r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
    if (std::abs(det-1) > triangulation_rotation_tolerance) return false;
    if (!std::isfinite(p.tx) || !std::isfinite(p.ty) || !std::isfinite(p.tz)) return false;
    const double scale=std::max({std::abs(p.tx),std::abs(p.ty),std::abs(p.tz)});
    if (scale==0) return false;
    t[0]=p.tx/scale; t[1]=p.ty/scale; t[2]=p.tz/scale;
    const double norm=std::hypot(t[0],t[1],t[2]);
    for (double &v:t) v/=norm;
    return true;
}
inline opencv_calib3d_triangulated_point classify_triangulation(
    const double (&h)[4], const double (&r)[3][3], const double (&t)[3],
    double x1, double y1, double x2, double y2) noexcept {
    opencv_calib3d_triangulated_point out{};
    out.status=OPENCV_CALIB3D_TRI_UNREPRESENTABLE;
    for (double v:h) if (!std::isfinite(v)) return out;
    if (h[3]==0) { out.status=OPENCV_CALIB3D_TRI_AT_INFINITY; return out; }
    const double a[3]={h[0]/h[3],h[1]/h[3],h[2]/h[3]};
    double b[3];
    for (double v:a) if (!std::isfinite(v)) return out;
    for (int i=0;i<3;++i) {
        b[i]=t[i];
        for (int j=0;j<3;++j) {
            const double term=r[i][j]*a[j];
            if (!std::isfinite(term)) return out;
            b[i]+=term;
            if (!std::isfinite(b[i])) return out;
        }
    }
    if (a[2]<=0 || b[2]<=0) { out.status=OPENCV_CALIB3D_TRI_NON_POSITIVE_DEPTH; return out; }
    out.status=OPENCV_CALIB3D_TRI_UNDEFINED_REPROJECTION;
    const double predicted[4]={a[0]/a[2],a[1]/a[2],b[0]/b[2],b[1]/b[2]};
    for (double v:predicted) if (!std::isfinite(v)) return out;
    const double residual[4]={predicted[0]-x1,predicted[1]-y1,
                              predicted[2]-x2,predicted[3]-y2};
    for (double v:residual) if (!std::isfinite(v)) return out;
    const double e1=std::hypot(residual[0],residual[1]);
    const double e2=std::hypot(residual[2],residual[3]);
    if (!std::isfinite(e1) || !std::isfinite(e2)) return out;
    return {OPENCV_CALIB3D_TRI_USABLE,a[0],a[1],a[2],a[2],b[2],e1,e2};
}
}
#endif