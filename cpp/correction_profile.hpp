#ifndef OPENCV_CALIB3D_CORRECTION_PROFILE_HPP
#define OPENCV_CALIB3D_CORRECTION_PROFILE_HPP

// Guarded native correspondence-correction arithmetic profile. Pure Float64 checks only: this
// header never corrects correspondences, solves polynomials or chooses a policy.
// Every threshold comes from the caller's opencv_calib3d_correction_options.

#include "opencv_calib3d_shim.h"
#include "fundamental_profile.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace opencv_calib3d_detail {

inline bool correction_options_fit(const opencv_calib3d_correction_options &o) noexcept {
    for (double v : {o.minimum_second_singular_ratio, o.maximum_third_singular_ratio,
                     o.minimum_epipolar_line_ratio, o.maximum_corrected_error_pixels,
                     o.absolute_displacement_excess_pixels, o.relative_displacement_excess})
        if (!std::isfinite(v)) return false;
    return o.minimum_second_singular_ratio > 0 && o.minimum_second_singular_ratio <= 1 &&
           o.maximum_third_singular_ratio >= 0 && o.maximum_third_singular_ratio < 1 &&
           o.maximum_third_singular_ratio < o.minimum_second_singular_ratio &&
           o.minimum_epipolar_line_ratio >= 0 && o.minimum_epipolar_line_ratio <= 1 &&
           o.maximum_corrected_error_pixels >= 0 &&
           o.absolute_displacement_excess_pixels >= 0 &&
           o.relative_displacement_excess >= 0 && o.relative_displacement_excess <= 1;
}

enum class fundamental_normalization { ok, nonfinite, zero, unrepresentable };

// Divide by the maximum absolute coefficient. Homogeneous geometry, sign and
// zero pattern are preserved. A nonzero coefficient whose quotient would be zero
// or subnormal is rejected: the normalized matrix could not carry it at full
// binary64 precision, so it would silently become a different matrix.
inline fundamental_normalization normalize_fundamental(
    const opencv_calib3d_fundamental &f, double (&n)[9]) noexcept {
    const double in[9] = {f.f00,f.f01,f.f02,f.f10,f.f11,f.f12,f.f20,f.f21,f.f22};
    double scale = 0;
    for (double v : in) {
        if (!std::isfinite(v)) return fundamental_normalization::nonfinite;
        scale = std::max(scale, std::abs(v));
    }
    if (scale == 0) return fundamental_normalization::zero;
    for (int i = 0; i < 9; ++i) {
        n[i] = in[i] / scale;
        if (!std::isfinite(n[i]) || (in[i] != 0 && std::abs(n[i]) < DBL_MIN))
            return fundamental_normalization::unrepresentable;
    }
    return fundamental_normalization::ok;
}

enum class epipolar_geometry { ok, unrepresentable, undefined_line };

// l2 = F*p1, l1 = F^T*p2 (row-major f). d1 = dist(p1,l1), d2 = dist(p2,l2).
// Same summation order as Ada Maximum_Epipolar_Error; no epsilon, no clamping.
inline epipolar_geometry epipolar_distances(const double (&f)[9],
                                            double x1, double y1, double x2, double y2,
                                            double &n1, double &n2,
                                            double &d1, double &d2) noexcept {
    n1 = n2 = d1 = d2 = 0;
    for (double v : {x1, y1, x2, y2}) if (!std::isfinite(v)) return epipolar_geometry::unrepresentable;
    double l2[3], l1[3], r1, r2;
    for (int i = 0; i < 3; ++i)
        if (!finite_sum3(f[3*i], x1, f[3*i+1], y1, f[3*i+2], l2[i]) ||
            !finite_sum3(f[i], x2, f[3+i], y2, f[6+i], l1[i]))
            return epipolar_geometry::unrepresentable;
    n1 = std::hypot(l1[0], l1[1]);
    n2 = std::hypot(l2[0], l2[1]);
    if (!std::isfinite(n1) || !std::isfinite(n2)) return epipolar_geometry::unrepresentable;
    if (n1 == 0 || n2 == 0) return epipolar_geometry::undefined_line;
    if (!finite_sum3(l1[0], x1, l1[1], y1, l1[2], r1) ||
        !finite_sum3(l2[0], x2, l2[1], y2, l2[2], r2))
        return epipolar_geometry::unrepresentable;
    d1 = std::abs(r1) / n1;
    d2 = std::abs(r2) / n2;
    if (!std::isfinite(d1) || !std::isfinite(d2)) return epipolar_geometry::unrepresentable;
    return epipolar_geometry::ok;
}

// Dimensionless, invariant to nonzero common scaling of F and of each homogeneous
// point: min(|l2.xy|/(||F||_F ||p1||), |l1.xy|/(||F||_F ||p2||)), within [0,1].
// A numerical quality measure, not a geometric correctness proof.
inline bool epipolar_line_quality(const double (&f)[9], double x1, double y1,
                                  double x2, double y2, double n1, double n2,
                                  double &quality) noexcept {
    quality = 0;
    double frobenius = 0;
    for (double v : f) frobenius = std::hypot(frobenius, v);
    const double p1 = std::hypot(x1, y1, 1.0), p2 = std::hypot(x2, y2, 1.0);
    if (!std::isfinite(frobenius) || frobenius == 0 || !std::isfinite(p1) || !std::isfinite(p2))
        return false;
    const double q2 = (n2 / frobenius) / p1, q1 = (n1 / frobenius) / p2;
    if (!std::isfinite(q1) || !std::isfinite(q2)) return false;
    quality = std::min(q1, q2);
    return true;
}

// Scaled four-dimensional Euclidean norm; never squares an unscaled residual.
inline bool displacement_norm4(const double (&d)[4], double &value) noexcept {
    value = 0;
    double scale = 0;
    for (double v : d) {
        if (!std::isfinite(v)) return false;
        scale = std::max(scale, std::abs(v));
    }
    if (scale == 0) return true;
    double sum = 0;
    for (double v : d) { const double r = v / scale; sum += r * r; }
    value = scale * std::sqrt(sum);
    return std::isfinite(value);
}

// Necessary (NOT sufficient) optimality condition with caller tolerances:
// D <= Bound*(1+relative) + absolute. A nonrepresentable allowance is an error,
// never a silent pass.
inline bool displacement_allowance(double bound, double relative, double absolute,
                                   double &allowed) noexcept {
    const double scaled = bound * (1.0 + relative);
    allowed = scaled + absolute;
    return std::isfinite(scaled) && std::isfinite(allowed);
}
} // namespace opencv_calib3d_detail
#endif
