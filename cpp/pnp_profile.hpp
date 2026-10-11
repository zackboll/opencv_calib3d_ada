#ifndef OPENCV_CALIB3D_PNP_PROFILE_HPP
#define OPENCV_CALIB3D_PNP_PROFILE_HPP

#include <cmath>
#include <cstdint>
#include <limits>

namespace opencv_calib3d_detail {
inline bool finite(double value) noexcept { return std::isfinite(value); }
inline double larger(double left, double right) noexcept { return left < right ? right : left; }
inline bool intrinsics_fit(double fx, double fy, double cx, double cy) noexcept {
    return finite(fx) && finite(fy) && finite(cx) && finite(cy) && fx > 0.0 && fy > 0.0;
}
inline bool distortion5_fit(double k1, double k2, double p1, double p2, double k3) noexcept {
    return finite(k1) && finite(k2) && finite(p1) && finite(p2) && finite(k3);
}
/* Exact zeros only. Positive separation is admissible even when tiny. */
inline bool ap3p_admissible(const double world[3][3], const double image[3][2],
                            double fx, double fy, double cx, double cy) noexcept {
    auto scaled_diff = [](double left, double right, double &delta) noexcept {
        const double scale = larger(std::abs(left), std::abs(right));
        if (scale > 1.0 && std::abs(left) > std::numeric_limits<double>::max() - std::abs(right))
            return false;
        delta = left - right;
        return finite(delta);
    };
    auto cross_norm = [&](const double a[3], const double b[3], double &value) noexcept {
        double terms[3];
        for (int i = 0; i < 3; ++i) {
            const int j = (i + 1) % 3, k = (i + 2) % 3;
            const double scale = larger(larger(std::abs(a[j]), std::abs(b[k])),
                                          larger(std::abs(a[k]), std::abs(b[j])));
            if (scale > 1.0 && std::abs(a[j]) > std::numeric_limits<double>::max() / scale)
                return false;
            if (scale > 1.0 && std::abs(a[k]) > std::numeric_limits<double>::max() / scale)
                return false;
            const double left = a[j] * b[k], right = a[k] * b[j];
            if (!finite(left) || !finite(right)) return false;
            if (!scaled_diff(left, right, terms[i])) return false;
        }
        const double squares = terms[0] * terms[0] + terms[1] * terms[1] + terms[2] * terms[2];
        if (!finite(squares)) return false;
        value = std::sqrt(squares);
        return finite(value);
    };
    double baseline[3], span[3], bearings[3][3];
    for (int axis = 0; axis < 3; ++axis) {
        if (!scaled_diff(world[0][axis], world[1][axis], baseline[axis])) return false;
        if (!scaled_diff(world[0][axis], world[2][axis], span[axis])) return false;
    }
    double baseline_norm = 0, area = 0;
    const double baseline_square = baseline[0] * baseline[0] + baseline[1] * baseline[1] +
                                   baseline[2] * baseline[2];
    if (!finite(baseline_square) || baseline_square == 0.0) return false;
    baseline_norm = std::sqrt(baseline_square);
    if (!finite(baseline_norm) || baseline_norm == 0.0) return false;
    if (!cross_norm(span, baseline, area) || area == 0.0) return false;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(image[i][0]) > std::numeric_limits<double>::max() / 2.0 ||
            std::abs(cx) > std::numeric_limits<double>::max() / 2.0 ||
            std::abs(image[i][1]) > std::numeric_limits<double>::max() / 2.0 ||
            std::abs(cy) > std::numeric_limits<double>::max() / 2.0)
            return false;
        const double x = (image[i][0] - cx) / fx;
        const double y = (image[i][1] - cy) / fy;
        if (!finite(x) || !finite(y) || (image[i][0] != cx && x == 0.0) ||
            (image[i][1] != cy && y == 0.0))
            return false;
        const double square = x * x + y * y + 1.0;
        if (!finite(square) || square == 0.0) return false;
        const double norm = std::sqrt(square);
        bearings[i][0] = x / norm;
        bearings[i][1] = y / norm;
        bearings[i][2] = 1.0 / norm;
        for (double component : bearings[i]) if (!finite(component)) return false;
    }
    double bearing_cross = 0, coplanar = 0;
    if (!cross_norm(bearings[0], bearings[1], bearing_cross) || bearing_cross == 0.0)
        return false;
    const double k3[3] = {
        (bearings[0][1] * bearings[1][2] - bearings[0][2] * bearings[1][1]) / bearing_cross,
        (bearings[0][2] * bearings[1][0] - bearings[0][0] * bearings[1][2]) / bearing_cross,
        (bearings[0][0] * bearings[1][1] - bearings[0][1] * bearings[1][0]) / bearing_cross};
    coplanar = k3[0] * bearings[2][0] + k3[1] * bearings[2][1] + k3[2] * bearings[2][2];
    return finite(coplanar) && coplanar != 0.0;
}
inline bool ransac_options_fit(std::int64_t iterations, double reprojection_error,
                               double confidence) noexcept {
    return iterations > 0 && iterations <= std::numeric_limits<std::int32_t>::max() &&
           finite(reprojection_error) && reprojection_error > 0.0 &&
           reprojection_error <= std::numeric_limits<float>::max() &&
            static_cast<float>(reprojection_error) > 0.0F &&
           finite(confidence) && confidence > 0.0 && confidence < 1.0;
}
} // namespace opencv_calib3d_detail

#endif
