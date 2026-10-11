#include "pnp_profile.hpp"
#include <cassert>
#include <cstdint>
#include <limits>
int main() {
    using namespace opencv_calib3d_detail;
    assert(intrinsics_fit(800.0, 820.0, 320.0, 240.0));
    assert(!intrinsics_fit(0.0, 820.0, 320.0, 240.0));
    assert(!intrinsics_fit(-1.0, 820.0, 320.0, 240.0));
    assert(distortion5_fit(0.0, 0.0, 0.0, 0.0, 0.0));
    assert(ransac_options_fit(100, 8.0, 0.99));
    assert(!ransac_options_fit(0, 8.0, 0.99));
    assert(!ransac_options_fit(100, 0.0, 0.99));
    assert(!ransac_options_fit(100, 1e-300, 0.99));
    assert(!ransac_options_fit(100, 8.0, 0.0));
    assert(!ransac_options_fit(100, 8.0, 1.0));
    assert(!ransac_options_fit(static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) + 1,
                               8.0, 0.99));
    const double world[3][3] = {{0.3, -0.2, 3.5}, {1.4, 0.1, 4.0}, {-0.5, 1.2, 5.3}};
    const double image[3][2] = {{700.0, 300.0}, {900.0, 400.0}, {500.0, 550.0}};
    const double repeated[3][3] = {{1, 2, 3}, {1, 2, 3}, {0, 1, 4}};
    const double collinear[3][3] = {{0, 0, 1}, {1, 0, 2}, {2, 0, 3}};
    const double same[3][2] = {{400, 300}, {400, 300}, {500, 420}};
    const double coplanar[3][2] = {{640, 360}, {700, 360}, {500, 360}};
    const double huge[3][3] = {{1e200, 0, 1e200}, {0, 1e200, 1e200}, {-1e200, -1e200, 2e200}};
    assert(ap3p_admissible(world, image, 800, 820, 640, 360));
    assert(!ap3p_admissible(repeated, image, 800, 820, 640, 360));
    assert(!ap3p_admissible(collinear, image, 800, 820, 640, 360));
    assert(!ap3p_admissible(world, same, 800, 820, 640, 360));
    assert(!ap3p_admissible(world, coplanar, 800, 820, 640, 360));
    assert(!ap3p_admissible(huge, image, 800, 820, 640, 360));
    return 0;
}
