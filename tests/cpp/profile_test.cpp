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
    return 0;
}
