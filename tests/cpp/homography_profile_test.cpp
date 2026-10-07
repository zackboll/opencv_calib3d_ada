#include "homography_profile.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    auto check = [](bool condition) {
        if (!condition) throw std::runtime_error("homography classifier oracle");
    };
    using namespace opencv_calib3d_detail;
    const opencv_calib3d_homography h{2,.5,10,0,3,-5,.01,.02,1};
    double x = 0, y = 0;
    check(map_homography(h,10,20,x,y));
    check(std::abs(x-40.0/1.5)<1e-12 && std::abs(y-55.0/1.5)<1e-12);
    check(final_homography_inlier(h,0,0,13,-1,5)); // exact 3-4-5 boundary
    check(!final_homography_inlier(h,0,0,13,-1,4.999));
    check(!map_homography(h,-100,0,x,y));
    const opencv_calib3d_homography tiny{1,0,0,0,1,0,0,0,1e-200};
    check(map_homography(tiny,1e-200,2e-200,x,y) && x==1 && y==2);
    check(!map_homography(tiny,1e200,1e200,x,y));
    const opencv_calib3d_homography identity{1,0,0,0,1,0,0,0,1};
    check(final_homography_inlier(identity,0,0,3e200,4e200,5.1e200));
    check(!final_homography_inlier(identity,0,0,3e200,4e200,4.9e200));
    check(!final_homography_inlier(identity,1e308,0,-1e308,0,1e308));
    const opencv_calib3d_homography overflow{2,0,0,0,1,0,0,0,1};
    check(!map_homography(overflow,std::numeric_limits<double>::max(),0,x,y));
    auto scaled=h;
    scaled={-4,-1,-20,0,-6,10,-.02,-.04,-2};
    check(map_homography(scaled,10,20,x,y) && std::abs(x-40.0/1.5)<1e-12);
    std::cout << "PASS: independent homography classifier boundary/hypot/infinity/scale/range oracles\n";
}