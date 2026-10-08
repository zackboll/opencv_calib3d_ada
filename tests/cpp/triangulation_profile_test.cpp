#include "triangulation_profile.hpp"
#include <cassert>
#include <limits>
int main() {
    using namespace opencv_calib3d_detail;
    opencv_calib3d_relative_pose pose{1,0,0,0,1,0,0,0,1,-1,0,0};
    double r[3][3],t[3];
    assert(triangulation_pose(pose,r,t));
    const double finite[4]={-2,-1,-5,-1};
    auto p=classify_triangulation(finite,r,t,.4,.2,.2,.2);
    assert(p.status==0 && p.x==2 && p.z==5 && p.error_first==0);
    const double infinity[4]={2,1,5,0};
    assert(classify_triangulation(infinity,r,t,0,0,0,0).status==1);
    const double nan[4]={std::numeric_limits<double>::quiet_NaN(),0,1,1};
    assert(classify_triangulation(nan,r,t,0,0,0,0).status==2);
    const double behind[4]={2,1,-5,1};
    assert(classify_triangulation(behind,r,t,0,0,0,0).status==3);
    const double tiny[4]={.4,.2,1,1e-300};
    assert(classify_triangulation(tiny,r,t,.4,.2,.4,.2).status==0);
    const double overflow[4]={1,1,1e-308,1};
    assert(classify_triangulation(overflow,r,t,-1e308,0,0,0).status==4);
    pose.tx=-std::numeric_limits<double>::max();
    assert(triangulation_pose(pose,r,t) && t[0]==-1);
    pose.r00=-1;
    assert(!triangulation_pose(pose,r,t));
}