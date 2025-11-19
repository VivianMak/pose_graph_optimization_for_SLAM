#include <Eigen/Dense>
#include <cmath>

#include "utils.hpp"
#include "gn_optimizer.hpp"

namespace gn_helper {

utils::Pose transforms_to_dvector(const GN::Vec3 xi,
                                    const GN::Vec3 xj,
                                    GN::Mat33& T1) 
{
    /*
    * Turn two homogeneous transforms into delta vector
    * @params T1 - from ICP
    * @params T2 - from odom
    */
    
    utils::Pose p1;
    utils::Pose del_p2 = xi - xj;

    // or p1.x?
    p1->x = T1(0, 2);
    p1->y = T1(1, 2);
    p1->theta = std::atan2(T1(1, 0), T1(0, 0));

    // p2->x = T2(0, 2);
    // p2->y = T2(1, 2);
    // p2->theta = std::atan2(T2(1, 0), T2(0, 0));

    utils::Pose delta_p = p1 - del_p2;

    return delta_p;
}

}
