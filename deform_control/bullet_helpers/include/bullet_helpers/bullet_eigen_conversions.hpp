#ifndef BULLET_EIGEN_CONVERSIONS_HPP
#define BULLET_EIGEN_CONVERSIONS_HPP

#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>

namespace BulletHelpers
{
    inline Eigen::Affine3d toEigenAffine3d(btTransform&& bt)
    {
        const btVector3& bt_origin = bt.getOrigin();
        const btQuaternion& bt_rot = bt.getRotation();
        const Eigen::Translation3d trans(bt_origin.getX(), bt_origin.getY(), bt_origin.getZ());
        const Eigen::Quaterniond rot(bt_rot.getW(), bt_rot.getX(), bt_rot.getY(), bt_rot.getZ());

        return trans*rot;
    }

    inline Eigen::Affine3d toEigenAffine3d(const btTransform& bt)
    {
        const btVector3& bt_origin = bt.getOrigin();
        const btQuaternion& bt_rot = bt.getRotation();
        const Eigen::Translation3d trans(bt_origin.getX(), bt_origin.getY(), bt_origin.getZ());
        const Eigen::Quaterniond rot(bt_rot.getW(), bt_rot.getX(), bt_rot.getY(), bt_rot.getZ());

        return trans*rot;
    }
}

#endif // BULLET_EIGEN_CONVERSIONS_HPP
