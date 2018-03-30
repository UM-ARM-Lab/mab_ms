#ifndef BULLET_ROS_CONVERSIONS_HPP
#define BULLET_ROS_CONVERSIONS_HPP

#include <btBulletDynamicsCommon.h>
#include <osg/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>

namespace BulletHelpers
{
    inline btQuaternion toBulletQuaternion(const geometry_msgs::Quaternion& quat)
    {
        return btQuaternion(quat.x, quat.y, quat.z, quat.w);
    }

    inline btVector3 toBulletVector3(const geometry_msgs::Point& pos, const float bt_scale)
    {
        return btVector3(pos.x, pos.y, pos.z)*bt_scale;
    }


    inline geometry_msgs::Vector3 toRosVector3(const btVector3& bt, const float bt_scale)
    {
        geometry_msgs::Vector3 ros;
        ros.x = bt.x() * bt_scale;
        ros.y = bt.y() * bt_scale;
        ros.z = bt.z() * bt_scale;
        return ros;
    }

    inline btTransform toBulletTransform(const geometry_msgs::Pose& pose, const float bt_scale)
    {
        btQuaternion rot = toBulletQuaternion(pose.orientation);
        btVector3 trans = toBulletVector3(pose.position, bt_scale);
        btTransform tf(rot, trans);
        return tf;
    }


    inline geometry_msgs::Quaternion toRosQuaternion(const btQuaternion& quat)
    {
        geometry_msgs::Quaternion ros;
        ros.x = quat.x();
        ros.y = quat.y();
        ros.z = quat.z();
        ros.w = quat.w();
        return ros;
    }

    inline geometry_msgs::Point toRosPoint(const btVector3& vec, const float bt_scale)
    {
        geometry_msgs::Point point;
        assert(-100.0f < vec.x() && vec.x() < 100.0f);
        assert(-100.0f < vec.y() && vec.y() < 100.0f);
        assert(-100.0f < vec.z() && vec.z() < 100.0f);
        point.x = vec.x()/bt_scale;
        point.y = vec.y()/bt_scale;
        point.z = vec.z()/bt_scale;
        return point;
    }

    inline geometry_msgs::Pose toRosPose(const btTransform& tf, const float bt_scale)
    {
        geometry_msgs::Pose pose;
        pose.position = toRosPoint(tf.getOrigin(), bt_scale);
        pose.orientation = toRosQuaternion(tf.getRotation());
        return pose;
    }

    inline std::vector<geometry_msgs::Point> toRosPointVector(const std::vector< btVector3>& bt, const float bt_scale)
    {
        std::vector<geometry_msgs::Point> ros(bt.size());
        for (size_t i = 0; i < bt.size() ; i++)
        {
            ros[i] = toRosPoint(bt[i], bt_scale);
        }
        return ros;
    }

    // We have to essentially double the entries for every internal point
    // of marker.points and marker.colors
    inline void convertLineStripToLineList(visualization_msgs::Marker& marker)
    {
        const size_t num_lines = marker.points.size();

        auto points = marker.points;
        auto colors = marker.colors;

        assert(points.size() >= 2);
        assert(points.size() == colors.size());

        marker.points.resize(num_lines * 2 - 2);
        marker.colors.resize(num_lines * 2 - 2);
        for (size_t ind = 1; ind < num_lines*2 - 2 ; ind++)
        {
            marker.points[ind] = points[(ind+1)/2];
            marker.colors[ind] = colors[(ind+1)/2];
        }
    }

    inline std::vector<btVector3> toBulletPointVector(const std::vector< geometry_msgs::Point>& ros, const float bt_scale)
    {
        std::vector<btVector3> bt(ros.size());
        for (size_t i = 0; i < ros.size() ; i++)
        {
            bt[i] = toBulletVector3(ros[i], bt_scale);
        }
        return bt;
    }

    inline btVector4 toBulletColor(const std_msgs::ColorRGBA& ros)
    {
        return btVector4(ros.r, ros.g, ros.b, ros.a);
    }

    inline std::vector<btVector4> toBulletColorArray(const std::vector< std_msgs::ColorRGBA>& ros)
    {
        std::vector<btVector4> bt(ros.size());
        for (size_t i = 0; i < ros.size() ; i++)
        {
            bt[i] = toBulletColor(ros[i]);
        }
        return bt;
    }

    inline std::vector<btVector4> toBulletColorArray(const std_msgs::ColorRGBA& ros, size_t num_copies)
    {
        return std::vector<btVector4>(num_copies, toBulletColor(ros));
    }


    inline osg::ref_ptr<osg::Vec3Array> toOsgRefVec3Array(const std::vector<geometry_msgs::Point>& ros, const float bt_scale)
    {
        osg::ref_ptr<osg::Vec3Array> out = new osg::Vec3Array();
        out->reserve(ros.size());
        for(auto point: ros)
        {
            out->push_back(osg::Vec3(point.x, point.y, point.z) * bt_scale);
        }
        return out;
    }

    inline osg::ref_ptr<osg::Vec4Array> toOsgRefVec4Array(const std::vector<std_msgs::ColorRGBA>& ros)
    {
        osg::ref_ptr<osg::Vec4Array> out = new osg::Vec4Array();
        out->reserve(ros.size());
        for(auto color: ros)
        {
            out->push_back(osg::Vec4(color.r, color.g, color.b, color.a));
        }
        return out;
    }
}

#endif // BULLET_ROS_CONVERSIONS_HPP
