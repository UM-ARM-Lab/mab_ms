#include "utils/util.h"

#include <haptics/thread_socket_interface.h>
#include <boost/foreach.hpp>

using namespace std;

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace util
{
    osg::ref_ptr<osg::Vec3Array> toVec3Array(const vector<btVector3>& in)
    {
        osg::ref_ptr<osg::Vec3Array> out = new osg::Vec3Array();
        out->reserve(in.size());
        BOOST_FOREACH(const btVector3& pt, in) out->push_back(osg::Vec3(pt.x(),pt.y(),pt.z()));
        return out;
    }

    osg::ref_ptr<osg::Vec4Array> toVec4Array(const vector<btVector4>& in)
    {
        osg::ref_ptr<osg::Vec4Array> out = new osg::Vec4Array();
        out->reserve(in.size());
        BOOST_FOREACH(const btVector3& pt, in) out->push_back(osg::Vec4(pt.x(),pt.y(),pt.z(),pt.w()));
        return out;
    }

    static const btMatrix3x3 HAPTIC_ROTATION(btQuaternion(-M_PI/2., 0., 0.));

    static inline btMatrix3x3 toHapticBtMatrix(const Matrix3d &m)
    {
        // note: the rows are permuted
        return btMatrix3x3(m(2, 0), m(2, 1), m(2, 2),
                           m(0, 0), m(0, 1), m(0, 2),
                           m(1, 0), m(1, 1), m(1, 2));
    }

    static inline btVector3 toHapticBtVector(const Vector3d &v)
    {
        // note: the components are permuted
        return btVector3(v.z(), v.x(), v.y());
    }

    bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2])
    {
        Vector3d start_proxy_pos, end_proxy_pos;
        Matrix3d start_proxy_rot, end_proxy_rot;
        if (!getDeviceState(start_proxy_pos, start_proxy_rot, buttons0,
                            end_proxy_pos, end_proxy_rot, buttons1))
            return false;
        trans0 = btTransform(toHapticBtMatrix(start_proxy_rot) * HAPTIC_ROTATION,
                             toHapticBtVector(start_proxy_pos));
        trans1 = btTransform(toHapticBtMatrix(end_proxy_rot) * HAPTIC_ROTATION,
                             toHapticBtVector(end_proxy_pos));


        return true;
    }

    void sendRobotState(btVector3 pos0, btVector3 pos1)
    {
        double out0[3] = {pos0[1], pos0[2], pos0[0]};
        double out1[3] = {pos1[1], pos1[2], pos1[0]};
        sendDeviceState(out0, true, out1, true);
        cout << "send pos: " << out0[0] << " " << out0[1] << " " << out0[2] << ", " <<
          out1[0] << " " << out1[1] << " " << out1[2] << endl;
      }
}
