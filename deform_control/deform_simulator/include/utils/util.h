#ifndef __UTIL_H__
#define __UTIL_H__

#include <LinearMath/btTransform.h>
#include <osg/Vec3d>
#include <osg/Geometry>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "utils/my_assert.h"

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

namespace util {

    // reads input from haptic devices (using getDeviceState),
    // and then transforms the rotations/coordinates to our coordinate system
    bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]);

    ///////////////// CONVERSIONS ////////////////////////////

    inline osg::Vec3d toOSGVector(const btVector3 &v) { return osg::Vec3d(v.x(), v.y(), v.z()); }
    inline btVector3 toBtVector(const osg::Vec3d &v) { return btVector3((btScalar)v.x(), (btScalar)v.y(), (btScalar)v.z()); }

    osg::ref_ptr<osg::Vec3Array> toVec3Array(const std::vector<btVector3>&);
    osg::ref_ptr<osg::Vec4Array> toVec4Array(const std::vector<btVector4>&);

    ///////////////// FILE IO ////////////////////////////

    template <class T>
    void read_2d_array(std::vector<std::vector<T>>& arr, std::string fname) {

        std::ifstream infile(fname.c_str());
        std::string line;
        arr.clear();
        while (getline(infile,line)) {
            std::stringstream ss (std::stringstream::in | std::stringstream::out);
            ss << line;
            std::vector<T> v;
            v.clear();
            while (ss) {
	T f;
	ss >> f;
	v.push_back(f);
            }
            arr.push_back(v);
        }
    }

    template <class T>
    void read_1d_array(std::vector<T>& arr, std::string fname) {
        std::ifstream infile(fname.c_str());
        T i;
        arr.clear();
        while (infile) {
            infile >>i;
            arr.push_back(i);
        }
    }

}

#endif // __UTIL_H__
