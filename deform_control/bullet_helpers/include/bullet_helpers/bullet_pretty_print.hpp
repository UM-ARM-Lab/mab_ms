#include <string>
#include <btBulletDynamicsCommon.h>

#include <arc_utilities/pretty_print.hpp>

#ifndef BULLET_PRETTY_PRINT_HPP
#define BULLET_PRETTY_PRINT_HPP

namespace PrettyPrint
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<>
    inline std::string PrettyPrint(const btVector3& vector_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(add_delimiters);
        UNUSED(separator);
        return "btVector3: <x: " + std::to_string(vector_to_print.x()) + " y: " + std::to_string(vector_to_print.y()) + " z: " + std::to_string(vector_to_print.z()) + ">";
    }

    template<>
    inline std::string PrettyPrint(const btQuaternion& quaternion_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(add_delimiters);
        UNUSED(separator);
        return "btQuaternion <x: " + std::to_string(quaternion_to_print.x()) + " y: " + std::to_string(quaternion_to_print.y()) + " z: " + std::to_string(quaternion_to_print.z()) + " w: " + std::to_string(quaternion_to_print.w()) + ">";
    }

    template<>
    inline std::string PrettyPrint(const btTransform& transform_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(add_delimiters);
        UNUSED(separator);
        btVector3 vector_to_print = transform_to_print.getOrigin();
        btQuaternion quaternion_to_print(transform_to_print.getRotation());
        return "btTransform <x: " + std::to_string(vector_to_print.x()) + " y: " + std::to_string(vector_to_print.y()) + " z: " + std::to_string(vector_to_print.z()) + ">, <x: " + std::to_string(quaternion_to_print.x()) + " y: " + std::to_string(quaternion_to_print.y()) + " z: " + std::to_string(quaternion_to_print.z()) + " w: " + std::to_string(quaternion_to_print.w()) + ">";
    }

}

#endif // BULLET_PRETTY_PRINT_HPP
