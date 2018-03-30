#include "smmap/deformable_model.h"

#include <arc_utilities/arc_exceptions.hpp>

using namespace smmap;

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////

DeformableModel::DeformableModel()
{
    if (!grippers_data_initialized_.load())
    {
        throw_arc_exception(std::runtime_error, "You must call SetGrippersData before constructing a DeformableObjectModel");
    }

    if (!function_pointers_initialized_.load())
    {
        throw_arc_exception(std::runtime_error, "You must call SetCallbackFunctions before constructing a DeformableObjectModel");
    }
}

////////////////////////////////////////////////////////////////////////////////
// Static member initialization
////////////////////////////////////////////////////////////////////////////////

std::atomic_bool DeformableModel::grippers_data_initialized_(false);
std::vector<GripperData> DeformableModel::grippers_data_;

void DeformableModel::SetGrippersData(
        const std::vector<GripperData>& grippers_data)
{
    grippers_data_ = grippers_data;
    grippers_data_initialized_.store(true);
}



std::atomic_bool DeformableModel::function_pointers_initialized_(false);
GripperCollisionCheckFunctionType DeformableModel::gripper_collision_check_fn_;

void DeformableModel::SetCallbackFunctions(
        const GripperCollisionCheckFunctionType& gripper_collision_check_fn)
{
    gripper_collision_check_fn_ = gripper_collision_check_fn;
    function_pointers_initialized_.store(true);
}
