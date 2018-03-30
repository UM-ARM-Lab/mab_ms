#ifndef DEFORMABLE_MODEL_H
#define DEFORMABLE_MODEL_H

#include <atomic>

#include "smmap/task_function_pointer_types.h"
#include "smmap/grippers.hpp"

namespace smmap
{
    class DeformableModel
    {
        public:
            typedef std::shared_ptr<DeformableModel> Ptr;

            DeformableModel();

            ////////////////////////////////////////////////////////////////////
            // Virtual functions that define the interface
            ////////////////////////////////////////////////////////////////////

            virtual void updateModel(const WorldState& previous, const WorldState& next) = 0;

            virtual ObjectPointSet getObjectDelta(
                    const WorldState& world_initial_state,
                    const AllGrippersSinglePoseDelta& grippers_pose_delta,
                    const double dt) const = 0;

            virtual std::pair<AllGrippersSinglePoseDelta, ObjectPointSet> getSuggestedGrippersCommand(
                    TaskDesiredObjectDeltaFunctionType task_desired_object_delta_fn,
                    const WorldState& world_initial_state,
                    const double dt,
                    const double max_gripper_velocity,
                    const double obstacle_avoidance_scale) const = 0;

            ////////////////////////////////////////////////////////////////////
            // Update/Set function for static members
            ////////////////////////////////////////////////////////////////////

            static void SetGrippersData(
                    const std::vector<GripperData>& grippers_data);

            static void SetCallbackFunctions(
                    const GripperCollisionCheckFunctionType& gripper_collision_check_fn);

        protected:

            ////////////////////////////////////////////////////////////////////
            // Destructor that prevents "delete pointer to base object"
            ////////////////////////////////////////////////////////////////////

            ~DeformableModel() {}

            ////////////////////////////////////////////////////////////////////
            // Static data
            ////////////////////////////////////////////////////////////////////

            static std::atomic_bool grippers_data_initialized_;
            static std::vector<GripperData> grippers_data_;

            static std::atomic_bool function_pointers_initialized_;
            static GripperCollisionCheckFunctionType gripper_collision_check_fn_;
    };
}

#endif // DEFORMABLE_MODEL_H
