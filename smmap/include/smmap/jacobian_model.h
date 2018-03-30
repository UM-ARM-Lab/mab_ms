#ifndef JACOBIAN_MODEL_H
#define JACOBIAN_MODEL_H

#include "smmap/deformable_model.h"

namespace smmap
{
    class JacobianModel : public DeformableModel
    {
        public:
            ////////////////////////////////////////////////////////////////////
            // Constructors and Destructor
            ////////////////////////////////////////////////////////////////////

            JacobianModel(bool optimize);

            ////////////////////////////////////////////////////////////////////
            // Virtual function overrides
            ////////////////////////////////////////////////////////////////////

            virtual ObjectPointSet getObjectDelta(
                    const WorldState& world_initial_state,
                    const AllGrippersSinglePoseDelta& gripper_pose_delta,
                    const double dt) const override final;

            virtual std::pair<AllGrippersSinglePoseDelta, ObjectPointSet> getSuggestedGrippersCommand(
                    TaskDesiredObjectDeltaFunctionType task_desired_object_delta_fn,
                    const WorldState& world_initial_state,
                    const double dt,
                    const double max_gripper_velocity,
                    const double obstacle_avoidance_scale) const override final;

        protected:

            ////////////////////////////////////////////////////////////////////
            // Static helpers
            ////////////////////////////////////////////////////////////////////

            static void ComputeObjectNodeDistanceMatrix();

            ////////////////////////////////////////////////////////////////////
            // Computation helpers
            ////////////////////////////////////////////////////////////////////

            ObjectPointSet getObjectDelta(
                    const ObjectPointSet& object_initial_configuration,
                    const AllGrippersSinglePose& grippers_pose,
                    const AllGrippersSinglePoseDelta& grippers_pose_delta) const;

            virtual Eigen::MatrixXd computeGrippersToObjectJacobian(
                    const AllGrippersSinglePose& grippers_pose,
                    const ObjectPointSet& current_configuration) const = 0;

            ////////////////////////////////////////////////////////////////////
            // Static members
            ////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////
            // Private members
            ////////////////////////////////////////////////////////////////////

            // Controls if we perform an optimization instead of a pseudo inverse + max velocity clipping
            bool optimize_;
    };
}

#endif // DIMINISHING_RIGIDITY_MODEL_H
