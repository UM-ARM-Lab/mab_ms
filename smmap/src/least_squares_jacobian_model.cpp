#include "smmap/least_squares_jacobian_model.h"


#include <ros/ros.h>

using namespace smmap;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructor
////////////////////////////////////////////////////////////////////////////////

LeastSquaresJacobianModel::LeastSquaresJacobianModel(
        const Eigen::MatrixXd& initial_jacobian,
        const long extra_samples,
        const bool optimize)
    : JacobianModel(optimize)
    , current_jacobian_(initial_jacobian)
    , next_buffer_ind_(0)
    , buffer_size_(initial_jacobian.cols() + extra_samples)
    , buffer_full_(false)
    , grippers_delta_wide_matrix_(initial_jacobian.cols(), buffer_size_)
    , deformable_delta_wide_matrix_(initial_jacobian.rows(), buffer_size_)
{
    ROS_INFO_NAMED("least_squares_jacobian", "Creating least squares jacobian");
}

////////////////////////////////////////////////////////////////////
// Virtual function overrides
////////////////////////////////////////////////////////////////////

void LeastSquaresJacobianModel::updateModel(const WorldState& previous, const WorldState& next)
{
    const AllGrippersSinglePoseDelta grippers_pose_deltas =
            CalculateGrippersPoseDelta(previous.all_grippers_single_pose_,
                                       next.all_grippers_single_pose_);

    const VectorXd grippers_delta = EigenHelpersConversions::VectorEigenVectorToEigenVectorX(grippers_pose_deltas);

    if (grippers_delta.squaredNorm() < 1e-6)
    {
        ROS_WARN_STREAM_NAMED("least_squares_jacobian", "Grippers did not move much, not updating: squared norm vel: " << grippers_delta.squaredNorm());
    }
    else
    {
        ROS_INFO_NAMED("least_squares_jacobian", "Adding data to buffer");
        grippers_delta_wide_matrix_.col(next_buffer_ind_) = grippers_delta;

        MatrixXd object_delta = next.object_configuration_ - previous.object_configuration_;
        object_delta.resize(current_jacobian_.rows(), 1);
        deformable_delta_wide_matrix_.col(next_buffer_ind_) = object_delta;

        next_buffer_ind_++;
        if (next_buffer_ind_ >= buffer_size_)
        {
            next_buffer_ind_ = 0;
        }

        if (next_buffer_ind_ == 0)
        {
            buffer_full_ = true;
        }
    }

    if (buffer_full_)
    {
        ROS_INFO_NAMED("least_squares_jacobian", "Updating jacobian");
        current_jacobian_ = deformable_delta_wide_matrix_ *
                EigenHelpers::Pinv(grippers_delta_wide_matrix_, EigenHelpers::SuggestedRcond());
    }
}

////////////////////////////////////////////////////////////////////
// Computation helpers
////////////////////////////////////////////////////////////////////

Eigen::MatrixXd LeastSquaresJacobianModel::computeGrippersToObjectJacobian(
        const AllGrippersSinglePose& grippers_pose,
        const ObjectPointSet& current_configuration) const
{
    (void)grippers_pose;
    (void)current_configuration;
    return current_jacobian_;
}
