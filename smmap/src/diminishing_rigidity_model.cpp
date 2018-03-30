#include "smmap/diminishing_rigidity_model.h"

#include <cmath>
#include <arc_utilities/arc_exceptions.hpp>

using namespace smmap;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Static member initialization
////////////////////////////////////////////////////////////////////////////////

std::atomic_bool DiminishingRigidityModel::static_data_initialized_(false);
Eigen::MatrixXd DiminishingRigidityModel::object_initial_node_distance_;
long DiminishingRigidityModel::num_nodes_;

////////////////////////////////////////////////////////////////////////////////
// Static helpers
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief DiminishingRigidityModel::SetInitialObjectConfiguration This function
 *          is not thread safe.
 * @param object_initial_configuration
 */
void DiminishingRigidityModel::SetInitialObjectConfiguration(
        const ObjectPointSet& object_initial_configuration)
{
    num_nodes_ = object_initial_configuration.cols();
    object_initial_node_distance_ = CalculateDistanceMatrix(object_initial_configuration);
    static_data_initialized_.store(true);
}

////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructor
////////////////////////////////////////////////////////////////////////////////

DiminishingRigidityModel::DiminishingRigidityModel(
        const double deformability,
        const bool optimize)
    : DiminishingRigidityModel(deformability, deformability, optimize)
{}

DiminishingRigidityModel::DiminishingRigidityModel(
        const double translation_deformability,
        const double rotation_deformability,
        const bool optimize)
    : JacobianModel(optimize)
    , translation_deformability_(translation_deformability)
    , rotation_deformability_(rotation_deformability)
{
    if (!static_data_initialized_.load())
    {
        throw_arc_exception(std::runtime_error, "You must call SetInitialObjectConfiguration before constructing a DiminishingRigidityModel");
    }

    if (translation_deformability < 0)
    {
        throw_arc_exception(std::invalid_argument, "translation_deformability must be >= 0");
    }
    if (rotation_deformability < 0)
    {
        throw_arc_exception(std::invalid_argument, "rotation_deformability must be >= 0");
    }
}

////////////////////////////////////////////////////////////////////////////////
// Virtual function overrides
////////////////////////////////////////////////////////////////////////////////

void DiminishingRigidityModel::updateModel(const WorldState& previous, const WorldState& next)
{
    // This model doesn't do any updates, so tell the compiler that it's okay
    // that these values are unused.
    (void)previous;
    (void)next;
}

////////////////////////////////////////////////////////////////////////////////
// Helper used only by AdaptiveJacobian (at the moment)
// Find a better way to do this
////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd DiminishingRigidityModel::getGrippersToObjectJacobian(
        const AllGrippersSinglePose& grippers_pose,
        const ObjectPointSet& current_configuration) const
{
    return computeGrippersToObjectJacobian(grippers_pose, current_configuration);
}

////////////////////////////////////////////////////////////////////////////////
// Computation helpers
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief DiminishingRigidityModel::computeObjectToGripperJacobian
 * Computes a Jacobian that converts gripper velocities in the individual
 * gripper frames into object velocities in the world frame
 * @param grippers_data
 */
Eigen::MatrixXd DiminishingRigidityModel::computeGrippersToObjectJacobian(
        const AllGrippersSinglePose& grippers_pose,
        const ObjectPointSet& current_configuration) const
{
    const ssize_t num_grippers = (ssize_t)grippers_pose.size();
    const ssize_t num_Jcols = num_grippers * 6;
    const ssize_t num_Jrows = num_nodes_ * 3;

    MatrixXd J(num_Jrows, num_Jcols);

    // for each gripper
    for (ssize_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
    {
        // Get all the data we need for a given gripper
        const Matrix3d& gripper_rot = grippers_pose[(size_t)gripper_ind].rotation();

        for (ssize_t node_ind = 0; node_ind < num_nodes_; node_ind++)
        {
            const double dist_to_gripper =
                    getMinimumDistanceToGripper(
                        grippers_data_[(size_t)gripper_ind].node_indices,
                        node_ind, object_initial_node_distance_);

            const Matrix3d& J_trans = gripper_rot;

            J.block<3, 3>(node_ind * 3, gripper_ind * 6) =
                    std::exp(-translation_deformability_ * dist_to_gripper) * J_trans;

            // Calculate the cross product between the grippers x, y, and z axes
            // and the vector from the gripper to the node
            const Vector3d gripper_to_node =
                    current_configuration.col(node_ind) -
                    grippers_pose[(size_t)gripper_ind].translation();

            Matrix3d J_rot;
            J_rot.col(0) = gripper_rot.col(0).cross(gripper_to_node);
            J_rot.col(1) = gripper_rot.col(1).cross(gripper_to_node);
            J_rot.col(2) = gripper_rot.col(2).cross(gripper_to_node);

            J.block<3, 3>(node_ind * 3, gripper_ind * 6 + 3) =
                    std::exp(-rotation_deformability_ * dist_to_gripper) * J_rot;
        }
    }

    return J;
}
