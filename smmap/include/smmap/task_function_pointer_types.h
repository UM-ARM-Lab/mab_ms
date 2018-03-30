#ifndef TASK_FUNCTION_POINTER_TYPES_H
#define TASK_FUNCTION_POINTER_TYPES_H

#include <functional>

#include "smmap/trajectory.hpp"
#include "smmap/visualization_tools.h"

namespace smmap
{
    typedef std::function<std::vector<CollisionData>(
            const AllGrippersSinglePose& gripper_poses
            )>
    GripperCollisionCheckFunctionType;

    typedef std::function<ObjectDeltaAndWeight(
            const WorldState& world_current_state
            )>
    TaskDesiredObjectDeltaFunctionType;

    typedef std::function<void(
            const size_t test_id,
            const WorldState& resulting_world_state
            )>
    TestGrippersPosesFeedbackCallbackFunctionType;

    typedef std::function<void(
            const WorldState& current_world_state,
            const Eigen::VectorXd& model_utility_mean,
            const Eigen::MatrixXd& model_utility_covariance,
            const ssize_t model_used,
            const std::vector<double> rewards_for_all_models,
            const double correlation_strength_factor
            )>
    LoggingFunctionType;
}

#endif // TASK_FUNCTION_POINTER_TYPES_H
