#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include "smmap/trajectory.hpp"
#include "smmap/task_function_pointer_types.h"

namespace smmap
{
    /**
     * @brief OptimizeTrajectoryDirectShooting
     * @param world_initial_state
     * @param grippers_pose_trajectory
     * @param error_fn_
     * @param derivitive_fn_
     * @param prediction_fn_
     * @param max_gripper_delta
     * @param dt
     * @return
     */
    inline AllGrippersPoseTrajectory OptimizeTrajectoryDirectShooting(
            const WorldState& world_initial_state,
            AllGrippersPoseTrajectory grippers_pose_trajectory,
            const ErrorFunctionType& error_fn_,
            const ErrorFunctionDerivitiveType& derivitive_fn_,
            const ObjectFinalConfigurationPredictionFunctionType& prediction_fn_,
            const double max_gripper_delta,
            const double dt)
    {
        ROS_INFO_NAMED("optimization" , "Using direct shooting to optimize the trajectory");

        // TODO: move these magic numbers elsewhere
        #pragma message "Direct shooting magic numbers here need to be moved elsewhere"
        const int MAX_ITTR = 1000;
        const double LEARNING_RATE = 0.05;
        const double TOLERANCE = 1e-6;

        AllGrippersPoseDeltaTrajectory grippers_pose_deltas_trajectory =
                CalculateGrippersPoseDeltas(grippers_pose_trajectory);

        double error_value =
                error_fn_(
                    prediction_fn_(
                        world_initial_state,
                        grippers_pose_trajectory,
                        grippers_pose_deltas_trajectory,
                        dt));

        int ittr = 0;
        double error_improvement = std::numeric_limits<double>::infinity();
        do
        {
            ROS_INFO_STREAM_NAMED("optimization" , "  Direct shooting itteration " << ittr << ". Current error value " << error_value);

            // Find the first derivitive of the objective function with
            // respect to the gripper pose deltas
            const Eigen::VectorXd velocity_update = -derivitive_fn_(
                        world_initial_state,
                        grippers_pose_trajectory,
                        grippers_pose_deltas_trajectory,
                        dt);

            // create a new delta and pose trajectory to test
            const AllGrippersPoseDeltaTrajectory test_grippers_pose_deltas_trajectory =
                    ClampGripperPoseDeltas(
                        AddGripperPoseDeltas(
                            grippers_pose_deltas_trajectory,
                            LEARNING_RATE * velocity_update),
                        max_gripper_delta);

            // Update the trajectory of the grippers based on the new velocities
            const AllGrippersPoseTrajectory test_grippers_pose_trajectory =
                    CalculateGrippersTrajectory(
                        world_initial_state.all_grippers_single_pose_,
                        test_grippers_pose_deltas_trajectory);

            // Calculate the new value of the objective function at the updated velocity
            // locations
            const double new_error_value =
                    error_fn_(
                        prediction_fn_(
                            world_initial_state,
                            test_grippers_pose_trajectory,
                            test_grippers_pose_deltas_trajectory,
                            dt));

            error_improvement = error_value - new_error_value;

            // If we've reduced error, use the new trajectory
            if (error_improvement > 0)
            {
                grippers_pose_trajectory = test_grippers_pose_trajectory;
                grippers_pose_deltas_trajectory = test_grippers_pose_deltas_trajectory;
                error_value = new_error_value;
            }

            ittr++;
        }
        // Continue while we are still making meaningful improvement
        while (ittr < MAX_ITTR  && error_improvement > std::abs(error_value) * TOLERANCE);

        ROS_INFO_STREAM_NAMED("planner" , "  Direct shooting final objective value                  " << error_value);

        return grippers_pose_trajectory;
    }

    /**
     * @brief ErrorFunctionNumericalDerivitive
     * @param world_initial_state
     * @param grippers_pose_trajectory
     * @param grippers_pose_deltas_trajectory
     * @param error_fn_
     * @param prediction_fn_
     * @param dt
     * @return
     */
    inline Eigen::VectorXd ErrorFunctionNumericalDerivitive(
            const WorldState& world_initial_state,
            AllGrippersPoseTrajectory grippers_pose_trajectory,
            AllGrippersPoseDeltaTrajectory grippers_pose_deltas_trajectory,
            const ErrorFunctionType& error_fn_,
            const ObjectFinalConfigurationPredictionFunctionType& prediction_fn_,
            const double dt)
    {
        assert(grippers_pose_trajectory.size() > 1);
        assert(grippers_pose_trajectory.size() - 1 == grippers_pose_deltas_trajectory.size());
        const size_t num_timesteps = grippers_pose_trajectory.size() - 1;
        const size_t num_grippers = grippers_pose_trajectory[0].size();
        assert(num_grippers > 0);

        #pragma message "Arbitrary step size for numeric differencing"
        const double h = 1e-6;

        // Allocate some space to store the results of the differencing.
        Eigen::VectorXd derivitives((ssize_t)(num_timesteps * num_grippers * 6));

        // Note that I am following the math found on the Finite difference page of
        // Wikipedia for "finite difference in several variables"
        // This loop fills out the Jacobian (first derivitive) of the objective function
        for (ssize_t ind = 0; ind < (ssize_t)(num_grippers * 6 * num_timesteps); ind++)
        {
            const ssize_t time_ind = ind / (ssize_t)(num_grippers * 6);
            const ssize_t vel_ind = ind % (ssize_t)(num_grippers * 6);
            AllGrippersPoseDeltaTrajectory new_grippers_pose_deltas_trajectory(grippers_pose_deltas_trajectory);
            AllGrippersPoseTrajectory new_grippers_pose_trajectory;

            // f(x + h, y)
            new_grippers_pose_deltas_trajectory[(size_t)time_ind][(size_t)vel_ind / 6](vel_ind  % 6) += h;
            new_grippers_pose_trajectory = CalculateGrippersTrajectory(grippers_pose_trajectory[0], new_grippers_pose_deltas_trajectory);
            const ObjectPointSet object_config_x_plus_h =
                    prediction_fn_(world_initial_state,
                                   new_grippers_pose_trajectory,
                                   new_grippers_pose_deltas_trajectory,
                                   dt);
            const double error_value_x_plus_h = error_fn_(object_config_x_plus_h);
            new_grippers_pose_deltas_trajectory = grippers_pose_deltas_trajectory;

            // f(x - h, y)
            new_grippers_pose_deltas_trajectory[(size_t)time_ind][(size_t)vel_ind / 6](vel_ind  % 6) -= h;
            new_grippers_pose_trajectory = CalculateGrippersTrajectory(grippers_pose_trajectory[0], new_grippers_pose_deltas_trajectory);
            const ObjectPointSet object_config_x_minus_h =
                    prediction_fn_(world_initial_state,
                                   new_grippers_pose_trajectory,
                                   new_grippers_pose_deltas_trajectory,
                                   dt);
            const double error_value_x_minus_h = error_fn_(object_config_x_minus_h);

            // f_x = [ f(x + h, y) - f(x - h, y) ] / ( 2h )
            derivitives( ind ) = ( error_value_x_plus_h - error_value_x_minus_h ) / ( 2*h );
        }
        return derivitives;
    }
}

#endif // OPTIMIZATION_HPP
