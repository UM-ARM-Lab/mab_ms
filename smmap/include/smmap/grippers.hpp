#ifndef GRIPPERS_HPP
#define GRIPPERS_HPP

#include <limits>
#include <memory>
#include <assert.h>

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <kinematics_toolbox/kinematics.h>
#include <smmap_experiment_params/ros_params.hpp>

namespace smmap
{
    // Forward delcaration, because world state includes stuff defined here
    struct WorldState;

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////               Kinematics Related functionality                     ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    typedef EigenHelpers::VectorAffine3d AllGrippersSinglePose;
    typedef std::vector<AllGrippersSinglePose> AllGrippersPoseTrajectory;

    typedef kinematics::VectorVector6d AllGrippersSinglePoseDelta;
    typedef std::vector<AllGrippersSinglePoseDelta> AllGrippersPoseDeltaTrajectory;

    struct GripperData
    {
        GripperData(const std::string& name, const std::vector<long>& node_indices)
            : name(name)
            , node_indices(node_indices)
        {}

        /// The name associated with this gripper
        std::string name;

        /// Vector of the indices of the nodes that are grasped by the gripper
        std::vector<long> node_indices;

        /**
         * @brief operator <<
         * @param out The stream to output the data too
         * @param data The gripper data to output
         * @return
         */
        friend std::ostream& operator<< (std::ostream& out, const GripperData& data)
        {
            out << data.name << " Node Indices: " << PrettyPrint::PrettyPrint(data.node_indices);
            return out;
        }
    };

    template<typename T>
    inline std::vector<long > VectorAnytypeToVectorLong(
            const std::vector<T>& vector_anytype)
    {
        std::vector<long> vector_signed(vector_anytype.size());
        for (size_t ind = 0; ind < vector_anytype.size(); ind++)
        {
            vector_signed[ind] = (long)(vector_anytype[ind]);
        }
        return vector_signed;
    }

    inline std::vector<std::string> GetGripperNames(
            const std::vector<GripperData> grippers_data)
    {
        std::vector<std::string> names(grippers_data.size());

        for (size_t gripper_ind = 0; gripper_ind < grippers_data.size(); gripper_ind++)
        {
            names[gripper_ind] = grippers_data[gripper_ind].name;
        }

        return names;
    }

    /**
     * @brief getMinimumDistanceToGripper
     * @param gripper_indices The indices of the nodes that the gripper is in contact with
     * @param node_index The index of the node that we want to get the distance to
     * @param object_initial_node_distance The matrix of distances between nodes
     * @return The the distance between given node, and the closest node grasped by the gripper
     */
    inline double getMinimumDistanceToGripper(
            const std::vector<long>& gripper_indices,
            long node_index,
            const Eigen::MatrixXd& object_initial_node_distance)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        for (long ind: gripper_indices)
        {
            min_dist = std::min(min_dist, object_initial_node_distance(ind, node_index));
        }

        return min_dist;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Basic Math
    ////////////////////////////////////////////////////////////////////////////

    inline AllGrippersPoseDeltaTrajectory AddGripperPoseDeltas(
            AllGrippersPoseDeltaTrajectory lhs, const Eigen::VectorXd& rhs)
    {
        const size_t num_timesteps = lhs.size();
        assert(num_timesteps > 0);
        const size_t num_grippers = lhs[0].size();
        assert(num_grippers > 0);
        assert(num_timesteps * num_grippers * 6 == (size_t)rhs.rows());

        for (size_t time_ind = 0; time_ind < num_timesteps; time_ind++)
        {
            for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
            {
                lhs[time_ind][gripper_ind] += rhs.segment<6>((ssize_t)(time_ind * num_grippers * 6 + gripper_ind * 6));
            }
        }

        return lhs;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Dot products
    ////////////////////////////////////////////////////////////////////////////

    #pragma message "SE3 velocity weight hard coded here"
    inline double GripperVelocityDotProduct(
            const kinematics::Vector6d& vel1,
            const kinematics::Vector6d& vel2)
    {
        kinematics::Vector6d weight = kinematics::Vector6d::Ones();
        weight(3) = 1.0/20.0;
        weight(4) = 1.0/20.0;
        weight(5) = 1.0/20.0;
        weight.array() = weight.array().square();

        return EigenHelpers::WeightedDotProduct(vel1, vel2, weight);
    }

    inline double MultipleGrippersVelocityDotProduct(
            const AllGrippersSinglePoseDelta& vel1,
            const AllGrippersSinglePoseDelta& vel2)
    {
        assert(vel1.size() == vel2.size());

        double dot_product = 0;
        for (size_t vel_ind = 0; vel_ind < vel1.size(); vel_ind++)
        {
            dot_product += GripperVelocityDotProduct(vel1[vel_ind], vel2[vel_ind]);
        }

        return dot_product;
    }

    inline double MultipleGrippersVelocityTrajectoryDotProduct(
            const AllGrippersPoseDeltaTrajectory& traj1,
            const AllGrippersPoseDeltaTrajectory& traj2)
    {
        assert(traj1.size() == traj2.size());

        double dot_product = 0;
        for (size_t time_ind = 0; time_ind < traj1.size(); time_ind++)
        {
            dot_product += MultipleGrippersVelocityDotProduct(traj1[time_ind], traj2[time_ind]);
        }

        return dot_product;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Conversion functions
    ////////////////////////////////////////////////////////////////////////////

    inline AllGrippersSinglePoseDelta CalculateGrippersPoseDelta(
            const AllGrippersSinglePose& prev,
            const AllGrippersSinglePose& next)
    {
        const size_t num_grippers = prev.size();
        assert(num_grippers > 0);
        assert(num_grippers == next.size());
        AllGrippersSinglePoseDelta grippers_pose_delta(num_grippers);
        for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
        {
            grippers_pose_delta[gripper_ind] =
                    kinematics::calculateError(
                        prev[gripper_ind],
                        next[gripper_ind]);
        }
        return grippers_pose_delta;
    }

    /**
     * @brief CalculateGrippersPoseDeltas
     * @param grippers_trajectory
     * @return
     */
    inline AllGrippersPoseDeltaTrajectory CalculateGrippersPoseDeltas(
            const AllGrippersPoseTrajectory& grippers_trajectory)
    {
        assert(grippers_trajectory.size() > 1);
        const size_t num_grippers = grippers_trajectory[0].size();

        AllGrippersPoseDeltaTrajectory grippers_pose_delta_traj(
                    grippers_trajectory.size() - 1,
                    AllGrippersSinglePoseDelta(num_grippers));

        for (size_t time_ind = 0; time_ind < grippers_pose_delta_traj.size(); time_ind++)
        {
            grippers_pose_delta_traj[time_ind] =
                    CalculateGrippersPoseDelta(
                        grippers_trajectory[time_ind],
                        grippers_trajectory[time_ind + 1]);
        }

        return grippers_pose_delta_traj;
    }

    /**
     * @brief CalculateGrippersTrajectory
     * @param grippers_initial_pose
     * @param grippers_pose_deltas
     * @return
     */
    inline AllGrippersPoseTrajectory CalculateGrippersTrajectory(
            const AllGrippersSinglePose& grippers_initial_pose,
            const AllGrippersPoseDeltaTrajectory& grippers_pose_deltas)
    {
        const size_t num_grippers = grippers_initial_pose.size();
        const size_t num_timesteps = grippers_pose_deltas.size();

        AllGrippersPoseTrajectory grippers_pose_trajectory(
                    num_timesteps + 1,
                    AllGrippersSinglePose(num_grippers));

        grippers_pose_trajectory[0] = grippers_initial_pose;

        for (size_t time_ind = 0; time_ind < num_timesteps; time_ind++)
        {
            for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind ++)
            {
                grippers_pose_trajectory[time_ind+1][gripper_ind] =
                        grippers_pose_trajectory[time_ind][gripper_ind] *
                        kinematics::expTwistAffine3d(grippers_pose_deltas[time_ind][gripper_ind], 1);
            }
        }

        return grippers_pose_trajectory;
    }

    /**
     * @brief CalculateGrippersTrajectory
     * @param grippers_initial_pose
     * @param grippers_pose_deltas
     * @return
     */
    inline AllGrippersPoseTrajectory CalculateGrippersTrajectory(
            const AllGrippersSinglePose &grippers_initial_pose,
            const Eigen::VectorXd &grippers_pose_deltas)
    {
        const size_t num_grippers = grippers_initial_pose.size();
        const size_t num_timesteps = (size_t)grippers_pose_deltas.size() / (num_grippers * 6);
        assert((size_t)grippers_pose_deltas.size() % num_timesteps == 0);

        AllGrippersPoseTrajectory grippers_pose_trajectory(
                    num_timesteps + 1,
                    AllGrippersSinglePose(num_grippers));

        grippers_pose_trajectory[0] = grippers_initial_pose;

        for (size_t time_ind = 0; time_ind < num_timesteps; time_ind++)
        {
            for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind ++)
            {
                const kinematics::Vector6d gripper_delta =
                        grippers_pose_deltas.segment<6>(
                            (ssize_t)(time_ind * num_grippers * 6 + gripper_ind * 6));

                grippers_pose_trajectory[time_ind+1][gripper_ind] =
                        grippers_pose_trajectory[time_ind][gripper_ind] *
                        kinematics::expTwistAffine3d(gripper_delta, 1);
            }
        }

        return grippers_pose_trajectory;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Norms induced by said dot products
    ////////////////////////////////////////////////////////////////////////////

    inline double GripperVelocity6dSquaredNorm(const kinematics::Vector6d& gripper_velocity)
    {
        return GripperVelocityDotProduct(gripper_velocity, gripper_velocity);
    }

    inline double GripperVelocity6dNorm(const kinematics::Vector6d& gripper_velocity)
    {
        return std::sqrt(GripperVelocity6dSquaredNorm(gripper_velocity));
    }

    inline double MultipleGrippersVelocity6dSquaredNorm(const AllGrippersSinglePoseDelta& grippers_velocity)
    {
        double squared_norm = 0;
        for (size_t gripper_ind = 0; gripper_ind < grippers_velocity.size(); gripper_ind++)
        {
            squared_norm += GripperVelocity6dSquaredNorm(grippers_velocity[gripper_ind]);
        }
        return squared_norm;
    }

    inline double MultipleGrippersVelocity6dSquaredNorm(const Eigen::VectorXd& grippers_velocity)
    {
        assert(grippers_velocity.size() % 6 == 0);

        double squared_norm = 0;
        for (long gripper_ind = 0; gripper_ind < grippers_velocity.size(); gripper_ind += 6)
        {
            squared_norm += GripperVelocity6dSquaredNorm(grippers_velocity.segment<6>(gripper_ind));
        }
        return squared_norm;
    }

    inline double MultipleGrippersVelocity6dNorm(const AllGrippersSinglePoseDelta& grippers_velocity)
    {
        return std::sqrt(MultipleGrippersVelocity6dSquaredNorm(grippers_velocity));
    }

    inline double MultipleGrippersVelocity6dNorm(const Eigen::VectorXd& grippers_velocity)
    {
        return std::sqrt(MultipleGrippersVelocity6dSquaredNorm(grippers_velocity));
    }

    inline double MultipleGrippersVelocityTrajectory6dSquaredNorm(const AllGrippersPoseDeltaTrajectory& grippers_trajectory)
    {
        double squared_norm = 0;
        for (size_t time_ind = 0; time_ind < grippers_trajectory.size(); time_ind++)
        {
            squared_norm += MultipleGrippersVelocity6dSquaredNorm(grippers_trajectory[time_ind]);
        }

        return squared_norm;
    }

    inline double MultipleGrippersVelocityTrajectory6dNorm(const AllGrippersPoseDeltaTrajectory& grippers_trajectory)
    {
        return std::sqrt(MultipleGrippersVelocityTrajectory6dSquaredNorm(grippers_trajectory));
    }

    inline Eigen::VectorXd ClampGripperPoseDeltas(Eigen::VectorXd velocities, const double max_pose_delta)
    {
        assert(velocities.size() % 6 == 0);
        for (ssize_t vel_ind = 0; vel_ind < velocities.size(); vel_ind += 6)
        {
            const double velocity_norm = GripperVelocity6dNorm(velocities.segment<6>(vel_ind));
            if (velocity_norm > max_pose_delta)
            {
                velocities.segment<6>(vel_ind) *= max_pose_delta / velocity_norm;
            }
        }
        return velocities;
    }

    inline AllGrippersSinglePoseDelta ClampGripperPoseDeltas(AllGrippersSinglePoseDelta pose_deltas, const double max_delta)
    {
        for (size_t gripper_ind = 0; gripper_ind < pose_deltas.size(); gripper_ind++)
        {
            pose_deltas[gripper_ind] = ClampGripperPoseDeltas(pose_deltas[gripper_ind], max_delta);
        }
        return pose_deltas;
    }

    inline AllGrippersPoseDeltaTrajectory ClampGripperPoseDeltas(AllGrippersPoseDeltaTrajectory pose_deltas, const double max_delta)
    {
        for (size_t time_ind = 0; time_ind < pose_deltas.size(); time_ind++)
        {
            pose_deltas[time_ind] = ClampGripperPoseDeltas(pose_deltas[time_ind], max_delta);
        }
        return pose_deltas;
    }


    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Collision related functionality                     ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /// Stores the result of a collision check from bullet (or similar)
    struct CollisionData
    {
        public:
            CollisionData(const Eigen::Vector3d& nearest_point_to_obstacle,
                           const Eigen::Vector3d& obstacle_surface_normal,
                           const double distance_to_obstacle)
                : nearest_point_to_obstacle_(nearest_point_to_obstacle)
                , obstacle_surface_normal_(obstacle_surface_normal)
                , distance_to_obstacle_(distance_to_obstacle)
            {}

            CollisionData(Eigen::Vector3d&& nearest_point_to_obstacle,
                           Eigen::Vector3d&& obstacle_surface_normal,
                           const double distance_to_obstacle)
                : nearest_point_to_obstacle_(nearest_point_to_obstacle)
                , obstacle_surface_normal_(obstacle_surface_normal)
                , distance_to_obstacle_(distance_to_obstacle)
            {}

            Eigen::Vector3d nearest_point_to_obstacle_;
            Eigen::Vector3d obstacle_surface_normal_;
            double distance_to_obstacle_;
    };

    /// Stores the result of a collision avoidance calculation for a single gripper
    struct CollisionAvoidanceResult
    {
        public:
            CollisionAvoidanceResult()
                : nullspace_projector(Eigen::Matrix<double, 6, 6>::Identity())
                , velocity(Eigen::Matrix<double, 6, 1>::Zero())
                , distance(std::numeric_limits<double>::infinity())
            {}
            Eigen::Matrix<double, 6, 6> nullspace_projector;
            Eigen::Matrix<double, 6, 1> velocity;
            double distance;
    };

    class GripperCollisionChecker
    {
        public:
            GripperCollisionChecker(ros::NodeHandle& nh)
            {
                collision_checker_client_ =
                    nh.serviceClient<smmap_msgs::GetGripperCollisionReport>(GetGripperCollisionCheckTopic(nh));

                collision_checker_client_.waitForExistence();
            }

            std::vector<CollisionData> gripperCollisionCheck(
                    const AllGrippersSinglePose& gripper_poses)
            {
                smmap_msgs::GetGripperCollisionReport collision_report_ros;
                collision_report_ros.request.pose =
                        EigenHelpersConversions::VectorAffine3dToVectorGeometryPose(gripper_poses);

                if (!collision_checker_client_.call(collision_report_ros))
                {
                    ROS_FATAL_NAMED("gripper collision check", "Unable to retrieve gripper collision report.");
                }

                std::vector<CollisionData> collision_report_eigen;
                collision_report_eigen.reserve(gripper_poses.size());

                for (size_t gripper_ind = 0; gripper_ind < gripper_poses.size(); gripper_ind++)
                {
                    collision_report_eigen.push_back(
                                CollisionData(
                                    EigenHelpersConversions::GeometryPointToEigenVector3d(
                                        collision_report_ros.response.gripper_nearest_point_to_obstacle[gripper_ind]),
                                    EigenHelpersConversions::GeometryVector3ToEigenVector3d(
                                        collision_report_ros.response.obstacle_surface_normal[gripper_ind]),
                                    collision_report_ros.response.gripper_distance_to_obstacle[gripper_ind]));
                }

                return collision_report_eigen;
            }

        private:
            ros::ServiceClient collision_checker_client_;
    };

    /**
     * @brief ComputeCollisionToGripperJacobian
     * @param point_on_gripper
     * @param gripper_pose
     * @return
     */
    inline Eigen::Matrix<double, 3, 6> ComputeCollisionToGripperJacobian(
            const Eigen::Vector3d& point_on_gripper,
            const Eigen::Affine3d& gripper_pose)
    {

        Eigen::Matrix<double, 3, 6> J_collision;
        const Eigen::Matrix3d gripper_rot = gripper_pose.rotation();

        // Translation - if I move the gripper along its x/y/z-axis, what happens to the given point?
        J_collision.block<3, 3>(0, 0) = gripper_rot;

        const Eigen::Vector3d gripper_to_point_in_collision =
                point_on_gripper - gripper_pose.translation();

        // If I rotate the gripper about its x/y/z-axis, what happens to the point in question?
        J_collision.block<3, 1>(0, 3) = gripper_rot.block<3, 1>(0, 0).cross(gripper_to_point_in_collision);
        J_collision.block<3, 1>(0, 4) = gripper_rot.block<3, 1>(0, 1).cross(gripper_to_point_in_collision);
        J_collision.block<3, 1>(0, 5) = gripper_rot.block<3, 1>(0, 2).cross(gripper_to_point_in_collision);

        return J_collision;
    }

    /**
     * @brief ComputeGripperObjectAvoidance
     * @param collision_data
     * @param gripper_pose
     * @param max_step_size
     * @return
     */
    inline CollisionAvoidanceResult ComputeGripperObjectAvoidance(
            const CollisionData& collision_data,
            const Eigen::Affine3d& gripper_pose,
            double max_step_size)
    {
        CollisionAvoidanceResult collision_avoidance_result;

        collision_avoidance_result.distance = collision_data.distance_to_obstacle_;

        // If we have a collision to avoid, then find the vector
        if (!std::isinf(collision_data.distance_to_obstacle_))
        {
            // Create the collision Jacobian
            const Eigen::Matrix<double, 3, 6> J_collision =
                    ComputeCollisionToGripperJacobian(
                        collision_data.nearest_point_to_obstacle_, gripper_pose);
            const Eigen::Matrix<double, 6, 3> J_collision_inv =
                    EigenHelpers::Pinv(J_collision, EigenHelpers::SuggestedRcond());

            // Create the collision avoidance vector to follow
            const Eigen::Vector3d& avoid_collision_delta = collision_data.obstacle_surface_normal_;

            collision_avoidance_result.velocity =  J_collision_inv * avoid_collision_delta;
            collision_avoidance_result.velocity *=
                    max_step_size / GripperVelocity6dNorm(collision_avoidance_result.velocity);

            collision_avoidance_result.nullspace_projector =
                    Eigen::Matrix<double, 6, 6>::Identity() - J_collision_inv * J_collision;
        }
        // Otherwise, leave the collision result as the default "no collision" state
        else {}

        return collision_avoidance_result;
    }

    inline std::vector<CollisionAvoidanceResult> ComputeGripperObjectAvoidance(
            const std::vector<CollisionData>& collision_data,
            const EigenHelpers::VectorAffine3d& gripper_pose,
            double max_step_size)
    {
        std::vector<CollisionAvoidanceResult> collision_avoidance_results;
        collision_avoidance_results.reserve(collision_data.size());

        for (size_t gripper_ind = 0; gripper_ind < gripper_pose.size(); gripper_ind++)
        {
            collision_avoidance_results.push_back(
                        ComputeGripperObjectAvoidance(
                                    collision_data[gripper_ind],
                                    gripper_pose[gripper_ind],
                                    max_step_size));
        }

        return collision_avoidance_results;
    }

    inline kinematics::Vector6d CombineDesiredAndObjectAvoidance(
            const kinematics::Vector6d& desired_motion,
            const CollisionAvoidanceResult& collision_result,
            const double obstacle_avoidance_scale)
    {
        if (!std::isinf(collision_result.distance))
        {
             const double collision_severity = std::min(1.0, std::exp(-obstacle_avoidance_scale * (collision_result.distance - GetRobotMinGripperDistance())));
             return collision_severity * (collision_result.velocity + collision_result.nullspace_projector * desired_motion) + (1.0 - collision_severity) * desired_motion;
        }
        // Otherwise use our desired velocity directly
        else
        {
             return desired_motion;
        }
    }

    inline std::vector<kinematics::Vector6d> CombineDesiredAndObjectAvoidance(
            const std::vector<kinematics::Vector6d>& desired_motion,
            const std::vector<CollisionAvoidanceResult>& collision_result,
            const double obstacle_avoidance_scale)
    {
        assert(desired_motion.size() == collision_result.size());
        std::vector<kinematics::Vector6d> result(desired_motion.size());
        for (size_t gripper_ind = 0; gripper_ind < desired_motion.size(); gripper_ind++)
        {
            result[gripper_ind] = CombineDesiredAndObjectAvoidance(
                        desired_motion[gripper_ind],
                        collision_result[gripper_ind],
                        obstacle_avoidance_scale);
        }
        return result;
    }
}

#endif // GRIPPERS_HPP
