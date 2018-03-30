#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <cmath>
#include <string>
#include <chrono>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/arc_exceptions.hpp>

#include "smmap_experiment_params/task_enums.h"

namespace smmap
{
    inline float GetClothXSize(ros::NodeHandle &nh);

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    inline DeformableType GetDeformableType(ros::NodeHandle& nh)
    {
        std::string deformable_type = ROSHelpers::GetParam<std::string>(nh, "deformable_type", "rope");

        if (deformable_type.compare("rope") == 0)
        {
            return DeformableType::ROPE;
        }
        else if (deformable_type.compare("cloth") == 0)
        {
            return DeformableType::CLOTH;
        }
        else
        {
            ROS_FATAL_STREAM("Unknown deformable type: " << deformable_type);
            throw_arc_exception(std::invalid_argument, "Unknown deformable type: " + deformable_type);
        }
    }

    inline TaskType GetTaskType(ros::NodeHandle& nh)
    {
        std::string task_type = ROSHelpers::GetParam<std::string>(nh, "task_type", "coverage");

        if (task_type.compare("cylinder_coverage") == 0)
        {
            return TaskType::CYLINDER_COVERAGE;
        }
        else if (task_type.compare("table_coverage") == 0)
        {
            return TaskType::TABLE_COVERAGE;
        }
        else if (task_type.compare("colab_folding") == 0)
        {
            return TaskType::COLAB_FOLDING;
        }
        else if (task_type.compare("wafr") == 0)
        {
            return TaskType::WAFR;
        }
        else
        {
            ROS_FATAL_STREAM("Unknown task type: " << task_type);
            throw_arc_exception(std::invalid_argument, "Unknown task type: " + task_type);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Table Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetTableSurfaceX(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_x", 0.0f);
    }

    inline float GetTableSurfaceY(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_y", 0.0f);
    }

    inline float GetTableSurfaceZ(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_z", 0.7f);
    }

    inline float GetTableHalfExtentsX(ros::NodeHandle& nh)       // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "table_x_size", 1.5f);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "table_x_size", 0.2f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown table size for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    inline float GetTableHalfExtentsY(ros::NodeHandle& nh)       // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "table_y_size", 1.5f);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "table_y_size", 0.2f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown table size for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    inline float GetTableSizeZ(ros::NodeHandle& nh)       // METERS
    {
        return ROSHelpers::GetParam(nh, "table_z_size", GetTableSurfaceZ(nh));
    }

    inline float GetTableLegWidth(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "table_leg_width", 0.05f);
    }

    inline float GetTableThickness(ros::NodeHandle& nh)   // METERS
    {
        return ROSHelpers::GetParam(nh, "table_thickness", 0.05f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cylinder Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetCylinderRadius(ros::NodeHandle& nh)   // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_radius", 0.15f);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_radius", 0.10f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder radius for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    inline float GetCylinderHeight(ros::NodeHandle& nh)   // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_height", 0.3f);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_height", 0.3f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder height for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    inline float GetCylinderCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_x", GetTableSurfaceX(nh));

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_x", GetTableSurfaceX(nh) - GetClothXSize(nh));

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder com for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    inline float GetCylinderCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_y", GetTableSurfaceY(nh) + GetCylinderRadius(nh) * 5.0f / 3.0f);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_y", GetTableSurfaceY(nh));

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder com for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    inline float GetCylinderCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_z", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) / 2.0f);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_z", GetTableSurfaceZ(nh));

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder com for deformable type " + std::to_string(GetDeformableType(nh)));
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeSegmentLength(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_segment_length", 0.025f);
    }

    inline float GetRopeRadius(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_radius", 0.01f);
    }

    inline int GetRopeNumLinks(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "rope_num_links", 50);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    // TODO: merge this and the cloth version into a single param?
    inline float GetRopeGripperApperture(ros::NodeHandle& nh) // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_gripper_apperture", 0.03f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope-Cylinder experiment settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_com_x", GetTableSurfaceX(nh) + 15.0f * GetRopeSegmentLength(nh));
    }

    inline float GetRopeCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_com_y", GetTableSurfaceY(nh));
    }

    inline float GetRopeCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_com_z", GetTableSurfaceZ(nh) + 5.0f * GetRopeRadius(nh));
    }

    ////////////////////////////////////////////////////////////////////
    // Cloth settings
    ////////////////////////////////////////////////////////////////////

    inline float GetClothXSize(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_x_size", 0.5f);
    }

    inline float GetClothYSize(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_y_size", 0.5f);
    }

    inline float GetClothCenterOfMassX(ros::NodeHandle& nh)   // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::COLAB_FOLDING:
            case TaskType::TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "cloth_com_x", GetTableSurfaceX(nh) + GetClothXSize(nh) / 2.0f);

            case TaskType::WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_x", GetCylinderCenterOfMassX(nh) + GetCylinderRadius(nh) * 1.5f + GetClothXSize(nh) / 2.0f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cloth com X for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    inline float GetClothCenterOfMassY(ros::NodeHandle& nh)   // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_com_y", GetTableSurfaceY(nh));
    }

    inline float GetClothCenterOfMassZ(ros::NodeHandle& nh)   // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_com_z", GetTableSurfaceZ(nh) + 0.01f);
    }

    inline float GetClothLinearStiffness(ros::NodeHandle& nh)
    {
        switch(GetTaskType(nh))
        {
            case TaskType::TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "cloth_linear_stiffness", 0.8f);

            case TaskType::COLAB_FOLDING:
            case TaskType::WAFR:
                return ROSHelpers::GetParam(nh, "cloth_linear_stiffness", 0.5f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cloth linear stiffness for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cloth BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    inline int GetClothNumDivsX(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cloth_num_divs_x", 45);
    }

    inline int GetClothNumDivsY(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cloth_num_divs_y", 45);
    }

    inline float GetClothGripperApperture(ros::NodeHandle& nh) // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_gripper_apperture", 0.1f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetFeedbackCovariance(ros::NodeHandle& nh)  // METERS^2
    {
        return ROSHelpers::GetParam(nh, "feedback_covariance", 0.0);
    }

    inline size_t GetNumSimstepsPerGripperCommand(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "num_simsteps_per_gripper_command", 4);
    }

    ////////////////////////////////////////////////////////////////////////////
    // (Fake) Robot settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetRobotControlPeriod(ros::NodeHandle& nh) // SECONDS
    {
        return ROSHelpers::GetParam(nh, "robot_control_rate", 0.01);
    }

    inline double GetMaxGripperVelocity(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "max_gripper_velocity", 0.2);
    }

    ////////////////////////////////////////////////////////////////////////////
    // World size settings for Graph/Dijkstras
    ////////////////////////////////////////////////////////////////////////////

    inline double GetWorldXStep(ros::NodeHandle& nh)    // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_x_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_x_step", 0.02);

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldXMin(ros::NodeHandle& nh)     // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_x_min", GetTableSurfaceX(nh) - GetTableHalfExtentsX(nh));

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_x_min", GetClothCenterOfMassX(nh) - 1.4 * GetClothXSize(nh));

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldXMax(ros::NodeHandle& nh)     // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_x_max", GetTableSurfaceX(nh) + GetTableHalfExtentsX(nh));

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_x_max", GetClothCenterOfMassX(nh) + 0.4 * GetClothXSize(nh));

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline int64_t GetWorldXNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldXMax(nh) - GetWorldXMin(nh))/GetWorldXStep(nh)) + 1;
    }

    inline double GetWorldYStep(ros::NodeHandle& nh)    // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_y_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_y_step", 0.02);

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldYMin(ros::NodeHandle& nh)     // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_y_min", GetTableSurfaceY(nh) - GetTableHalfExtentsY(nh));

            case DeformableType::CLOTH:
                switch (GetTaskType(nh))
                {
                    case TaskType::COLAB_FOLDING:
                        return -0.05;

                    case TaskType::TABLE_COVERAGE:
                        return ROSHelpers::GetParam(nh, "world_y_min", GetClothCenterOfMassY(nh) - 0.65 * GetClothYSize(nh));

                    case TaskType::CYLINDER_COVERAGE:
                    case TaskType::WAFR:
                        return ROSHelpers::GetParam(nh, "world_y_min", GetClothCenterOfMassY(nh) - 0.75 * GetClothYSize(nh));

                    default:
                        ROS_FATAL_STREAM("Unknown task type for " << __func__);
                        throw_arc_exception(std::invalid_argument, std::string("Unknown task type for ") + __func__);
                };

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldYMax(ros::NodeHandle& nh)     // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_y_max", GetTableSurfaceY(nh) + GetTableHalfExtentsY(nh));

            case DeformableType::CLOTH:
                switch (GetTaskType(nh))
                {
                    case TaskType::COLAB_FOLDING:
                        return 0.05;

                    case TaskType::TABLE_COVERAGE:
                        return ROSHelpers::GetParam(nh, "world_y_max", GetClothCenterOfMassY(nh) + 0.65 * GetClothYSize(nh));

                    case TaskType::CYLINDER_COVERAGE:
                    case TaskType::WAFR:
                        return ROSHelpers::GetParam(nh, "world_y_max", GetClothCenterOfMassY(nh) + 0.75 * GetClothYSize(nh));

                    default:
                        ROS_FATAL_STREAM("Unknown task type for " << __func__);
                        throw_arc_exception(std::invalid_argument, std::string("Unknown task type for ") + __func__);
                };

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline int64_t GetWorldYNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldYMax(nh) - GetWorldYMin(nh))/GetWorldYStep(nh)) + 1;
    }

    inline double GetWorldZStep(ros::NodeHandle& nh)    // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_z_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_z_step", 0.02);

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldZMin(ros::NodeHandle& nh)     // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_z_min", GetTableSurfaceZ(nh));

            case DeformableType::CLOTH:
                switch (GetTaskType(nh))
                {
                    case TaskType::COLAB_FOLDING:
                        return -0.05;

                    case TaskType::TABLE_COVERAGE:
                        return ROSHelpers::GetParam(nh, "world_z_min", GetClothCenterOfMassY(nh) - 0.65 * GetClothXSize(nh));

                    case TaskType::CYLINDER_COVERAGE:
                    case TaskType::WAFR:
                        return ROSHelpers::GetParam(nh, "world_z_min", GetClothCenterOfMassZ(nh) - 1.0 * GetClothXSize(nh));

                    default:
                        ROS_FATAL_STREAM("Unknown task type for " << __func__);
                        throw_arc_exception(std::invalid_argument, std::string("Unknown task type for ") + __func__);
                };

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldZMax(ros::NodeHandle& nh)     // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_z_max", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) + GetRopeRadius(nh) * 5.0);

            case DeformableType::CLOTH:
                switch (GetTaskType(nh))
                {
                    case TaskType::COLAB_FOLDING:
                        return 0.05;

                    case TaskType::TABLE_COVERAGE:
                        return ROSHelpers::GetParam(nh, "world_z_max", GetClothCenterOfMassY(nh) + 0.1 * GetClothXSize(nh));

                    case TaskType::CYLINDER_COVERAGE:
                    case TaskType::WAFR:
                        return ROSHelpers::GetParam(nh, "world_z_max", GetClothCenterOfMassZ(nh) + 0.5 * GetClothXSize(nh));

                    default:
                        ROS_FATAL_STREAM("Unknown task type for " << __func__);
                        throw_arc_exception(std::invalid_argument, std::string("Unknown task type for ") + __func__);
                };

            default:
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline int64_t GetWorldZNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldZMax(nh) - GetWorldZMin(nh))/GetWorldZStep(nh)) + 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetUseMultiModel(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_multi_model", false);
    }

    inline bool GetUseAdaptiveModel(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_adaptive_model", false);
    }

    inline double GetAdaptiveModelLearningRate(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "adaptive_model_learning_rate", 1e-6);
    }

    inline int GetPlanningHorizon(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "planning_horizion", 1);
    }

    inline double GetProcessNoiseFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "process_noise_factor", 0.1);
    }

    inline double GetObservationNoiseFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "observation_noise_factor", 0.01);
    }

    inline double GetCorrelationStrengthFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "correlation_strength_factor", 0.9);
    }

    inline double GetMaxCorrelationStrengthFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "max_correlation_strength_factor", GetCorrelationStrengthFactor(nh));
    }

    inline bool GetOptimizationEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "optimization_enabled", false);
    }

    inline double GetRobotMinGripperDistance()
    {
        return 0.005;
    }

    inline bool GetCalculateRegret(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "calculate_regret", false);
    }

    inline bool GetUseRandomSeed(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_random_seed", false);
    }

    inline size_t GetPlannerSeed(ros::NodeHandle& nh)
    {
        size_t seed;
        if (GetUseRandomSeed(nh))
        {
            assert(nh.hasParam("static_seed") == false);
            seed = std::chrono::system_clock::now().time_since_epoch().count();
        }
        else
        {
            std::string seed_as_string = ROSHelpers::GetParam<std::string>(nh, "static_seed", "a8710913d2b5df6c"); // a30cd67f3860ddb3) // MD5 sum of "Dale McConachie"
            std::stringstream ss;
            ss << std::hex << seed_as_string;
            ss >> seed;
        }
        return seed;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetLoggingEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "logging_enabled", false);
    }

    inline std::string GetLogFolder(ros::NodeHandle& nh)
    {
        const std::string task_type = ROSHelpers::GetParam<std::string>(nh, "task_type", "unknown");
        return ROSHelpers::GetParam<std::string>(nh, "log_folder", "/tmp/smmap/" + task_type + "/");
    }

    inline std::string GetDijkstrasStorageLocation(ros::NodeHandle& nh)
    {
        std::string dijkstras_file_path;
        switch (GetTaskType(nh))
        {
            case TaskType::WAFR:
                dijkstras_file_path = GetLogFolder(nh) + "../cloth_wafr.dijkstras_serialized";
                break;

            default:
                dijkstras_file_path = GetLogFolder(nh) + "../unknown_trial.dijkstras_serialized";
                break;
        }

        return ROSHelpers::GetParam<std::string>(nh, "dijkstras_file_path", dijkstras_file_path);
    }

    inline bool GetScreenshotsEnabled(ros::NodeHandle& nh)
    {
        const bool screenshots_enabled = ROSHelpers::GetParam(nh, "screenshots_enabled", false);
        // The viewer must be enabled for screen shots to be enabled
        assert(!screenshots_enabled || ROSHelpers::GetParam(nh, "start_bullet_viewer", true));
        return screenshots_enabled;
    }

    inline std::string GetScreenshotFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "screenshot_folder", GetLogFolder(nh) + "screenshots/");
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetTestGrippersPosesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "test_grippers_poses_topic", "test_grippers_poses");
    }

    inline std::string GetExecuteGrippersMovementTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "execute_grippers_movement_topic", "execute_grippers_movement");
    }

    inline std::string GetSimulatorFeedbackTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "simulator_feedback_topic", "simulator_feedback");
    }

    inline std::string GetCoverPointsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_cover_points_topic", "get_cover_points");
    }

    inline std::string GetMirrorLineTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_mirror_line_topic", "get_mirror_line");
    }

    inline std::string GetFreeSpaceGraphTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_free_space_graph_topic", "get_free_space_graph");
    }

    inline std::string GetGripperNamesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_gripper_names_topic", "get_gripper_names");
    }

    inline std::string GetGripperAttachedNodeIndicesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_gripper_attached_node_indices", "get_gripper_attached_node_indices");
    }

    inline std::string GetGripperPoseTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_gripper_pose_topic", "get_gripper_pose");
    }

    inline std::string GetObjectInitialConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_object_initial_configuration_topic", "get_object_initial_configuration");
    }

    inline std::string GetObjectCurrentConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_object_current_configuration_topic", "get_object_current_configuration");
    }

    inline std::string GetVisualizationMarkerTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "visualization_marker_topic", "visualization_marker");
    }

    inline std::string GetVisualizationMarkerArrayTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "visualization_marker_array_topic", "visualization_marker_array");
    }

    inline std::string GetConfidenceTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "confidence_topic", "confidence");
    }

    inline std::string GetConfidenceImageTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "confidence_image_topic", "confidence_image");
    }

    inline std::string GetGripperCollisionCheckTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "get_gripper_collision_check_topic", "get_gripper_collision_check");
    }

    inline std::string GetTerminateSimulationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "terminate_simulation_topic", "terminate_simulation");
    }
}

#endif // ROS_PARAMS_HPP
