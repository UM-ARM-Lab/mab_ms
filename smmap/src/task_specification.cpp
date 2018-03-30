#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/log.hpp>

#include "smmap/task_specification.h"
#include "smmap/task_specification_implementions.hpp"

using namespace smmap;

////////////////////////////////////////////////////////////////////////////////
// Static helper functions - could be private given how they are
// used but making public as they are static - probably should be moved out of class
////////////////////////////////////////////////////////////////////////////////

double TaskSpecification::CalculateErrorWithTheshold(
        const ObjectPointSet& target_points,
        const ObjectPointSet& deformable_object,
        const double minimum_threshold)
{
    Eigen::VectorXd error(target_points.cols());

    // for every cover point, find the nearest deformable object point
    #pragma omp parallel for
    for (ssize_t target_ind = 0; target_ind < target_points.cols(); target_ind++)
    {
        const Eigen::Vector3d& target_point = target_points.col(target_ind);

        // find the closest deformable object point
        double min_dist_squared = std::numeric_limits<double>::infinity();
        for (ssize_t deformable_ind = 0; deformable_ind < deformable_object.cols(); deformable_ind++)
        {
            const Eigen::Vector3d& deformable_point = deformable_object.col(deformable_ind);
            const double new_dist_squared = (target_point - deformable_point).squaredNorm();
            min_dist_squared = std::min(new_dist_squared, min_dist_squared);
        }

        if (std::sqrt(min_dist_squared) >= minimum_threshold)
        {
            error(target_ind) = std::sqrt(min_dist_squared);
        }
        else
        {
            error(target_ind) = 0;
        }
    }

    return error.sum();
}

ObjectDeltaAndWeight TaskSpecification::CalculateObjectErrorCorrectionDeltaWithThreshold(
        const ObjectPointSet& target_points,
        const ObjectPointSet& deformable_object,
        const double minimum_threshold)
{
    stopwatch(RESET);

    const ssize_t num_nodes = deformable_object.cols();
    ObjectDeltaAndWeight desired_object_delta(num_nodes * 3);

    // for every target point, find the nearest deformable object point
    for (ssize_t target_ind = 0; target_ind < target_points.cols(); target_ind++)
    {
        const Eigen::Vector3d& target_point = target_points.col(target_ind);

        // find the closest deformable object point
        ssize_t min_ind = -1;
        double min_dist_squared = std::numeric_limits<double>::infinity();
        for (ssize_t deformable_ind = 0; deformable_ind < num_nodes; deformable_ind++)
        {
            const Eigen::Vector3d& deformable_point = deformable_object.col(deformable_ind);
            const double new_dist_squared = (target_point - deformable_point).squaredNorm();
            if (new_dist_squared < min_dist_squared)
            {
                min_dist_squared = new_dist_squared;
                min_ind = deformable_ind;
            }
        }

        const double min_dist = std::sqrt(min_dist_squared);
        if (min_dist > minimum_threshold)
        {
            desired_object_delta.delta.segment<3>(min_ind * 3) =
                    desired_object_delta.delta.segment<3>(min_ind * 3)
                    + (target_point - deformable_object.col(min_ind));

            const double weight = std::max(desired_object_delta.weight(min_ind * 3), min_dist);
            desired_object_delta.weight(min_ind * 3) = weight;
            desired_object_delta.weight(min_ind * 3 + 1) = weight;
            desired_object_delta.weight(min_ind * 3 + 2) = weight;
        }
    }

    ROS_INFO_STREAM_NAMED("target_point_task", "Found best delta in " << stopwatch(READ) << " seconds");

    return desired_object_delta;
}

////////////////////////////////////////////////////////////////////
// Constructor to initialize objects that all TaskSpecifications share
////////////////////////////////////////////////////////////////////

TaskSpecification::TaskSpecification(ros::NodeHandle& nh, const DeformableType deformable_type, const TaskType task_type)
    : TaskSpecification(nh, Visualizer(nh), deformable_type, task_type)
{}

TaskSpecification::TaskSpecification(ros::NodeHandle& nh, Visualizer vis, const DeformableType deformable_type, const TaskType task_type)
    : first_step_calculated_(false)
    , deformable_type_(deformable_type)
    , task_type_(task_type)
    , nh_(nh)
    , vis_(vis)
    , object_initial_node_distance_(CalculateDistanceMatrix(GetObjectInitialConfiguration(nh)))
    , num_nodes_(object_initial_node_distance_.cols())
{}

////////////////////////////////////////////////////////////////////////////////
// Static builder function
////////////////////////////////////////////////////////////////////////////////

TaskSpecification::Ptr TaskSpecification::MakeTaskSpecification(
        ros::NodeHandle& nh)
{
    TaskType task_type = GetTaskType(nh);
    DeformableType deformable_type = GetDeformableType(nh);

    if (deformable_type == DeformableType::ROPE && task_type == TaskType::CYLINDER_COVERAGE)
    {
        return std::make_shared<RopeCylinderCoverage>(nh);
    }
    else if (deformable_type == DeformableType::CLOTH && task_type == TaskType::TABLE_COVERAGE)
    {
        return std::make_shared<ClothTableCoverage>(nh);
    }
    else if (deformable_type == DeformableType::CLOTH && task_type == TaskType::CYLINDER_COVERAGE)
    {
        return std::make_shared<ClothCylinderCoverage>(nh);
    }
    else if (deformable_type == DeformableType::CLOTH && task_type == TaskType::COLAB_FOLDING)
    {
        return std::make_shared<ClothColabFolding>(nh);
    }
    else if (deformable_type == DeformableType::CLOTH && task_type == TaskType::WAFR)
    {
        return std::make_shared<ClothWAFR>(nh);
    }
    else
    {
        throw_arc_exception(std::invalid_argument, "Invalid task and deformable pair in MakeTaskSpecification(), this should not be possible");
        return nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Virtual function wrappers
////////////////////////////////////////////////////////////////////////////////

double TaskSpecification::defaultDeformability() const
{
    return deformability_impl();
}

double TaskSpecification::collisionScalingFactor() const
{
    return collisionScalingFactor_impl();
}

double TaskSpecification::stretchingScalingThreshold() const
{
    return stretchingScalingThreshold_impl();
}

double TaskSpecification::maxTime() const
{
    return maxTime_impl();
}

void TaskSpecification::visualizeDeformableObject(
        Visualizer& vis,
        const std::string& marker_name,
        const ObjectPointSet& object_configuration,
        const std_msgs::ColorRGBA& color) const
{
    visualizeDeformableObject_impl(vis, marker_name, object_configuration, color);
}

void TaskSpecification::visualizeDeformableObject(
        Visualizer& vis,
        const std::string& marker_name,
        const ObjectPointSet& object_configuration,
        const std::vector<std_msgs::ColorRGBA>& colors) const
{
    visualizeDeformableObject_impl(vis, marker_name, object_configuration, colors);
}

double TaskSpecification::calculateError(
        const ObjectPointSet& object_configuration) const
{
    return calculateError_impl(object_configuration);
}

ObjectDeltaAndWeight TaskSpecification::calculateObjectErrorCorrectionDelta(
        const WorldState& world_state) const
{
    return calculateObjectErrorCorrectionDelta_impl(world_state);
}

/**
 * @brief TaskSpecification::calculateStretchingCorrectionDelta
 * @param object_configuration
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::calculateStretchingCorrectionDelta(
        const ObjectPointSet& object_configuration,
        bool visualize) const
{
    ObjectDeltaAndWeight stretching_correction (num_nodes_ * 3);

    const Eigen::MatrixXd node_squared_distance =
            CalculateSquaredDistanceMatrix(object_configuration);

    const double stretching_correction_threshold = stretchingScalingThreshold();

    EigenHelpers::VectorVector3d start_points;
    EigenHelpers::VectorVector3d end_points;

    for (ssize_t first_node = 0; first_node < num_nodes_; first_node++)
    {
        for (ssize_t second_node = first_node + 1; second_node < num_nodes_; second_node++)
        {
            const double max_distance = stretching_correction_threshold + object_initial_node_distance_(first_node, second_node);
            if (node_squared_distance(first_node, second_node) > max_distance * max_distance)
            {
                const double node_distance_delta = std::sqrt(node_squared_distance(first_node, second_node)) - object_initial_node_distance_(first_node, second_node);
                assert(node_distance_delta > stretching_correction_threshold);
                // The correction vector points from the first node to the second node,
                // and is half the length of the "extra" distance
                const Eigen::Vector3d correction_vector = 0.5 * node_distance_delta
                        * (object_configuration.block<3, 1>(0, second_node)
                            - object_configuration.block<3, 1>(0, first_node));

                stretching_correction.delta.segment<3>(3 * first_node) += correction_vector;
                stretching_correction.delta.segment<3>(3 * second_node) -= correction_vector;

                // Set the weight to be the stretch distance of the worst offender
                const double first_node_max_stretch = std::max(stretching_correction.weight(3 * first_node), node_distance_delta);
                stretching_correction.weight(3 * first_node) = first_node_max_stretch;
                stretching_correction.weight(3 * first_node + 1) = first_node_max_stretch;
                stretching_correction.weight(3 * first_node + 2) = first_node_max_stretch;

                // Set the weight to be the stretch distance of the worst offender
                const double second_node_max_stretch = std::max(stretching_correction.weight(3 * second_node), node_distance_delta);
                stretching_correction.weight(3 * second_node) = second_node_max_stretch;
                stretching_correction.weight(3 * second_node + 1) = second_node_max_stretch;
                stretching_correction.weight(3 * second_node + 2) = second_node_max_stretch;

                if (visualize)
                {
                    start_points.push_back(object_configuration.block<3, 1>(0, first_node));
                    end_points.push_back(object_configuration.block<3, 1>(0, first_node) + correction_vector);

                    start_points.push_back(object_configuration.block<3, 1>(0, second_node));
                    end_points.push_back(object_configuration.block<3, 1>(0, first_node) - correction_vector);
                }
            }
        }
    }

    if (visualize && start_points.size() > 0)
    {
        vis_.visualizeLines("stretching_lines", start_points, end_points, Visualizer::Blue());
    }

    return stretching_correction;
}

/**
 * @brief TaskSpecification::calculateStretchingCorrectionDelta
 * @param world_state
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::calculateStretchingCorrectionDelta(
        const WorldState& world_state,
        bool visualize) const
{
    return calculateStretchingCorrectionDelta(world_state.object_configuration_, visualize);
}

/**
 * @brief TaskSpecification::calculateStretchingError
 * @param object_configuration
 * @return
 */
double TaskSpecification::calculateStretchingError(
        const ObjectPointSet& object_configuration) const
{
    const Eigen::MatrixXd node_squared_distance =
            CalculateSquaredDistanceMatrix(object_configuration);

    const double stretching_correction_threshold = stretchingScalingThreshold();

    ssize_t squared_error = 0;
    #pragma omp parallel for reduction(+ : squared_error)
    for (ssize_t first_node = 0; first_node < num_nodes_; first_node++)
    {
        // A node is never overstretched relative to itself, so we can start at the next node
        int node_overstretches = 0;
        for (ssize_t second_node = 0; second_node < num_nodes_; second_node++)
        {
            if (first_node != second_node)
            {
                const double max_distance = stretching_correction_threshold + object_initial_node_distance_(first_node, second_node);
                if (node_squared_distance(first_node, second_node) > (max_distance * max_distance))
                {
                    node_overstretches++;
                }
            }
        }
        squared_error += node_overstretches * node_overstretches;
    }

    return std::sqrt((double)squared_error) / (double)(num_nodes_ * num_nodes_);
}

/**
 * @brief TaskSpecification::calculateStretchingError
 * @param world_state
 * @return
 */
double TaskSpecification::calculateStretchingError(
        const WorldState& world_state) const
{
    return calculateStretchingError(world_state.object_configuration_);
}

/**
 * @brief TaskSpecification::combineErrorCorrectionAndStretchingCorrection
 * @param error_correction
 * @param stretching_correction
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::combineErrorCorrectionAndStretchingCorrection(
        const ObjectDeltaAndWeight& error_correction,
        const ObjectDeltaAndWeight& stretching_correction) const
{
    ObjectDeltaAndWeight combined(num_nodes_ * 3);

    for (ssize_t node_ind = 0; node_ind < num_nodes_ * 3; node_ind += 3)
    {
        const Eigen::Vector3d error_correction_perpendicular =
                EigenHelpers::VectorRejection(stretching_correction.delta.segment<3>(node_ind),
                                               error_correction.delta.segment<3>(node_ind) );

        combined.delta.segment<3>(node_ind) = stretching_correction.delta.segment<3>(node_ind) + error_correction_perpendicular;
        combined.weight.segment<3>(node_ind) = stretching_correction.weight.segment<3>(node_ind) + error_correction.weight.segment<3>(node_ind);
    }

    // Normalize the weights for later use
    const double combined_normalizer = combined.weight.maxCoeff();
    if (combined_normalizer > 0)
    {
        combined.weight /= combined_normalizer;
    }

    return combined;
}


ObjectDeltaAndWeight TaskSpecification::calculateDesiredDirection(const WorldState& world_state)
{
    if (sim_time_last_time_first_step_calced_ != world_state.sim_time_)
    {
        first_step_calculated_.store(false);
    }

    if (first_step_calculated_.load())
    {
        return first_step_desired_motion_;
    }
    else
    {
        std::lock_guard<std::mutex> lock(first_step_mtx_);
        if (first_step_calculated_.load())
        {
            return first_step_desired_motion_;
        }
        else
        {
            ROS_INFO_NAMED("task", "Determining desired direction");
            first_step_error_correction_ = calculateObjectErrorCorrectionDelta(world_state);

            first_step_stretching_correction_ = calculateStretchingCorrectionDelta(world_state, false);

            first_step_desired_motion_ = combineErrorCorrectionAndStretchingCorrection(
                        first_step_error_correction_, first_step_stretching_correction_);

            sim_time_last_time_first_step_calced_ = world_state.sim_time_;
            first_step_calculated_.store(true);
            return first_step_desired_motion_;
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
// Coverage Task
////////////////////////////////////////////////////////////////////////////////

CoverageTask::CoverageTask(ros::NodeHandle& nh, const DeformableType deformable_type, const TaskType task_type)
    : TaskSpecification(nh, deformable_type, task_type)
    , cover_points_(GetCoverPoints(nh))
    , num_cover_points_(cover_points_.cols())
{}

double CoverageTask::getErrorThreshold() const
{
    return getErrorThreshold_impl();
}

double CoverageTask::calculateError_impl(const ObjectPointSet& current_configuration) const
{
    return CalculateErrorWithTheshold(cover_points_, current_configuration, getErrorThreshold());
}

////////////////////////////////////////////////////////////////////////////////
// Direct Coverage Task
////////////////////////////////////////////////////////////////////////////////

DirectCoverageTask::DirectCoverageTask(ros::NodeHandle& nh, const DeformableType deformable_type, const TaskType task_type)
    : CoverageTask(nh, deformable_type, task_type)
{}

ObjectDeltaAndWeight DirectCoverageTask::calculateObjectErrorCorrectionDelta_impl(const WorldState& world_state) const
{
    ROS_INFO_NAMED("direct_coverage_task" , "Finding 'best' object delta");
    return CalculateObjectErrorCorrectionDeltaWithThreshold(
                cover_points_, world_state.object_configuration_, getErrorThreshold());
}

////////////////////////////////////////////////////////////////////////////////
// Dijkstras Coverage Task
////////////////////////////////////////////////////////////////////////////////

DijkstrasCoverageTask::DijkstrasCoverageTask(ros::NodeHandle& nh, const DeformableType deformable_type, const TaskType task_type)
    : CoverageTask(nh, deformable_type, task_type)
    , free_space_grid_(GetWorldXMin(nh), GetWorldXStep(nh), GetWorldXNumSteps(nh),
                       GetWorldYMin(nh), GetWorldYStep(nh), GetWorldYNumSteps(nh),
                       GetWorldZMin(nh), GetWorldZStep(nh), GetWorldZNumSteps(nh))
{
    GetFreeSpaceGraph(nh, free_space_graph_, cover_ind_to_free_space_graph_ind_);
    assert(cover_ind_to_free_space_graph_ind_.size() == (size_t)num_cover_points_);

    const bool need_to_run_dijkstras = !loadDijkstrasResults();

    if (need_to_run_dijkstras)
    {
        ROS_INFO_STREAM_NAMED("coverage_task", "Generating " << num_cover_points_ << " Dijkstra's solutions");
        stopwatch(RESET);
        dijkstras_results_.resize((size_t)num_cover_points_);
        #pragma omp parallel for schedule(guided)
        for (size_t cover_ind = 0; cover_ind < (size_t)num_cover_points_; cover_ind++)
        {
            const int64_t free_space_graph_ind = cover_ind_to_free_space_graph_ind_[cover_ind];
            const auto result = arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector3d>::PerformDijkstrasAlgorithm(free_space_graph_, free_space_graph_ind);
            dijkstras_results_[cover_ind] = result.second;
        }
        ROS_INFO_NAMED("coverage_task", "Writing solutions to file");

        saveDijkstrasResults();
        ROS_INFO_STREAM_NAMED("coverage_task", "Found solutions in " << stopwatch(READ) << " seconds");
    }
}


bool DijkstrasCoverageTask::saveDijkstrasResults()
{
    try
    {
        std::vector<uint8_t> buffer;
        // First serialize the graph that created the results
        ROS_INFO_NAMED("coverage_task", "Serializing the data");
        free_space_graph_.SerializeSelf(buffer, &EigenHelpers::Serialize<Eigen::Vector3d>);

        // Next serialize the results themselves
        const auto first_serializer = [] (const std::vector<int64_t>& vec_to_serialize, std::vector<uint8_t>& buffer) { return arc_helpers::SerializeVector<int64_t>(vec_to_serialize, buffer, &arc_helpers::SerializeFixedSizePOD<int64_t>); };
        const auto second_serializer = [] (const std::vector<double>& vec_to_serialize, std::vector<uint8_t>& buffer) { return arc_helpers::SerializeVector<double>(vec_to_serialize, buffer, &arc_helpers::SerializeFixedSizePOD<double>); };
        const auto pair_serializer = [&first_serializer, &second_serializer] (const std::pair<std::vector<int64_t>, std::vector<double>>& pair_to_serialize, std::vector<uint8_t>& buffer) { return arc_helpers::SerializePair<std::vector<int64_t>, std::vector<double>>(pair_to_serialize, buffer, first_serializer, second_serializer); };
        arc_helpers::SerializeVector<std::pair<std::vector<int64_t>, std::vector<double>>>(dijkstras_results_, buffer, pair_serializer);

        // Compress and save to file
        ROS_INFO_NAMED("coverage_task", "Compressing for storage");
        const std::vector<uint8_t> compressed_serialized_data = ZlibHelpers::CompressBytes(buffer);
        ROS_INFO_NAMED("coverage_task", "Saving Dijkstras results to file");
        const std::string dijkstras_file_path = GetDijkstrasStorageLocation(nh_);

        // If it hasn't been opened, assume that it is because the
        // directory doesn't exist.
        boost::filesystem::path p(dijkstras_file_path);
        boost::filesystem::path dir = p.parent_path();
        if (!boost::filesystem::is_directory(dir))
        {
            std::cerr << "\x1b[33;1m" << dir << " does not exist! Creating ... ";

            // NOTE: create_directories should be able to return true in this case
            // however due to a bug related to a trailing '/' this is not currently
            // the case in my version of boost
            // https://svn.boost.org/trac/boost/ticket/7258
            boost::filesystem::create_directories(dir);
            if (boost::filesystem::is_directory(dir))
//            if (boost::filesystem::create_directories(p))
            {
                std::cerr << "Succeeded!\x1b[37m\n";
            }
            else
            {
                std::cerr << "\x1b[31;1mFailed!\x1b[37m\n";
            }
        }

        std::ofstream output_file(dijkstras_file_path, std::ios::out | std::ios::binary);
        uint64_t serialized_size = compressed_serialized_data.size();
        output_file.write(reinterpret_cast<const char*>(compressed_serialized_data.data()), (std::streamsize)serialized_size);
        output_file.close();
        return true;
    }
    catch (...)
    {
        ROS_ERROR_NAMED("coverage_task", "Saving Dijkstras results to file failed");
        return false;
    }
}

bool DijkstrasCoverageTask::loadDijkstrasResults()
{
    try
    {
        ROS_INFO_NAMED("coverage_task", "Checking if Dijkstra's solution already exists");
        const std::string dijkstras_file_path = GetDijkstrasStorageLocation(nh_);
        stopwatch(RESET);
        std::ifstream prev_dijkstras_result(dijkstras_file_path, std::ios::binary | std::ios::in | std::ios::ate);
        if (!prev_dijkstras_result.is_open())
        {
            throw_arc_exception(std::runtime_error, "Couldn't open file");
        }

        ROS_INFO_NAMED("coverage_task", "Reading contents of file");
        std::streamsize size = prev_dijkstras_result.tellg();
        prev_dijkstras_result.seekg(0, std::ios::beg);
        std::vector<uint8_t> file_buffer((size_t)size);
        if (!(prev_dijkstras_result.read(reinterpret_cast<char*>(file_buffer.data()), size)))
        {
            throw_arc_exception(std::runtime_error, "Unable to read entire contents of file");
        }
        const std::vector<uint8_t> decompressed_dijkstras_results = ZlibHelpers::DecompressBytes(file_buffer);

        // First check that the graph we have matches the graph that is stored
        std::vector<uint8_t> temp_buffer;
        const uint64_t serialzed_graph_size = free_space_graph_.SerializeSelf(temp_buffer, &EigenHelpers::Serialize<Eigen::Vector3d>);
        const auto mismatch_results = std::mismatch(temp_buffer.begin(), temp_buffer.end(), decompressed_dijkstras_results.begin());
        if (mismatch_results.first != temp_buffer.end())
        {
            throw_arc_exception(std::runtime_error, "Mismatch in serialzed graphs, need to regenerate Dijkstras results");
        }

        // Next deserialze the Dijkstras results
        const auto first_deserializer = [] (const std::vector<uint8_t>& buffer, const uint64_t current) { return arc_helpers::DeserializeVector<int64_t>(buffer, current, &arc_helpers::DeserializeFixedSizePOD<int64_t>); };
        const auto second_deserializer = [] (const std::vector<uint8_t>& buffer, const uint64_t current) { return arc_helpers::DeserializeVector<double>(buffer, current, &arc_helpers::DeserializeFixedSizePOD<double>); };
        const auto pair_deserializer = [&first_deserializer, &second_deserializer] (const std::vector<uint8_t>& buffer, const uint64_t current) { return arc_helpers::DeserializePair<std::vector<int64_t>, std::vector<double>>(buffer, current, first_deserializer, second_deserializer); };

        uint64_t current_position = serialzed_graph_size;
        const auto deserialized_result = arc_helpers::DeserializeVector<std::pair<std::vector<int64_t>, std::vector<double>>>(decompressed_dijkstras_results, current_position, pair_deserializer);
        dijkstras_results_ = deserialized_result.first;
        current_position += deserialized_result.second;
        if (current_position != decompressed_dijkstras_results.size())
        {
            throw_arc_exception(std::runtime_error, "Invalid data size found");
        }

        ROS_INFO_STREAM_NAMED("coverage_task", "Read solutions in " << stopwatch(READ) << " seconds");
        return true;
    }

    catch (...)
    {
        ROS_ERROR_NAMED("coverage_task", "Loading Dijkstras results from file failed");
        return false;
    }
}

ObjectDeltaAndWeight DijkstrasCoverageTask::calculateObjectErrorCorrectionDelta_Dijkstras(
        const ObjectPointSet& object_configuration, const double minimum_threshold) const
{
    ROS_INFO_NAMED("coverage_task" , "Finding 'best' object delta");

    ObjectDeltaAndWeight desired_object_delta(num_nodes_ * 3);

    stopwatch(RESET);
    // For every cover point, find the nearest deformable object point
    for (ssize_t cover_ind = 0; cover_ind < num_cover_points_; cover_ind++)
    {
        const Eigen::Vector3d& cover_point = cover_points_.col(cover_ind);

        // Find the closest deformable object point
        double min_dist = std::numeric_limits<double>::infinity();
        ssize_t min_ind = -1;
        ssize_t target_point_ind_in_graph = -1;

        for (ssize_t deformable_ind = 0; deformable_ind < num_nodes_; deformable_ind++)
        {
            const Eigen::Vector3d& deformable_point = object_configuration.col(deformable_ind);
            const double straight_line_distance_squared = (cover_point - deformable_point).squaredNorm();

            int64_t target_ind;
            double graph_dist;

            // If we are more than a grid cell away from the cover point, then lookup our position in the rest of the grid
            if (straight_line_distance_squared > std::pow(free_space_grid_.minStepDimension(), 2))
            {
                const ssize_t deformable_point_ind_in_graph = free_space_grid_.worldPosToGridIndexClamped(deformable_point);
                target_ind = dijkstras_results_[(size_t)cover_ind].first[(size_t)deformable_point_ind_in_graph];
                graph_dist = dijkstras_results_[(size_t)cover_ind].second[(size_t)deformable_point_ind_in_graph];
            }
            // Otherwise, use the cover point directly
            else
            {
                target_ind = cover_ind_to_free_space_graph_ind_[(size_t)cover_ind];
                graph_dist = std::sqrt(straight_line_distance_squared);
            }

            // If we've found something closer, update our record of the closest point on the deformable object
            if (graph_dist < min_dist)
            {
                min_dist = graph_dist;
                min_ind = deformable_ind;
                target_point_ind_in_graph = target_ind;
            }
        }

        // If we are at least some minimum threshold away, use this
        // cover point as a "pull" on the nearest deformable point
        if (min_dist > minimum_threshold)
        {
            const Eigen::Vector3d& closest_point = object_configuration.col(min_ind);
            const Eigen::Vector3d& target_point = free_space_graph_.GetNodeImmutable(target_point_ind_in_graph).GetValueImmutable();

            desired_object_delta.delta.segment<3>(min_ind * 3) =
                    desired_object_delta.delta.segment<3>(min_ind * 3)
                    + (target_point - closest_point);

            const double weight = std::max(desired_object_delta.weight(min_ind * 3), min_dist);
            desired_object_delta.weight(min_ind * 3) = weight;
            desired_object_delta.weight(min_ind * 3 + 1) = weight;
            desired_object_delta.weight(min_ind * 3 + 2) = weight;
        }
    }
    ROS_INFO_STREAM_NAMED("coverage_task", "Found best delta in " << stopwatch(READ) << " seconds");

    return desired_object_delta;
}

ObjectDeltaAndWeight DijkstrasCoverageTask::calculateObjectErrorCorrectionDelta_impl(const WorldState& world_state) const
{
    ROS_INFO_NAMED("dijkstras_coverage_task" , "Finding 'best' object delta");
    return calculateObjectErrorCorrectionDelta_Dijkstras(world_state.object_configuration_, getErrorThreshold());
}
