#ifndef ROS_COMMUNICATION_HELPERS_HPP
#define ROS_COMMUNICATION_HELPERS_HPP

#include <ros/ros.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/dijkstras.hpp>
#include <smmap_experiment_params/task_enums.h>
#include <smmap_msgs/messages.h>

#include "smmap/grippers.hpp"
#include "smmap/trajectory.hpp"

namespace smmap
{
    inline std::vector<GripperData> GetGrippersData(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("grippers_data" , "Getting grippers data");
        std::vector<GripperData> grippers_data;

        // Service client to get the names of each gripper
        ros::ServiceClient gripper_names_client =
            nh.serviceClient<smmap_msgs::GetGripperNames>(GetGripperNamesTopic(nh));
        gripper_names_client.waitForExistence();

        smmap_msgs::GetGripperNames names_srv_data;
        if (!gripper_names_client.call(names_srv_data))
        {
            ROS_FATAL_NAMED("grippers_data", "Unabled to retrieve gripper names.");
        }
        std::vector<std::string> gripper_names = names_srv_data.response.names;

        // Service client to get the attached nodes and transform for each gripper
        ros::ServiceClient gripper_node_indices_client =
            nh.serviceClient<smmap_msgs::GetGripperAttachedNodeIndices>(GetGripperAttachedNodeIndicesTopic(nh));
        gripper_node_indices_client.waitForExistence();

        grippers_data.reserve(gripper_names.size());
        for (size_t gripper_ind = 0; gripper_ind < gripper_names.size(); gripper_ind++)
        {
            smmap_msgs::GetGripperAttachedNodeIndices node_indices_srv_data;
            node_indices_srv_data.request.name = gripper_names[gripper_ind];

            if (gripper_node_indices_client.call(node_indices_srv_data))
            {
                grippers_data.push_back(GripperData(gripper_names[gripper_ind],
                                          VectorAnytypeToVectorLong(node_indices_srv_data.response.indices)));
            }
            else
            {
                ROS_ERROR_STREAM_NAMED("grippers_data", "Unable to retrieve node indices for gripper: " << gripper_names[gripper_ind]);
            }
        }

        return grippers_data;
    }

    inline ObjectPointSet GetObjectInitialConfiguration(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("object_initial_configuration" , "Getting object initial configuration");

        // Get the initial configuration of the object
        ros::ServiceClient object_initial_configuration_client =
            nh.serviceClient<smmap_msgs::GetPointSet>(GetObjectInitialConfigurationTopic(nh));

        object_initial_configuration_client.waitForExistence();

        smmap_msgs::GetPointSet srv_data;
        object_initial_configuration_client.call(srv_data);

        ROS_INFO_NAMED("object_initial_configuration" , "Number of points on object: %zu", srv_data.response.points.size());

        return EigenHelpersConversions::VectorGeometryPointToEigenMatrix3Xd(srv_data.response.points);
    }

    inline ObjectPointSet GetCoverPoints(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("cover_points_helper" , "Getting cover points");

        // Get the initial configuration of the object
        ros::ServiceClient cover_points_client =
            nh.serviceClient<smmap_msgs::GetPointSet>(GetCoverPointsTopic(nh));

        cover_points_client.waitForExistence();

        smmap_msgs::GetPointSet srv_data;
        cover_points_client.call(srv_data);
        ObjectPointSet cover_points =
            EigenHelpersConversions::VectorGeometryPointToEigenMatrix3Xd(srv_data.response.points);

        ROS_INFO_NAMED("cover_points_helper" , "Number of cover points: %zu", srv_data.response.points.size());

        return cover_points;
    }

    // TODO: replace these out params with something else
    inline void GetFreeSpaceGraph(ros::NodeHandle& nh, arc_dijkstras::Graph<Eigen::Vector3d>& free_space_graph, std::vector<int64_t>& cover_ind_to_free_space_graph_ind)
    {
        ROS_INFO_NAMED("get_free_space_graph_helper", "Getting free space graph");

        // First we collect the data in serialzed form
        ros::ServiceClient free_space_graph_client =
            nh.serviceClient<smmap_msgs::GetFreeSpaceGraph>(GetFreeSpaceGraphTopic(nh));

        free_space_graph_client.waitForExistence();

        smmap_msgs::GetFreeSpaceGraph srv_data;
        free_space_graph_client.call(srv_data);

        // Next we deserialize the graph itself
        {
            // Define the graph value deserialization function
            const auto value_deserializer_fn = [] (const std::vector<uint8_t>& buffer, const uint64_t current)
            {
                uint64_t current_position = current;

                // Deserialze 3 floats, converting into doubles afterwards
                std::pair<float, uint64_t> x = arc_helpers::DeserializeFixedSizePOD<float>(buffer, current_position);
                current_position += x.second;
                std::pair<float, uint64_t> y = arc_helpers::DeserializeFixedSizePOD<float>(buffer, current_position);
                current_position += y.second;
                std::pair<float, uint64_t> z = arc_helpers::DeserializeFixedSizePOD<float>(buffer, current_position);
                current_position += z.second;

                const Eigen::Vector3d deserialized(x.first, y.first, z.first);

                // Figure out how many bytes were read
                const uint64_t bytes_read = current_position - current;
                return std::make_pair(deserialized, bytes_read);
            };

            uint64_t current_position = 0;
            current_position += free_space_graph.DeserializeSelf(srv_data.response.graph_data_buffer, current_position, value_deserializer_fn);
            assert(current_position == srv_data.response.graph_data_buffer.size());
            assert(free_space_graph.CheckGraphLinkage());
            ROS_INFO_STREAM_NAMED("ros_communications_helpers", "Recieved " << free_space_graph.GetNodesImmutable().size() << " graph nodes");
        }

        // Last we copy the map between cover point indices and graph indices
        cover_ind_to_free_space_graph_ind = srv_data.response.cover_point_ind_to_graph_ind;
    }
}

#endif // ROS_COMMUNICATION_HELPERS_HPP
