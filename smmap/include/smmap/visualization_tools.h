#ifndef VISUALIZATION_TOOLS_H
#define VISUALIZATION_TOOLS_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <smmap_experiment_params/ros_params.hpp>

#include "smmap/trajectory.hpp"

namespace smmap
{
    class Visualizer
    {
        public:
            static void InitializeStandardColors();
            static std_msgs::ColorRGBA Red();
            static std_msgs::ColorRGBA Green();
            static std_msgs::ColorRGBA Blue();
            static std_msgs::ColorRGBA Black();
            static std_msgs::ColorRGBA Magenta();

        public:
            Visualizer(ros::NodeHandle& nh);

            void visualizePoints(
                    const std::string& marker_name,
                    EigenHelpers::VectorVector3d points,
                    const std_msgs::ColorRGBA& color) const;

            void visualizePoints(
                    const std::string& marker_name,
                    const EigenHelpers::VectorVector3d points,
                    const std::vector<std_msgs::ColorRGBA>& colors) const;

            void visualizeRope(
                    const std::string& marker_name,
                    const ObjectPointSet& rope,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeRope(
                    const std::string& marker_name,
                    const ObjectPointSet& rope,
                    const std::vector<std_msgs::ColorRGBA>& colors) const;

            void visualizeCloth(
                    const std::string& marker_name,
                    const ObjectPointSet& cloth,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeCloth(
                    const std::string& marker_name,
                    const ObjectPointSet& cloth,
                    const std::vector<std_msgs::ColorRGBA>& colors) const;

            void visualizeGripper(
                    const std::string& marker_name,
                    const geometry_msgs::Pose& pose,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeObjectDelta(
                    const std::string& marker_name,
                    const ObjectPointSet& current,
                    const ObjectPointSet& desired,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeTranslation(
                    const std::string& marker_name,
                    const geometry_msgs::Point& start,
                    const geometry_msgs::Point& end,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeTranslation(
                    const std::string& marker_name,
                    const Eigen::Vector3d& start,
                    const Eigen::Vector3d& end,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeTranslation(
                    const std::string& marker_name,
                    const Eigen::Affine3d &start,
                    const Eigen::Affine3d &end,
                    const std_msgs::ColorRGBA& color) const;

            void visualizeLines(
                    const std::string& marker_name,
                    const EigenHelpers::VectorVector3d& start,
                    const EigenHelpers::VectorVector3d& end,
                    const std_msgs::ColorRGBA& color) const;

        private:
            mutable ros::Publisher visualization_marker_pub_;
            mutable ros::Publisher visualization_marker_array_pub_;

            static bool standard_colors_initialized_;
            static std_msgs::ColorRGBA red_;
            static std_msgs::ColorRGBA green_;
            static std_msgs::ColorRGBA blue_;
            static std_msgs::ColorRGBA black_;
            static std_msgs::ColorRGBA magenta_;
    };
}

#endif // VISUALIZATION_TOOLS_H
