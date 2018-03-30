#include "smmap/robot_interface.hpp"

#include <std_srvs/Empty.h>
#include <ros/callback_queue.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include "smmap/ros_communication_helpers.hpp"


using namespace smmap;

RobotInterface::RobotInterface(ros::NodeHandle& nh)
    : nh_(nh)
    , grippers_data_(GetGrippersData(nh_))
    , gripper_collision_checker_(nh_)
    , execute_gripper_movement_client_(nh_.serviceClient<smmap_msgs::ExecuteGripperMovement>(GetExecuteGrippersMovementTopic(nh_), true))
    , test_grippers_poses_client_(nh_, GetTestGrippersPosesTopic(nh_), false)
    , dt_(GetRobotControlPeriod(nh_))
    , max_gripper_velocity_(GetMaxGripperVelocity(nh_))
    // TODO: remove this hardcoded spin rate
    , spin_thread_(ROSHelpers::Spin, 1000)
{}

RobotInterface::~RobotInterface()
{
    ROS_INFO_NAMED("task", "Terminating");
    spin_thread_.join();
}

WorldState RobotInterface::start()
{
    ROS_INFO_NAMED("robot_bridge", "Waiting for the robot gripper action server to be available");
    execute_gripper_movement_client_.waitForExistence();
    test_grippers_poses_client_.waitForServer();

    ROS_INFO_NAMED("robot_bridge", "Kickstarting the planner with a no-op");
    return sendGrippersPoses_impl(noOpGripperMovement());
}

bool RobotInterface::ok() const
{
    return ros::ok();
}

void RobotInterface::shutdown()
{
    ros::ServiceClient shutdown_sim_client_ = nh_.serviceClient<std_srvs::Empty>(GetTerminateSimulationTopic(nh_));
    std_srvs::Empty empty;
    shutdown_sim_client_.call(empty);

    ros::shutdown();
}

const std::vector<GripperData>& RobotInterface::getGrippersData() const
{
    return grippers_data_;
}

const AllGrippersSinglePose RobotInterface::getGrippersPose()
{
    AllGrippersSinglePose grippers_pose(grippers_data_.size());

    for (size_t gripper_ind = 0; gripper_ind < grippers_data_.size(); gripper_ind++)
    {
        ros::ServiceClient gripper_pose_client =
            nh_.serviceClient<smmap_msgs::GetGripperPose>(GetGripperPoseTopic(nh_));
        gripper_pose_client.waitForExistence();

        smmap_msgs::GetGripperPose pose_srv_data;
        pose_srv_data.request.name = grippers_data_[gripper_ind].name;
        if (!gripper_pose_client.call(pose_srv_data))
        {
            ROS_FATAL_STREAM_NAMED("task", "Unabled to retrieve gripper pose: "
                                    << grippers_data_[gripper_ind].name);
        }

        grippers_pose[gripper_ind] =
                EigenHelpersConversions::GeometryPoseToEigenAffine3d(pose_srv_data.response.pose);
    }

    return grippers_pose;
}

WorldState RobotInterface::sendGrippersPoses(const AllGrippersSinglePose& grippers_poses)
{
    return sendGrippersPoses_impl(toRosGrippersPoses(grippers_poses));
}

bool RobotInterface::testGrippersPoses(const std::vector<AllGrippersSinglePose>& grippers_poses,
                       const TestGrippersPosesFeedbackCallbackFunctionType& feedback_callback)
{
    return testGrippersPoses_impl(toRosTestPosesGoal(grippers_poses), feedback_callback);
}

std::vector<CollisionData> RobotInterface::checkGripperCollision(const AllGrippersSinglePose& grippers_poses)
{
    return gripper_collision_checker_.gripperCollisionCheck(grippers_poses);
}


////////////////////////////////////////////////////////////////////
// ROS objects and helpers
////////////////////////////////////////////////////////////////////

smmap_msgs::ExecuteGripperMovementRequest RobotInterface::noOpGripperMovement()
{
    smmap_msgs::ExecuteGripperMovementRequest movement_request;
    movement_request.grippers_names = GetGripperNames(grippers_data_);
    movement_request.gripper_poses.resize(grippers_data_.size());

    ros::ServiceClient gripper_pose_client =
        nh_.serviceClient<smmap_msgs::GetGripperPose>(GetGripperPoseTopic(nh_));
    gripper_pose_client.waitForExistence();
    for (size_t gripper_ind = 0; gripper_ind < grippers_data_.size(); gripper_ind++)
    {
        smmap_msgs::GetGripperPose pose_srv_data;
        pose_srv_data.request.name = grippers_data_[gripper_ind].name;
        if (!gripper_pose_client.call(pose_srv_data))
        {
            ROS_FATAL_STREAM_NAMED("task", "Unabled to retrieve gripper pose: "
                                    << grippers_data_[gripper_ind].name);
        }

        movement_request.gripper_poses[gripper_ind] = pose_srv_data.response.pose;
    }
    return movement_request;
}

smmap_msgs::ExecuteGripperMovementRequest RobotInterface::toRosGrippersPoses(
        const AllGrippersSinglePose& grippers_pose) const
{
    smmap_msgs::ExecuteGripperMovementRequest movement_request;
    movement_request.grippers_names = GetGripperNames(grippers_data_);
    movement_request.gripper_poses = EigenHelpersConversions::VectorAffine3dToVectorGeometryPose(grippers_pose);
    return movement_request;
}

smmap_msgs::TestGrippersPosesGoal RobotInterface::toRosTestPosesGoal(
        const std::vector<AllGrippersSinglePose>& grippers_poses) const
{
    smmap_msgs::TestGrippersPosesGoal goal;
    goal.gripper_names = GetGripperNames(grippers_data_);

    goal.poses_to_test.resize(grippers_poses.size());
    for (size_t pose_ind = 0; pose_ind < grippers_poses.size(); pose_ind++)
    {
        goal.poses_to_test[pose_ind].pose =
                EigenHelpersConversions::VectorAffine3dToVectorGeometryPose(
                    grippers_poses[pose_ind]);
    }

    return goal;
}

WorldState RobotInterface::sendGrippersPoses_impl(
        const smmap_msgs::ExecuteGripperMovementRequest& movement)
{
    smmap_msgs::ExecuteGripperMovementResponse result;
    if (!execute_gripper_movement_client_.call(movement, result))
    {
        ROS_FATAL_NAMED("planner", "Sending a gripper movement to the robot failed");
    }
    return ConvertToEigenFeedback(result.sim_state);
}



void RobotInterface::internalTestPoseFeedbackCallback(const smmap_msgs::TestGrippersPosesActionFeedbackConstPtr& feedback, const TestGrippersPosesFeedbackCallbackFunctionType& feedback_callback)
{
    ROS_INFO_STREAM_NAMED("robot_interface", "Got feedback for test number " << feedback->feedback.test_id);
    feedback_callback(feedback->feedback.test_id, ConvertToEigenFeedback(feedback->feedback.sim_state));
    if (feedback_recieved_[feedback->feedback.test_id] == false)
    {
        feedback_recieved_[feedback->feedback.test_id] = true;
        feedback_counter_--;
    }
}


bool RobotInterface::testGrippersPoses_impl(
        const smmap_msgs::TestGrippersPosesGoal& goal,
        const TestGrippersPosesFeedbackCallbackFunctionType& feedback_callback)
{

    feedback_counter_ = goal.poses_to_test.size();
    feedback_recieved_.clear();
    feedback_recieved_.resize(goal.poses_to_test.size(), false);

    ros::Subscriber internal_feedback_sub = nh_.subscribe<smmap_msgs::TestGrippersPosesActionFeedback>(
                GetTestGrippersPosesTopic(nh_) + "/feedback", 1000, boost::bind(&RobotInterface::internalTestPoseFeedbackCallback, this, _1, feedback_callback));

    test_grippers_poses_client_.sendGoal(goal);

    const bool result = test_grippers_poses_client_.waitForResult();

    while (feedback_counter_ > 0)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(0.0001));
    }

    return result;
}
