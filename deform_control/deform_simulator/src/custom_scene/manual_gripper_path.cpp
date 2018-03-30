#include "custom_scene/manual_gripper_path.h"

using namespace smmap;

ManualGripperPath::ManualGripperPath(GripperKinematicObject::Ptr gripper,
        std::function<btTransform (u_int32_t /*current_step*/)> gripper_path_function)
    : gripper_(gripper)
    , gripper_path_function_(gripper_path_function)
    , current_step_(0)
{}

void ManualGripperPath::advanceGripper()
{
    btTransform next_step = gripper_path_function_(current_step_);

    // Apply the given translation in world frame
    btTransform trans = btTransform(btQuaternion(0, 0, 0, 1), next_step.getOrigin());
    // Apply the given rotation in gripper frame
    btTransform rot = btTransform(next_step.getRotation());

    gripper_->setWorldTransform(trans * gripper_->getWorldTransform() * rot);
    current_step_++;
}

btTransform smmap::gripperPath0(u_int32_t current_step)
{
    // start with a no-op transform
    btTransform transform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));

    // Move in (world) -x
    if (current_step < 100) //&& current_step % 10 == 0)
    {
        transform = btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.02f, 0, 0));
    }
    // rotate around (gripper) -y
    else if (current_step < 200) //&& current_step % 10 == 0)
    {
        transform = btTransform(btQuaternion(btVector3(0, 1, 0), -0.01f), btVector3(0, 0, 0));
    }
    // move in (world) -x
    else if (current_step < 300) //&& current_step % 10 == 0)
    {
        transform = btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.02f, 0, 0));
    }
    // rotate around (gripper) -y
    else if (current_step < 350) //&& current_step % 10 == 0)
    {
        transform = btTransform(btQuaternion(btVector3(0, 1, 0), -0.01f), btVector3(0, 0, 0));
    }

    return transform;
}

btTransform smmap::gripperPath1(u_int32_t current_step)
{
    // start with a no-op transform
    btTransform transform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));

    transform = gripperPath0(current_step);

    return transform;
}
