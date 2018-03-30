#ifndef MANUALGRIPPERPATH_H
#define MANUALGRIPPERPATH_H

#include <functional>

#include "gripper_kinematic_object.h"

namespace smmap
{
    class ManualGripperPath
    {
        public:
            ManualGripperPath(GripperKinematicObject::Ptr gripper,
                    std::function<btTransform (u_int32_t /*current_step*/)> gripper_path_function);

            void advanceGripper();

        private:
            const GripperKinematicObject::Ptr gripper_;
            std::function<btTransform (u_int32_t /*current_step*/)> gripper_path_function_;
            u_int32_t current_step_;

    };

    btTransform gripperPath0(u_int32_t step_num);
    btTransform gripperPath1(u_int32_t step_num);
}

#endif // MANUALGRIPPERPATH_H
