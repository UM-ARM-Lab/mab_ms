#ifndef SMMAP_MESSAGES_H
#define SMMAP_MESSAGES_H

// This pragma is here because the ROS message generator has an extra ';' on one
// line of code, and we can't push this off to be a system include
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "smmap_msgs/ConfidenceStamped.h"
#include "smmap_msgs/GetGripperAttachedNodeIndices.h"
#include "smmap_msgs/GetGripperPose.h"
#include "smmap_msgs/GetGripperCollisionReport.h"
#include "smmap_msgs/SimulatorFeedback.h"
#include "smmap_msgs/PointCloud.h"
#include "smmap_msgs/TestGrippersPosesAction.h"
#include "smmap_msgs/ExecuteGripperMovement.h"
// This pragma is here because the service call has an empty request
// (or response) message thus the allocator that it is passed never gets used
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "smmap_msgs/GetGripperNames.h"
#include "smmap_msgs/GetPointSet.h"
#include "smmap_msgs/GetMirrorLine.h"
#include "smmap_msgs/GetFreeSpaceGraph.h"
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

#endif // SMMAP_MESSAGES_H
