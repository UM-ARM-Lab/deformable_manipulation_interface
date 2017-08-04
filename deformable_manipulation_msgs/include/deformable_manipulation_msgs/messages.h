#ifndef SMMAP_MESSAGES_H
#define SMMAP_MESSAGES_H

// This pragma is here because the ROS message generator has an extra ';' on one
// line of code, and we can't push this off to be a system include
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "deformable_manipulation_msgs/ConfidenceStamped.h"
#include "deformable_manipulation_msgs/GetGripperAttachedNodeIndices.h"
#include "deformable_manipulation_msgs/GetGripperPose.h"
#include "deformable_manipulation_msgs/GetGripperCollisionReport.h"
#include "deformable_manipulation_msgs/GetGripperStretchingVectorInfo.h"
#include "deformable_manipulation_msgs/SimulatorFeedback.h"
#include "deformable_manipulation_msgs/PointCloud.h"
#include "deformable_manipulation_msgs/TestGrippersPosesAction.h"
#include "deformable_manipulation_msgs/ExecuteGripperMovement.h"
// This pragma is here because the service call has an empty request
// (or response) message thus the allocator that it is passed never gets used
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "deformable_manipulation_msgs/GetGripperNames.h"
#include "deformable_manipulation_msgs/GetPointSet.h"
#include "deformable_manipulation_msgs/GetMirrorLine.h"
#include "deformable_manipulation_msgs/GetFreeSpaceGraph.h"
#include "deformable_manipulation_msgs/GetSignedDistanceField.h"
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

#endif // SMMAP_MESSAGES_H
