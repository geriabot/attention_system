#include "behaviortree_cpp/bt_factory.h"

#include "attention_system_behaviors/ActivateTwisting.hpp"
#include "attention_system_behaviors/ChangeVisualDetectionClass.hpp"
#include "attention_system_behaviors/ClearVisualDetectionClass.hpp"
#include "attention_system_behaviors/DetectionExists.hpp"
#include "attention_system_behaviors/DetectionNearSightCenter.hpp"
#include "attention_system_behaviors/DetectionNearSightLimit.hpp"
#include "attention_system_behaviors/GetDetections2DInformation.hpp"
#include "attention_system_behaviors/GetTFCompactTreeInformation.hpp"
#include "attention_system_behaviors/IsRobotOutOfBounds.hpp"
#include "attention_system_behaviors/JointTrackingKinematicallyFeasible.hpp"
#include "attention_system_behaviors/N3dDetectionsSameClassExist.hpp"
#include "attention_system_behaviors/No3dDetectionsSameClassExist.hpp"
#include "attention_system_behaviors/PublishRobotOutOfBoundsReferenceTF.hpp"
#include "attention_system_behaviors/PublishTwist.hpp"
#include "attention_system_behaviors/RequestBehaviorInputIntelligence.hpp"
#include "attention_system_behaviors/RequestSingleDetectionTrackPoint.hpp"
#include "attention_system_behaviors/StartTrackTF.hpp"
#include "attention_system_behaviors/TFExists.hpp"
#include "attention_system_behaviors/TrackedDetectionsClustered.hpp"
#include "attention_system_behaviors/RequestNDetectionsSameClassTrackPoint.hpp"
#include "attention_system_behaviors/TriggerDeactivationService.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::ChangeVisualDetectionClass>("ChangeVisualDetectionClass");
  factory.registerNodeType<attention_system_behaviors::ClearVisualDetectionClass>("ClearVisualDetectionClass");
  factory.registerNodeType<attention_system_behaviors::DetectionExists>("DetectionExists");
  factory.registerNodeType<attention_system_behaviors::DetectionNearSightCenter>("DetectionNearSightCenter");
  factory.registerNodeType<attention_system_behaviors::DetectionNearSightLimit>("DetectionNearSightLimit");
  factory.registerNodeType<attention_system_behaviors::GetDetections2DInformation>("GetDetections2DInformation");
  factory.registerNodeType<attention_system_behaviors::GetTFCompactTreeInformation>("GetTFCompactTreeInformation");
  factory.registerNodeType<attention_system_behaviors::IsRobotOutOfBounds>("IsRobotOutOfBounds");
  factory.registerNodeType<attention_system_behaviors::JointTrackingKinematicallyFeasible>("JointTrackingKinematicallyFeasible");
  factory.registerNodeType<attention_system_behaviors::N3dDetectionsSameClassExist>("N3dDetectionsSameClassExist");
  factory.registerNodeType<attention_system_behaviors::No3dDetectionsSameClassExist>("No3dDetectionsSameClassExist");
  factory.registerNodeType<attention_system_behaviors::PublishRobotOutOfBoundsReferenceTF>("PublishRobotOutOfBoundsReferenceTF");
  factory.registerNodeType<attention_system_behaviors::RequestBehaviorInputIntelligence>("RequestBehaviorInputIntelligence");
  factory.registerNodeType<attention_system_behaviors::RequestSingleDetectionTrackPoint>("RequestSingleDetectionTrackPoint");
  factory.registerNodeType<attention_system_behaviors::StartTrackTF>("StartTrackTF");
  factory.registerNodeType<attention_system_behaviors::TFExists>("TFExists");
  factory.registerNodeType<attention_system_behaviors::ActivateTwisting>("ActivateTwisting");
  factory.registerNodeType<attention_system_behaviors::PublishTwist>("PublishTwist");
  factory.registerNodeType<attention_system_behaviors::RequestNDetectionsSameClassTrackPoint>("RequestNDetectionsSameClassTrackPoint");
  factory.registerNodeType<attention_system_behaviors::TriggerDeactivationService>("TriggerDeactivationService");
  factory.registerNodeType<attention_system_behaviors::TrackedDetectionsClustered>("TrackedDetectionsClustered");
}
