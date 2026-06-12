#ifndef ATTENTION_AUX_PERCEPTION_NODES__DETECTION_3D_PERCEPTION_HPP
#define ATTENTION_AUX_PERCEPTION_NODES__DETECTION_3D_PERCEPTION_HPP

#include "attention_aux_perception_nodes/Detection3dProjectorNoDepht.hpp"

namespace attention_aux_perception_nodes
{

class Detection3dProjector : public Detection3dProjectorNoDepht
{

public:
  Detection3dProjector();
  virtual ~Detection3dProjector() = default;
};

}  // namespace attention_aux_perception_nodes

#endif  // ATTENTION_AUX_PERCEPTION_NODES__DETECTION_3D_PERCEPTION_HPP
