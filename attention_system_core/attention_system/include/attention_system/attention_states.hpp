#ifndef ATTENTION_STATES_HPP_
#define ATTENTION_STATES_HPP_

#include "attention_system/attention_states.hpp"

namespace attention_system
{

enum class AttentionState {
  IDLE = 0,
  SETTING_BEHAVIOR_ACTION = 1,
  SETTING_BEHAVIOR_INPUT = 2,
  ACTING = 3
};

} // namespace attention_system

#endif // ATTENTION_STATES_HPP_