#ifndef ATTENTION_INTELLIGENCE_STATES_HPP_
#define ATTENTION_INTELLIGENCE_STATES_HPP_

#include "attention_system/intelligence/attention_intelligence_states.hpp"

namespace attention_system
{

enum class IntelligenceState {
  IDLE = 0,
  ASK_BEHAVIOR_ACTION = 1,
  ASK_BEHAVIOR_INPUTS = 2
};

} // namespace attention_system

#endif // ATTENTION_INTELLIGENCE_STATES_HPP_