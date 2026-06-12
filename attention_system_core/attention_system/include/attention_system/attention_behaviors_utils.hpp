#ifndef ATTENTION_BEHAVIORS_UTILS_HPP_
#define ATTENTION_BEHAVIORS_UTILS_HPP_

#include <iostream>
#include <unordered_map>
#include <map>
#include <vector>
#include <string>

namespace attention_system
{

struct AttentionActuationCapability {
  bool value;
  std::string name;
  std::vector<std::string> alternative_capabilities;

  bool operator==(const AttentionActuationCapability& other) const {
    return value == other.value && 
           name == other.name && 
           alternative_capabilities == other.alternative_capabilities;
  }
};

/**
 * @brief A set of AttentionActuationCapability structs for the attention_system.
 * * NOTE: It is not generalistic. Is specific for attention system.
 * * Check get_all_capabilities method comment
 */
struct AttentionActuationCapabilities {
  AttentionActuationCapability turn_around{false, "turn_around", {}};
  AttentionActuationCapability move_around{false, "move_around", {}};
  AttentionActuationCapability use_joint{false, "use_joint", {}};

  private:
    /**
     * @brief Returns a list of pointers to all capabilities. 
     * * If you add new capabilities to the struct, you just 
     * have to add them to this list in this method
     * (also in get_all_capabilities_mod).
     */
    std::vector<const AttentionActuationCapability*> get_all_capabilities() const {
      return {&turn_around, &move_around, &use_joint};
    }

    /**
     * @brief Returns a vector of modificable pointers to all capabilities.
     * Useful for modifying the capabilities iteratively.
     */
    std::vector<AttentionActuationCapability*> get_all_capabilities_mod() {
      return {&turn_around, &move_around, &use_joint};
    }

  public:
    /**
     * @brief Returns a modifiable pointer to the capability based on its name.
     */
    AttentionActuationCapability* get_capability_by_name(const std::string& target_name) {
      for (auto* cap : get_all_capabilities_mod()) {
        if (cap->name == target_name) {
          return cap;
        }
      }
      return nullptr;
    }

    friend std::ostream& operator<<(std::ostream& os, const AttentionActuationCapabilities& act) {
      os << "{ ";
      auto caps = act.get_all_capabilities();
      
      for (size_t i = 0; i < caps.size(); ++i) {
        const auto* cap = caps[i];
        
        os << cap->name << ": " << (cap->value ? "true" : "false");

        if (!cap->alternative_capabilities.empty()) {
          os << " (alternatives: [";
          for (size_t j = 0; j < cap->alternative_capabilities.size(); ++j) {
            os << "'" << cap->alternative_capabilities[j] << "'";
            if (j < cap->alternative_capabilities.size() - 1) {
              os << ", ";
            }
          }
          os << "])";
        }

        if (i < caps.size() - 1) {
          os << ", ";
        }
      }
      os << " }";
      return os;
    }

    bool operator==(const AttentionActuationCapabilities & other) const {
      auto my_caps = get_all_capabilities();
      auto other_caps = other.get_all_capabilities();

      for (size_t i = 0; i < my_caps.size(); ++i) {
        if (!(*my_caps[i] == *other_caps[i])) {
          return false;
        }
      }
      return true;
    }

  private:
    /**
     * @brief Helper to evaluate a capability value by its string name.
     */
    bool is_capability_available(const std::string& target_name) const {
      for (const auto* cap : get_all_capabilities()) {
        if (cap->name == target_name) {
          return (cap->value);
        }
      }

      return false;
    }

    /**
     * @brief Checks if a specific requirement is satisfied directly or via its alternatives.
    */
    bool is_requirement_met(const AttentionActuationCapability& req) const {
      if (!req.value) {
        return true;
      }

      if (is_capability_available(req.name)) {
        return true;
      }

      for (const std::string& alt : req.alternative_capabilities) {
        if (is_capability_available(alt)) {
          return true;
        }
      }

      return false;
    }

  public:
    /**
     * @brief Checks if a set of capabilities are either available, or not required, given a set of required capabilities.
     * * @param other The required capabilities.
     * @return true if there is no required capability missing.
     */
    bool meets_requirements(const AttentionActuationCapabilities& other) const {
      for (const auto* req : other.get_all_capabilities()) {
        if (!is_requirement_met(*req)) {
          return false;
        }
      }
      return true;
    }
};

struct AttentionBehaviorPromptInformation {
  std::string explanation;
  std::vector<std::string> inputs;

  friend std::ostream& operator<<(std::ostream& os, const AttentionBehaviorPromptInformation& info) {
    os << "{\n";
    os << "    explanation: '" << info.explanation << "',\n";
    
    os << "    inputs: [";
    for (size_t i = 0; i < info.inputs.size(); ++i) {
      os << "'" << info.inputs[i] << "'";
      if (i < info.inputs.size() - 1) {
        os << ", ";
      }
    }
    os << "]\n";
    os << "  }";
    return os;
  }
};

struct AttentionBehaviorParams {
  int behavior_id;
  std::string name;
  AttentionActuationCapabilities behavior_actuation_needed;
  AttentionBehaviorPromptInformation prompt_information;
  std::vector<std::string> bt_blackboard_inputs_not_assigned;
  std::unordered_map<std::string, std::string> bt_blackboard_inputs_assigned;
  std::string node_name;

  friend std::ostream& operator<<(std::ostream& os, const AttentionBehaviorParams& params) {
    os << "AttentionBehaviorParams {\n";
    os << "  behavior_id: " << params.behavior_id << ",\n";
    os << "  name: '" << params.name << "',\n";
    os << "  node_name: `" << params.node_name << "',\n";

    os << "  behavior_actuation_needed: " << params.behavior_actuation_needed << ",\n";
    os << "  prompt_information: " << params.prompt_information << ",\n";
    
    os << "  bt_blackboard_inputs_not_assigned: [";
    for (size_t i = 0; i < params.bt_blackboard_inputs_not_assigned.size(); ++i) {
      os << "'" << params.bt_blackboard_inputs_not_assigned[i] << "'";
      if (i < params.bt_blackboard_inputs_not_assigned.size() - 1) {
        os << ", ";
      }
    }
    os << "],\n";

    os << "  bt_blackboard_inputs_assigned: {";
    bool first = true;
    for (const auto& [key, value] : params.bt_blackboard_inputs_assigned) {
      if (!first) os << ", ";
      os << "'" << key << "': '" << value << "'";
      first = false;
    }
    os << "}\n";
    
    os << "}";
    return os;
  }
};

}

#endif  // ATTENTION_BEHAVIORS_UTILS_HPP_
