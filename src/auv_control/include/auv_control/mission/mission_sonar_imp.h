#ifndef AUV_CONTROL_MISSION_SONAR_MISSION_H
#define AUV_CONTROL_MISSION_SONAR_MISSION_H

#include "auv_control/mission/mission.h" // Keep the base class for polymorphism
#include "auv_control/msg/test_sonar_topic.hpp" // Use the ROS 2 message
#include "auv_control/mapping/config.h" // For float3, etc.

#include <cmath>
#include <mutex>
#include <memory>
#include <vector>
#include <functional>

// Note: Using a raw pointer for convert_obs_to_world is risky.
// A std::vector<float3> would be a safer return type to prevent memory leaks.
struct float3; 

class SonarMission : public Mission {
public:
    SonarMission();

    // --- Core Mission Interface ---
    void initialize() override;
    void terminate() override;
    void report() override;
    // The step() function is inherited from the Mission base class

    // --- New Data Injection Method ---
    // This is how the MissionSystem provides sonar data to this class.
    void update_sonar_data(const auv_control::msg::TestSonarTopic::SharedPtr& msg);

private:
    // This helper now takes a const reference to the message
    std::vector<float3> convert_obs_to_world(const std::array<double, 12>& state, 
                                             const auv_control::msg::TestSonarTopic& sonar_data);
    
    // The rest of the internal state machine and logic remains the same
    void state_initial();
    void updateMap(const std::array<double, 12>& current_state) override;
    void state_0(const std::array<double, 12>& current_state);
    void state_1(const std::array<double, 12>& current_state);
    void state_2(const std::array<double, 12>& current_state);
    void state_3(const std::array<double, 12>& current_state);
    void state_4(const std::array<double, 12>& current_state);
    void state_5(const std::array<double, 12>& current_state);
    void setState(int new_state);
    void set_state_list();

protected:
    // Member data for storing the latest sonar reading
    auv_control::msg::TestSonarTopic testsonar_state_;
    std::mutex testsonar_mtx_;
};

#endif // AUV_CONTROL_MISSION_SONAR_MISSION_H