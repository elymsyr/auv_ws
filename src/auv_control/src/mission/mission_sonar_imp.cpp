#include "auv_control/mission/mission_sonar_imp.h"
#include <iostream>

// The constructor is now very simple. It just sets up the state machine.
SonarMission::SonarMission() : Mission("Sonar Obstacle Avoidance") {
    std::cout << "[SonarMission] Constructor called\n";
    set_state_list();
    std::cout << "[SonarMission] Initialization complete\n";
}

// This is the new entry point for sonar data from the MissionSystem.
void SonarMission::update_sonar_data(const auv_control::msg::TestSonarTopic::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(testsonar_mtx_);
    testsonar_state_ = *msg;
}

void SonarMission::initialize() { state_initial(); }
void SonarMission::terminate() { /* Add termination logic if needed */ }
void SonarMission::report() { /* Add reporting logic if needed */ }

std::vector<float3> SonarMission::convert_obs_to_world(const std::array<double, 12>& state, 
                                                       const auv_control::msg::TestSonarTopic& sonar_data) {
    std::vector<float3> obstacle_world(10);
    double robot_x = state[0], robot_y = state[1], robot_z = state[2];
    double yaw = state[5];
    
    const double deg2rad = M_PI / 180.0;
    double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
    
    for (int i = 0; i < 10; i++) {
        if (sonar_data.detections[i] < 0.1) continue;

        double angle_rad = sonar_data.degrees[i] * deg2rad;
        double local_x = sonar_data.detections[i] * std::cos(angle_rad);
        double local_y = sonar_data.detections[i] * std::sin(angle_rad);
        
        double world_x = local_x * cos_yaw - local_y * sin_yaw + robot_x;
        double world_y = local_x * sin_yaw + local_y * cos_yaw + robot_y;
        
        obstacle_world[i] = {static_cast<float>(world_x), static_cast<float>(world_y), static_cast<float>(robot_z)};
    }
    return obstacle_world;
}

void SonarMission::updateMap(const std::array<double, 12>& current_state) {
    auv_control::msg::TestSonarTopic local_sonar_data;
    {
        std::lock_guard<std::mutex> lock(testsonar_mtx_);
        local_sonar_data = testsonar_state_;
    }
    
    std::vector<float3> obstacles = convert_obs_to_world(current_state, local_sonar_data);
    for (const auto& obs : obstacles) {
        if (obs.x < 0.2f && obs.y < 0.2f) continue;
        map_->updateSinglePoint(obs.x, obs.y, 255.0f);
    }
}

// --- ALL OTHER STATE MACHINE METHODS (state_0, state_1, etc.) ---
// No changes are needed for the other state machine methods (state_0, 
// state_1, state_2, state_3, state_4, state_5, setState, set_state_list, state_initial).
// They can remain exactly as you wrote them. I am omitting them here for brevity.
// ...
#include "auv_control/mission/mission_sonar_imp.h"
#include <iostream>  // Add for debug output

// ... (previous methods) ...

void SonarMission::state_initial() {
    std::cout << "[SonarMission] INITIAL STATE\n";
    ref = make_float2(28.0f, 32.0f);
    ref_depth = -1;
    state = 0;

    for (size_t i = 0; i < HORIZON; i++) {
        std::array<double, 12> ref_state{};
        ref_state.fill(0.0);
        ref_path[i] = ref_state;
    }
    std::cout << "  Set reference point: (" << ref.x << ", " << ref.y << ")\n";
}

void SonarMission::state_0(const std::array<double, 12>& current_state) {
    std::cout << "\n[SonarMission] STATE 0 - Depth adjustment\n";
    std::cout << "  Current depth: " << current_state[2] << " | Target depth: ";
    
    if (ref_depth == -1) {
        ref_depth = 5;
        std::cout << "5 (initial set)\n";
    } else if (ref_depth < 15) {
        ref_depth += 5;
        std::cout << ref_depth << " (+5 increment)\n";
    } else {
        ref_depth -=10;
        std::cout << ref_depth << " (-10 decrement)\n";
    }
    
    setState(1);
    map_->resetAll();
    
    double depth_error = std::abs(current_state[2] - ref_depth);
    std::cout << "  Depth error: " << depth_error << "\n";
    
    for (size_t i = 0; i < HORIZON; i++) {
        ref_path[i][0] = current_state[0];
        ref_path[i][1] = current_state[1];
        ref_path[i][2] = ref_depth;

        if (depth_error < 1) {
            ref_path[i][5] = current_state[5] + 0.1 * i;
        } else {
            ref_path[i][5] = current_state[5];
        }
    }
}

void SonarMission::state_1(const std::array<double, 12>& current_state) {
    std::cout << "\n[SonarMission] STATE 1 - Depth stabilization\n";
    double depth_error = std::abs(current_state[2] - ref_depth);
    std::cout << "  Depth error: " << depth_error 
              << " | Velocity: " << current_state[8] << "\n";
    
    if (depth_error < 0.2 && current_state[8] < 0.1) {
        std::cout << "  Conditions met for state transition\n";
        setState(2);
    }

    for (size_t i = 0; i < HORIZON; i++) {
        ref_path[i][0] = current_state[0];
        ref_path[i][1] = current_state[1];
        ref_path[i][2] = ref_depth;

        if (depth_error < 1) {
            ref_path[i][5] = current_state[5] + 0.1 * i;
        } else {
            ref_path[i][5] = current_state[5];
        }
    }
}

void SonarMission::state_2(const std::array<double, 12>& current_state) {
    std::cout << "\n[SonarMission] STATE 2 - Yaw alignment\n";
    double dx = static_cast<double>(ref.x) - current_state[0];
    double dy = static_cast<double>(ref.y) - current_state[1];
    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = std::abs(desired_yaw - current_state[5]);
    
    std::cout << "  Target: (" << ref.x << ", " << ref.y << ")\n"
              << "  Current: (" << current_state[0] << ", " << current_state[1] << ")\n"
              << "  Yaw error: " << yaw_error 
              << " | Angular vel: " << current_state[11] << "\n";
    
    if (yaw_error < 0.1 && current_state[11] < 0.1) {
        std::cout << "  Yaw aligned to target\n";
        setState(3);
    }

    for (size_t i = 0; i < HORIZON; i++) {
        ref_path[i][0] = current_state[0];
        ref_path[i][1] = current_state[1];
        ref_path[i][2] = ref_depth;
        ref_path[i][5] = desired_yaw;
    }
}

void SonarMission::state_3(const std::array<double, 12>& current_state) {
    std::cout << "\n[SonarMission] STATE 3 - Path planning\n";
    Path path = map_->findPath(ref);
    std::cout << "  Path length: " << path.length << "\n";

    double pos_error = std::hypot(ref.x - current_state[0], ref.y - current_state[1]);
    std::cout << "  Position error: " << pos_error << "\n";

    if (path.length < 2 && pos_error >= 0.2) {
        std::cout << "  Path finding failed! Attempt: " << path_error << "/5\n";
        if (path_error > 5) {
            std::cout << "  Max path errors reached, resetting\n";
            setState(0);
            return;
        }

        double dx = static_cast<double>(ref.x) - current_state[0];
        double dy = static_cast<double>(ref.y) - current_state[1];
        double desired_yaw = std::atan2(dy, dx);
        for (int k = 0; k < HORIZON; k++) {
            ref_path[k][0] = current_state[0];
            ref_path[k][1] = current_state[1];
            ref_path[k][2] = ref_depth;
            ref_path[k][5] = desired_yaw;
        }
        path_error++;
    } else {
        for (int k = 0; k < HORIZON; k++) {
            float2 position = path.trajectory[k];
            ref_path[k][0] = position.x;
            ref_path[k][1] = position.y;
            ref_path[k][2] = ref_depth;
            ref_path[k][5] = path.angle[k];
        }
        path_error = 0;
    }
    
    if (pos_error < 0.2 && current_state[6] < 0.1 && current_state[7] < 0.1) {
        std::cout << "  Target position reached!\n";
        ref_depth = 1;
        setState(4);
    }
}

void SonarMission::state_4(const std::array<double, 12>& current_state) {
    std::cout << "\n[SonarMission] STATE 4 - Final approach\n";
    ref_depth = -2;
    for (int k = 0; k < HORIZON; k++) {
        ref_path[k][0] = ref.x;
        ref_path[k][1] = ref.y;
        ref_path[k][2] = ref_depth;
        ref_path[k][5] = 0;
        ref_path[k][8] = -2 + (2/HORIZON)*k;
    }
    std::cout << "  Moving to final depth\n";

    if (current_state[2] <= 0.2) {
        std::cout << "  Surface reached!\n";
        setState(5);
    }
}

void SonarMission::state_5(const std::array<double, 12>& current_state) {
    std::cout << "\n[SonarMission] STATE 6 - Mission complete\n";
    for (int k = 0; k < HORIZON; k++) {
        ref_path[k][0] = current_state[0];
        ref_path[k][1] = current_state[1];
        ref_path[k][2] = -1;
        ref_path[k][5] = 0;
        ref_path[k][8] = -1.5;
    }
    std::cout << "  Terminating mission\n";
    terminate();
    report();
}

void SonarMission::setState(int new_state) {
    if (state != new_state) {
        std::cout << "[STATE CHANGE] " << state << " -> " << new_state << "\n";
        state = new_state;
    }
}

void SonarMission::set_state_list() {
    state_list = {
        [this](const std::array<double, 12>& s){ state_0(s); },
        [this](const std::array<double, 12>& s){ state_1(s); },
        [this](const std::array<double, 12>& s){ state_2(s); },
        [this](const std::array<double, 12>& s){ state_3(s); },
        [this](const std::array<double, 12>& s){ state_4(s); },
        [this](const std::array<double, 12>& s){ state_5(s); },
    };
}