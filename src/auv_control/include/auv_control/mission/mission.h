#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <string>
#include <array>
#include <any>
#include <functional>
#include "auv_control/mapping/environment.h"
#include <mutex>
#include <memory>
#include "auv_control/mission/mission_imp.h"
#include <thread>
#include <atomic>
#include <queue>

class Mission {
public:
    Mission(std::string name_) : name_(name_) {create_map();} 
    virtual ~Mission() {}
    
    std::vector<SensorType> sensors;
    std::vector<std::function<void(const std::array<double,12>&)>> state_list;
    std::string name_;
    int id_;
    float2 ref;
    float ref_depth;
    std::array<std::array<double, 12>, HORIZON> ref_path;
    int state = -1;
    int path_error = 0;

    // Mission lifecycle methods
    virtual void initialize() {}
    
    std::array<std::array<double, 12>, HORIZON> step(const std::array<double, 12>& current_state) {
        map_->slide(current_state[0], current_state[1]);
        updateMap(current_state);
        if (state >= 0 && state < static_cast<int>(state_list.size())) {
            state_list[state](current_state);
        }
        return ref_path;
    }
    
    virtual void terminate() {}
    
    virtual void report() {}

    virtual std::string getMissionName() const {
        return name_;
    }

private:
    void create_map() {
        map_ = std::make_unique<EnvironmentMap>();
    }
    virtual void updateMap(const std::array<double, 12>& current_state) {};
    
    EnvironmentMap* map() { return map_.get(); }
    virtual void set_state_list() {}

protected:
    std::unique_ptr<EnvironmentMap> map_;
};

#endif // MISSION_H