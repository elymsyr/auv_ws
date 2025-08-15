#ifndef AUV_CONTROL_MISSION_IMP_H
#define AUV_CONTROL_MISSION_IMP_H

#include <mutex>
#include <any>

// This enum defines the types of missions and commands that can be sent
// to the MissionSystem via its ROS 2 service.
enum MissionIMP {
    SonarMisTest,
    FollowMisTest,
    Test,
    SonarMis,
    FollowMis,
    START,
    STOP,
    REPORT
};

// Enum for sensor types
enum class SensorType {
    Sonar,
    Camera,
    GPS
};

#include <thread>
#include <atomic>

// Base class template for sensor data with mutex protection and collector thread
template <typename T>
class SensorDataBase {
public:
    mutable std::mutex mtx;
    std::thread collector_thread;
    std::atomic<bool> running{false};

    virtual ~SensorDataBase() {
        stopCollector();
    }

    virtual T getData() const = 0;
    virtual void setData(const T& data) = 0;

    // Start the collector thread
    virtual void startCollector() {
        running = true;
        collector_thread = std::thread([this]() { this->collect(); });
    }

    // Stop the collector thread
    virtual void stopCollector() {
        running = false;
        if (collector_thread.joinable())
            collector_thread.join();
    }

    // Overwrite this in derived classes for specific sensor collection logic
    virtual void collect() = 0;
};

// SonarData (BlueRobotics Sonar)
struct SonarData : public SensorDataBase<float> {
    float distance = 0.0f;
    float angle = 0.0f;

    SonarData(float angle = 10.0f) : angle(angle) {}

    void setData(const float& foundDistance) override {
        std::lock_guard<std::mutex> lock(mtx);
        distance = foundDistance;
    }

    float getData() const override {
        std::lock_guard<std::mutex> lock(mtx);
        return distance;
    }

    // Simulated collector for BlueRobotics Sonar
    void collect() override {
        while (running) {
            // TODO: Replace with actual BlueRobotics Sonar data acquisition
            float simulated_distance = 1.0f; // Placeholder
            setData(simulated_distance);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

// CameraData (Raspberry Pi Camera Module 3)
struct CameraData : public SensorDataBase<std::vector<uint8_t>> {
    std::vector<uint8_t> image;
    int width = 0;
    int height = 0;
    int channels = 0;
    double timestamp = 0.0;

    CameraData(int width = 0, int height = 0, int channels = 0)
        : width(width), height(height), channels(channels) {}

    void setData(const std::vector<uint8_t>& foundImage) override {
        std::lock_guard<std::mutex> lock(mtx);
        image = foundImage;
    }

    std::vector<uint8_t> getData() const override {
        std::lock_guard<std::mutex> lock(mtx);
        return image;
    }

    // Simulated collector for Raspberry Pi Camera Module 3
    void collect() override {
        while (running) {
            // TODO: Replace with actual camera capture logic
            std::vector<uint8_t> simulated_image(width * height * channels, 0); // Placeholder
            setData(simulated_image);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
};

// GPSData (HERE4 module)
struct GPSData : public SensorDataBase<std::array<double, 4>> {
    double latitude = 0.0;      // Latitude in degrees
    double longitude = 0.0;     // Longitude in degrees
    double altitude = 0.0;      // Altitude in meters
    double accuracy = 0.0;      // Estimated horizontal accuracy in meters
    double timestamp = 0.0;     // Time the data was recorded

    void setData(const std::array<double, 4>& data) override {
        std::lock_guard<std::mutex> lock(mtx);
        latitude = data[0];
        longitude = data[1];
        altitude = data[2];
        accuracy = data[3];
    }

    std::array<double, 4> getData() const override {
        std::lock_guard<std::mutex> lock(mtx);
        return {latitude, longitude, altitude, accuracy};
    }

    // Simulated collector for HERE4 GPS module
    void collect() override {
        while (running) {
            // TODO: Replace with actual HERE4 GPS data acquisition
            std::array<double, 4> simulated_gps = {0.0, 0.0, 0.0, 1.0}; // Placeholder
            setData(simulated_gps);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
};

#endif // AUV_CONTROL_MISSION_IMP_H