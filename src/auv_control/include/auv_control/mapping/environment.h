#ifndef ENVIRONMENT_MAP_H
#define ENVIRONMENT_MAP_H

#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <utility>
#include <cstdint>
#include "auv_control/mapping/config.h"

#define CHECK_CUDA(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        fprintf(stderr, "CUDA error at %s:%d code=%d(%s)\n", \
            __FILE__, __LINE__, err, cudaGetErrorString(err)); \
        exit(EXIT_FAILURE); \
    } \
}

#define CUDA_CALL(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                  << ": " << cudaGetErrorString(err) << "\n"; \
        exit(EXIT_FAILURE); \
    } \
}

struct Node {
    int x, y;
    float g, h;
    int parent_x, parent_y;
    int status;
};

struct Path {
    int2* points;
    int length;
    float2* trajectory;
    float* angle;
};

struct PointBatch {
    int count;
    float2* coords_dev;
    uint8_t* values_dev;
};

class EnvironmentMap {
public:
    EnvironmentMap();
    ~EnvironmentMap();

    void slide(float dx, float dy);
    void updateWithBatch(class PointBatch* batch);
    void updateSinglePoint(float world_x, float world_y, uint8_t value, char mode = 'w');
    
    uint8_t* getGridDevicePtr() const;
    void copyGridToHost(uint8_t* host_buffer) const;
    void save(std::string name) const;

    std::vector<std::pair<float, float>> obstacle_selection(int number_obs = 0);

    static class PointBatch* createPointBatch(int count);
    static void destroyPointBatch(class PointBatch* batch);
    static void fillPointBatchWithRandom(class PointBatch* batch, int grid_width, int grid_height);
    void debug_grid_update(float world_x, float world_y);

    // astar
    // void updateGrid();
    Path findPath(float2 ref, float nu_x = 0.0f, float nu_y = 0.0f);
    void copyNodeToHost(float* host_buffer, const char mode = 'f') const;
    void resetAll();

    int width_;
    int height_;
    int2 shift_total_;
    float3 world_position_;
    float r_m_;
    int2 shift_;
    uint8_t *grid_;
    uint8_t *tempGrid_;
    float2 ref_;
    float centre_move_factor_ = 0.0f;
    float circle_radius_ = 10.0f;
    int number_obs_to_feed_ = 50;
    int max_iter_ = 1000;
    int obstacle_radius_ = 12;
    int N;

private:
    float round_;
    float2 v_;
    Node* node_grid_;
    Path path_;
    float* d_dists;
    float* d_prior;
    float2* d_coords;
    int* d_count;
    int start_x;
    int start_y;
    float spacing_factor_;

    // astar
    void initializeGrid();
};

// map helper
float2 move_to(float x1, float y1, float x2, float y2, float factor);
float distance(float x1, float y1, float x2, float y2);
float angleBetweenPoints(float fromX, float fromY, float toX, float toY);
float2 createPath(int m, float k, float spacing, const EnvironmentMap& map, const Path& path);
void drawDirection(EnvironmentMap& map, float x, float y, float angle, float length = 1.0f, float value = 200.0f);

// map kernel
__global__ void slidePhase1(uint8_t* grid, uint8_t* tempGrid, int width, int height, int2 shift);
__global__ void slidePhase2(uint8_t* grid, uint8_t* tempGrid, int width, int height);
__global__ void pointUpdateKernel(uint8_t* grid, int width, int height, float x_r, float y_r, float r_m, float2* coords_dev, uint8_t* values_dev, int count);
__global__ void singlePointUpdateKernel(uint8_t* grid, int width, int height, float x_r, float y_r, float r_m,float world_x, float world_y, uint8_t value, int radius = 5, char mode = 'w');
__global__ void obstacleSelectionKernel(uint8_t* grid, int width, int height, float wx, float wy, float* output_dists, float2* output_coords, int* output_count, int max_output, float circle_radius, float r_m_);

// astar kernel
__global__ void adjustGoalKernel(uint8_t* grid, int width, int height, int goal_x, int goal_y, int* adjusted_goal);
__device__ float atomicMinFloat(float* address, float val);
__global__ void initKernel(Node* grid, int width, int height);
__global__ void resetGridKernel(Node* grid, uint8_t* map, int width, int height, int goal_x, int goal_y);
__global__ void resetAllKernel(Node* grid, uint8_t* map, int width, int height);
__global__ void wavefrontKernel(Node* grid, int width, int height, int* d_updated);
__global__ void reconstructPathKernel(Node* grid, int2* path, int* path_length, int start_x, int start_y, int goal_x, int goal_y, int width);
__global__ void adjustPointToOpenNode(uint8_t* grid, int width, int height, int x, int y, int* adjusted_point);
__global__ void samplePathKernel(int2* raw_path, int raw_path_length, float2* trajectory, int N, int grid_width, int grid_height, float r_m, float2 world_position);
__global__ void computeYawKernel(float2* trajectory, float* angles, int N);
__global__ void smoothAnglesKernel(float* angles, int N, float smooth_factor);
__global__ void computeCurvatureKernel(int2* path, int path_length, float* curvatures);
__global__ void adaptiveSamplePathKernel(int2* raw_path, int raw_path_length,  float* curvatures, float2* trajectory, int N, int grid_width, int grid_height, float r_m, float2 world_position, float spacing_factor);

#endif