#include "auv_control/mapping/environment.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>
#include <cuda_runtime.h>
#include <cfloat>
#include "auv_control/mapping/config.h"

EnvironmentMap::EnvironmentMap() : width_(WIDTH), height_(HEIGHT), N(HORIZON), spacing_factor_(SPACING_FACTOR), r_m_(R_M_), round_(2 * M_PI) {
    int2 init = make_int2(0, 0);
    shift_ = init;
    shift_total_ = init;
    world_position_ = make_float3(0.0f, 0.0f, 0.0f);
    start_x = width_ / 2;
    start_y = height_ / 2;
    size_t size = WIDTH * HEIGHT * sizeof(uint8_t);
    cudaMalloc(&grid_, size);
    cudaMalloc(&tempGrid_, size);
    cudaMemset(grid_, 0, size);
    CHECK_CUDA(cudaMalloc(&node_grid_, width_ * height_ * sizeof(Node)));
    initializeGrid();
}

EnvironmentMap::~EnvironmentMap() {
    cudaFree(grid_);
    cudaFree(tempGrid_);
    CHECK_CUDA(cudaFree(node_grid_));
}

std::vector<std::pair<float, float>> EnvironmentMap::obstacle_selection(int number_obs) {
    const int MAX_CANDIDATES = 1000;
    const int TOP_K = number_obs > 0 ? number_obs : number_obs_to_feed_;

    // Device memory allocations
    float* d_dists;
    float* d_prior;
    float2* d_coords;
    int* d_count;
    
    CUDA_CALL(cudaMalloc(&d_dists, MAX_CANDIDATES * sizeof(float)));
    CUDA_CALL(cudaMalloc(&d_prior, MAX_CANDIDATES * sizeof(float)));
    CUDA_CALL(cudaMalloc(&d_coords, MAX_CANDIDATES * sizeof(float2)));
    CUDA_CALL(cudaMalloc(&d_count, sizeof(int)));
    CUDA_CALL(cudaMemset(d_count, 0, sizeof(int)));

    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);

    obstacleSelectionKernel<<<blocks, threads>>>(
        grid_, width_, height_, world_position_.x, world_position_.y,
        d_dists, d_coords, d_count, MAX_CANDIDATES, circle_radius_, r_m_
    );
    CUDA_CALL(cudaDeviceSynchronize());
    
    // Get candidate count
    int h_count;
    CUDA_CALL(cudaMemcpy(&h_count, d_count, sizeof(int), cudaMemcpyDeviceToHost));
    h_count = min(h_count, MAX_CANDIDATES);
    
    // Early return if no obstacles
    if (h_count == 0) {
        std::vector<std::pair<float, float>> result(TOP_K, {10000.0f, 10000.0f});
        CUDA_CALL(cudaFree(d_dists));
        CUDA_CALL(cudaFree(d_coords));
        CUDA_CALL(cudaFree(d_count));
        CUDA_CALL(cudaFree(d_prior));
        return result;
    }
    
    // Copy candidates to host
    std::vector<float> h_prior(h_count);
    std::vector<float2> h_coords(h_count);
    CUDA_CALL(cudaMemcpy(h_prior.data(), d_prior, h_count * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CALL(cudaMemcpy(h_coords.data(), d_coords, h_count * sizeof(float2), cudaMemcpyDeviceToHost));
    
    // Create index array and sort by distance to projected point
    std::vector<int> indices(h_count);
    for (int i = 0; i < h_count; ++i) indices[i] = i;
    
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return h_prior[a] < h_prior[b];
    });
    
    // Prepare results with top K closest obstacles
    std::vector<std::pair<float, float>> result;
    int num_to_return = std::min(TOP_K, h_count);
    for (int i = 0; i < num_to_return; ++i) {
        int idx = indices[i];
        result.push_back({h_coords[idx].x, h_coords[idx].y});
    }
    
    // Pad with default values if needed
    for (int i = num_to_return; i < TOP_K; ++i) {
        result.push_back({10000.0f, 10000.0f});
    }
    
    // Cleanup
    CUDA_CALL(cudaFree(d_dists));
    CUDA_CALL(cudaFree(d_coords));
    CUDA_CALL(cudaFree(d_count));
    CUDA_CALL(cudaFree(d_prior));

    return result;
}

PointBatch* EnvironmentMap::createPointBatch(int count) {
    PointBatch* batch = new PointBatch;
    batch->count = count;
    cudaMalloc(&batch->coords_dev, count * sizeof(float2));
    cudaMalloc(&batch->values_dev, count * sizeof(uint8_t));
    return batch;
}

void EnvironmentMap::destroyPointBatch(PointBatch* batch) {
    cudaFree(batch->coords_dev);
    cudaFree(batch->values_dev);
    delete batch;
}

void EnvironmentMap::fillPointBatchWithRandom(PointBatch* batch, int grid_width, int grid_height) {
    float2* h_coords = new float2[batch->count];
    uint8_t* h_values = new uint8_t[batch->count];
    
    for(int i = 0; i < batch->count; ++i) {
        h_coords[i].x = ((rand() % (grid_width * 25 * 2)) - (grid_width * 25)) / 100.0f;
        h_coords[i].y = ((rand() % (grid_height * 25 * 2)) - (grid_height * 25)) / 100.0f;
        h_values[i] = rand() % 1 + 50; // TEST
    }
    
    cudaMemcpy(batch->coords_dev, h_coords, 
               batch->count * sizeof(float2), cudaMemcpyHostToDevice);
    cudaMemcpy(batch->values_dev, h_values,
               batch->count * sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    delete[] h_coords;
    delete[] h_values;
}

void EnvironmentMap::slide(float x, float y) {
    world_position_.x = x;
    world_position_.y = y;
    shift_.x = static_cast<int>(x / r_m_) - shift_total_.x;
    shift_.y = static_cast<int>(y / r_m_) - shift_total_.y;
    if (shift_.x == 0 && shift_.y == 0) return;
    shift_total_.x += shift_.x;
    shift_total_.y += shift_.y;

    // std::cout << "world_position_: " << world_position_.x << ", " << world_position_.y << "\n"
    //             << "shift_total_: " << shift_total_.x << ", " << shift_total_.y << "\n";

    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);

    slidePhase1<<<blocks, threads>>>(grid_, tempGrid_, width_, height_, shift_);
    cudaDeviceSynchronize();
    slidePhase2<<<blocks, threads>>>(grid_, tempGrid_, width_, height_);
    cudaDeviceSynchronize();
}

void EnvironmentMap::updateWithBatch(PointBatch* batch) {
    const int blockSize = 256;
    const int gridSize = (batch->count + blockSize - 1) / blockSize;
    pointUpdateKernel<<<gridSize, blockSize>>>(
        grid_, width_, height_,
        world_position_.x, world_position_.y, r_m_,
        batch->coords_dev, batch->values_dev, batch->count
    );
    cudaDeviceSynchronize();
}

void EnvironmentMap::updateSinglePoint(float world_x, float world_y, uint8_t value, char mode) {
    singlePointUpdateKernel<<<1, 1>>>(grid_, width_, height_, 
                                      world_position_.x, world_position_.y, r_m_, 
                                      world_x, world_y, value, obstacle_radius_, mode);
    cudaDeviceSynchronize();
}

uint8_t* EnvironmentMap::getGridDevicePtr() const {
    return grid_;
}

void EnvironmentMap::copyGridToHost(uint8_t* host_buffer) const {
    size_t grid_size = width_ * height_ * sizeof(uint8_t);
    cudaMemcpy(host_buffer, grid_, grid_size, cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
}

void EnvironmentMap::save(std::string name) const {
    std::string filename_grid = "grid_" + name + ".bin";
    std::string filename_node_f = "node_" + name + "_f.bin";
    std::string filename_node_h = "node_" + name + "_h.bin";
    std::string filename_node_g = "node_" + name + "_g.bin";

    size_t total_elements = width_ * height_;
    uint8_t* h_grid = new uint8_t[total_elements];
    copyGridToHost(h_grid);

    std::ofstream file(filename_grid, std::ios::binary);
    file.write(reinterpret_cast<char*>(h_grid), total_elements);
    file.close();
    
    delete[] h_grid;

    float* node_buffer = new float[total_elements];
    
    copyNodeToHost(node_buffer, 'f');
    std::ofstream out_f(filename_node_f, std::ios::out | std::ios::binary);
    out_f.write(reinterpret_cast<const char*>(node_buffer), total_elements * sizeof(float));
    out_f.close();

    copyNodeToHost(node_buffer, 'h');
    std::ofstream out_h(filename_node_h, std::ios::out | std::ios::binary);
    out_h.write(reinterpret_cast<const char*>(node_buffer), total_elements * sizeof(float));
    out_h.close();

    copyNodeToHost(node_buffer, 'g');
    std::ofstream out_g(filename_node_g, std::ios::out | std::ios::binary);
    out_g.write(reinterpret_cast<const char*>(node_buffer), total_elements * sizeof(float));
    out_g.close();

    delete[] node_buffer;
}

void EnvironmentMap::debug_grid_update(float world_x, float world_y) {
    // Calculate expected grid coordinates
    int x_coor = static_cast<int>((world_x - world_position_.x) / r_m_ + width_ / 2.0f);
    int y_coor = static_cast<int>((world_y - world_position_.y) / r_m_ + height_ / 2.0f);

    std::cout << "=== Grid Update Debug ===\n";
    std::cout << "World coordinates: (" << world_x << ", " << world_y << ")\n";
    std::cout << "world_position_.x: " << world_position_.x << ", world_position_.y: " << world_position_.y << "\n";
    std::cout << "r_m_: " << r_m_ << "\n";
    std::cout << "Calculated grid coordinates: (" << x_coor << ", " << y_coor << ")\n";
    
    // Check if coordinates are valid
    if (x_coor >= 0 && x_coor < width_ && y_coor >= 0 && y_coor < height_) {
        std::cout << "Coordinates are within grid bounds\n";
        
        // Directly set the value using a debug kernel
        uint8_t debug_value = 255;
        
        // Create a debug batch
        PointBatch* debug_batch = createPointBatch(1);
        float2 coord = make_float2(world_x, world_y);
        
        CUDA_CALL(cudaMemcpy(debug_batch->coords_dev, &coord, sizeof(float2), cudaMemcpyHostToDevice));
        CUDA_CALL(cudaMemcpy(debug_batch->values_dev, &debug_value, sizeof(uint8_t), cudaMemcpyHostToDevice));
        
        // Run update kernel
        const int blockSize = 256;
        const int gridSize = (1 + blockSize - 1) / blockSize;
        pointUpdateKernel<<<gridSize, blockSize>>>(
            grid_, width_, height_,
            world_position_.x, world_position_.y, r_m_,
            debug_batch->coords_dev, debug_batch->values_dev, 1
        );
        CUDA_CALL(cudaDeviceSynchronize());
        
        // Check if value was set
        uint8_t grid_value;
        size_t offset = y_coor * width_ + x_coor;
        CUDA_CALL(cudaMemcpy(&grid_value, grid_ + offset, sizeof(uint8_t), cudaMemcpyDeviceToHost));
        
        std::cout << "Value at (" << x_coor << ", " << y_coor << "): " 
                  << static_cast<int>(grid_value) << "\n";
        
        destroyPointBatch(debug_batch);
    } else {
        std::cout << "Coordinates are OUTSIDE grid bounds!\n";
    }
    std::cout << "========================\n";
}

void EnvironmentMap::copyNodeToHost(float* host_buffer, const char mode) const {
    size_t node_count = width_ * height_;
    size_t byte_size = node_count * sizeof(Node);
    Node* node_buffer = new Node[node_count]; // Allocate correct number of elements
    cudaMemcpy(node_buffer, node_grid_, byte_size, cudaMemcpyDeviceToHost);
    
    if (mode == 'f') {
        for (size_t i = 0; i < node_count; ++i) {
            host_buffer[i] = node_buffer[i].g + node_buffer[i].h;
        }
    } else if (mode == 'g') {
        for (size_t i = 0; i < node_count; ++i) {
            host_buffer[i] = node_buffer[i].g;
        }
    } else if (mode == 'h') {
        for (size_t i = 0; i < node_count; ++i) {
            host_buffer[i] = node_buffer[i].h;
        }
    } else {
        fprintf(stderr, "Invalid mode '%c' in copyNodeToHost\n", mode);
    }
    delete[] node_buffer;
}

void EnvironmentMap::resetAll() {
    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);
    resetAllKernel<<<blocks, threads>>>(node_grid_, grid_, width_, height_);
    CUDA_CALL(cudaDeviceSynchronize());
}