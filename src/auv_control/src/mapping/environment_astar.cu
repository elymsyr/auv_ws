#include <cmath>
#include <vector>
#include <cuda_runtime.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "auv_control/mapping/environment.h"
#include <fstream>
#include <iostream>

void EnvironmentMap::initializeGrid() {
    dim3 block(16, 16);
    dim3 grid((width_ + block.x - 1) / block.x, 
              (height_ + block.y - 1) / block.y);

    initKernel<<<grid, block>>>(node_grid_, width_, height_);
    CHECK_CUDA(cudaDeviceSynchronize());
}

Path EnvironmentMap::findPath(float2 ref,  float nu_x, float nu_y) {
    start_x = width_ / 2;
    start_y = height_ / 2;
    v_.x = nu_x;
    v_.y = nu_y;

    float grid_ref_x = (ref.x - world_position_.x) / r_m_ + (width_ / 2.0f);
    float grid_ref_y = (ref.y - world_position_.y) / r_m_ + (height_ / 2.0f);
    grid_ref_x = std::max(0.0f, std::min(grid_ref_x, static_cast<float>(width_ - 1)));
    grid_ref_y = std::max(0.0f, std::min(grid_ref_y, static_cast<float>(height_ - 1)));
    int goal_x = static_cast<int>(grid_ref_x);
    int goal_y = static_cast<int>(grid_ref_y);
    
    // Adjust goal to nearest open node
    int* d_goal;
    int h_goal[2] = {goal_x, goal_y};
    CUDA_CALL(cudaMalloc(&d_goal, 2 * sizeof(int)));
    CUDA_CALL(cudaMemcpy(d_goal, h_goal, 2 * sizeof(int), cudaMemcpyHostToDevice));
    adjustPointToOpenNode<<<1, 1>>>(grid_, width_, height_, goal_x, goal_y, d_goal);
    CUDA_CALL(cudaDeviceSynchronize());
    CUDA_CALL(cudaMemcpy(h_goal, d_goal, 2 * sizeof(int), cudaMemcpyDeviceToHost));
    goal_x = h_goal[0];
    goal_y = h_goal[1];
    CUDA_CALL(cudaFree(d_goal));

    // Adjust start to nearest open node
    int* d_start;
    int h_start[2] = {start_x, start_y};
    CUDA_CALL(cudaMalloc(&d_start, 2 * sizeof(int)));
    CUDA_CALL(cudaMemcpy(d_start, h_start, 2 * sizeof(int), cudaMemcpyHostToDevice));
    adjustPointToOpenNode<<<1, 1>>>(grid_, width_, height_, start_x, start_y, d_start);
    CUDA_CALL(cudaDeviceSynchronize());
    CUDA_CALL(cudaMemcpy(h_start, d_start, 2 * sizeof(int), cudaMemcpyDeviceToHost));
    start_x = h_start[0];
    start_y = h_start[1];
    CUDA_CALL(cudaFree(d_start));

    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);
    resetGridKernel<<<blocks, threads>>>(node_grid_, grid_, width_, height_, goal_x, goal_y);
    CUDA_CALL(cudaDeviceSynchronize());
              
    // Step 2: Wavefront propagation
    int* d_updated;
    int h_updated = 1;
    CUDA_CALL(cudaMalloc(&d_updated, sizeof(int)));
    
    for (int iter = 0; iter < max_iter_ && h_updated; iter++) {
        h_updated = 0;
        CUDA_CALL(cudaMemcpy(d_updated, &h_updated, sizeof(int), cudaMemcpyHostToDevice));
        
        wavefrontKernel<<<blocks, threads>>>(node_grid_, width_, height_, d_updated);
        CUDA_CALL(cudaDeviceSynchronize());
        CUDA_CALL(cudaMemcpy(&h_updated, d_updated, sizeof(int), cudaMemcpyDeviceToHost));
    }

    // Step 3: Reconstruct path
    int2* d_path;
    int* d_path_length;
    CUDA_CALL(cudaMalloc(&d_path, width_ * height_ * sizeof(int2)));
    CUDA_CALL(cudaMalloc(&d_path_length, sizeof(int)));
    CUDA_CALL(cudaMemset(d_path_length, 0, sizeof(int)));
    
    reconstructPathKernel<<<1, 1>>>(node_grid_, d_path, d_path_length, 
                                   start_x, start_y, goal_x, goal_y, width_);
    CUDA_CALL(cudaDeviceSynchronize());
    
    int path_length;
    CUDA_CALL(cudaMemcpy(&path_length, d_path_length, sizeof(int), cudaMemcpyDeviceToHost));
    
    int2* h_path = new int2[path_length];
    if (path_length > 0) {
        CUDA_CALL(cudaMemcpy(h_path, d_path, path_length * sizeof(int2), cudaMemcpyDeviceToHost));
    }

    // Step 4: Sample path and create trajectory on GPU
    float* d_curvatures;
    CUDA_CALL(cudaMalloc(&d_curvatures, path_length * sizeof(float)));
    
    dim3 block_curv(256);
    dim3 grid_curv((path_length + block_curv.x - 1) / block_curv.x);
    
    computeCurvatureKernel<<<grid_curv, block_curv>>>(d_path, path_length, d_curvatures);
    CUDA_CALL(cudaDeviceSynchronize());

    // Step 5: Sample path with dynamic spacing
    float2* d_trajectory;
    float* d_angles;
    CUDA_CALL(cudaMalloc(&d_trajectory, (N+1) * sizeof(float2)));
    CUDA_CALL(cudaMalloc(&d_angles, (N+1) * sizeof(float)));
    
    dim3 block_traj(256);
    dim3 grid_traj((N+1 + block_traj.x - 1) / block_traj.x);
    
    adaptiveSamplePathKernel<<<grid_traj, block_traj>>>(
        d_path, path_length,
        d_curvatures,
        d_trajectory, N,
        width_, height_,
        r_m_,
        make_float2(world_position_.x, world_position_.y), spacing_factor_
    );
    CUDA_CALL(cudaDeviceSynchronize());
    
    // Compute yaw angles
    computeYawKernel<<<grid_traj, block_traj>>>(d_trajectory, d_angles, N);
    CUDA_CALL(cudaDeviceSynchronize());
    
    // Smooth angles
    float smooth_factor = 0.3f;
    smoothAnglesKernel<<<grid_traj, block_traj>>>(d_angles, N, smooth_factor);
    CUDA_CALL(cudaDeviceSynchronize());
    
    // Copy results back to host
    float2* h_trajectory = new float2[N+1];
    float* h_angles = new float[N+1];
    CUDA_CALL(cudaMemcpy(h_trajectory, d_trajectory, (N+1)*sizeof(float2), cudaMemcpyDeviceToHost));
    CUDA_CALL(cudaMemcpy(h_angles, d_angles, (N+1)*sizeof(float), cudaMemcpyDeviceToHost));

    // Cleanup
    CUDA_CALL(cudaFree(d_updated));
    CUDA_CALL(cudaFree(d_path));
    CUDA_CALL(cudaFree(d_path_length));
    
    return {h_path, path_length, h_trajectory, h_angles};
}