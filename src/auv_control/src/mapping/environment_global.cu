#include "auv_control/mapping/environment.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>
#include <cuda_runtime.h>
#include <cfloat>

// Map
__global__ void slidePhase1(uint8_t* grid, uint8_t* tempGrid, int width, int height, int2 shift) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    int dst_idx = ty * width + tx;
    int src_x = tx + shift.x;
    int src_y = ty + shift.y;

    if (src_x >= 0 && src_x < width && src_y >= 0 && src_y < height) {
        tempGrid[dst_idx] = grid[src_y * width + src_x];
    } else {
        tempGrid[dst_idx] = 0;
    }
}

__global__ void slidePhase2(uint8_t* grid, uint8_t* tempGrid, int width, int height) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    int idx = ty * width + tx;
    grid[idx] = tempGrid[idx];
}

__global__ void pointUpdateKernel(uint8_t* grid, int width, int height, float x_r, float y_r, float r_m, float2* coords_dev, uint8_t* values_dev, int count) {
    unsigned tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= count) return;

    float2 coord = coords_dev[tid];
    uint8_t val = values_dev[tid];

    int x_coor = static_cast<int>((coord.x - x_r) / r_m + width / 2.0f);
    int y_coor = static_cast<int>((coord.y - y_r) / r_m + height / 2.0f);

    if (x_coor >= 0 && x_coor < width && y_coor >= 0 && y_coor < height) {
        grid[y_coor * width + x_coor] = val;
    }
}

__global__ void singlePointUpdateKernel(uint8_t* grid, int width, int height, float x_r, float y_r, float r_m, float world_x, float world_y, uint8_t value, int radius, char mode) {
    if (mode == 'w') {
        world_x = (world_x - x_r) / r_m + width / 2.0f;
        world_y = (world_y - y_r) / r_m + height / 2.0f;
    }

    int x_coor = __float2int_rd(world_x);
    int y_coor = __float2int_rd(world_y);

    if (x_coor >= 0 && x_coor < width && y_coor >= 0 && y_coor < height) {
        int index = y_coor * width + x_coor;
        grid[index] = value;
    }

    if (value >= 250) {
        for (int di = -radius; di <= radius; di++) {
            for (int dj = -radius; dj <= radius; dj++) {
                // Calculate squared distance to avoid sqrt
                if (di * di + dj * dj <= radius * radius) {
                    int ni = y_coor + di;
                    int nj = x_coor + dj;
                    if (ni >= 0 && ni < height && nj >= 0 && nj < width) {
                        int n_index = ni * width + nj;
                        if (grid[n_index] < 250) {
                            grid[n_index] = value;
                        }
                    }
                }
            }
        }
    }
}

__global__ void obstacleSelectionKernel(uint8_t* grid, int width, int height, float wx, float wy, float* output_dists, float2* output_coords, int* output_count, int max_output, float circle_radius, float r_m_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (idx >= width || idy >= height) return;
    
    int grid_idx = idy * width + idx;
    int cx = width / 2;
    int cy = height / 2;
    float dx = (float)(idx - cx);
    float dy = (float)(idy - cy);
    float dist = sqrtf(dx * dx + dy * dy) * r_m_;
    if (grid[grid_idx] >= 250 && (dist < circle_radius)) {
        int pos = atomicAdd(output_count, 1);
        if (pos < max_output) {
            output_dists[pos] = dist;
            output_coords[pos] = make_float2(wx + dx * r_m_, wy + dy * r_m_);
        }
        return;
    }
}

// A*
__global__ void adjustGoalKernel(uint8_t* grid, int width, int height, int goal_x, int goal_y, int* adjusted_goal) {
    // Check if original goal is free
    if (goal_x >= 0 && goal_x < width && goal_y >= 0 && goal_y < height) {
        if (grid[goal_y * width + goal_x] < 250) {
            adjusted_goal[0] = goal_x;
            adjusted_goal[1] = goal_y;
            return;
        }
    }

    int max_radius = max(width, height);
    // Search in increasing radius around goal
    for (int r = 1; r < max_radius; r++) {
        // Check top and bottom rows
        for (int dx = -r; dx <= r; dx++) {
            int x = goal_x + dx;
            int y_top = goal_y + r;
            int y_bottom = goal_y - r;

            // Top row
            if (x >= 0 && x < width && y_top >= 0 && y_top < height) {
                if (grid[y_top * width + x] < 250) {
                    adjusted_goal[0] = x;
                    adjusted_goal[1] = y_top;
                    return;
                }
            }

            // Bottom row
            if (x >= 0 && x < width && y_bottom >= 0 && y_bottom < height) {
                if (grid[y_bottom * width + x] < 250) {
                    adjusted_goal[0] = x;
                    adjusted_goal[1] = y_bottom;
                    return;
                }
            }
        }

        // Check left and right columns (skip corners)
        for (int dy = -r + 1; dy < r; dy++) {
            int y = goal_y + dy;
            int x_right = goal_x + r;
            int x_left = goal_x - r;

            // Right column
            if (y >= 0 && y < height && x_right >= 0 && x_right < width) {
                if (grid[y * width + x_right] < 250) {
                    adjusted_goal[0] = x_right;
                    adjusted_goal[1] = y;
                    return;
                }
            }

            // Left column
            if (y >= 0 && y < height && x_left >= 0 && x_left < width) {
                if (grid[y * width + x_left] < 250) {
                    adjusted_goal[0] = x_left;
                    adjusted_goal[1] = y;
                    return;
                }
            }
        }
    }

    // Fallback to original goal
    adjusted_goal[0] = goal_x;
    adjusted_goal[1] = goal_y;
}

__device__ float atomicMinFloat(float* address, float val) {
    int* address_as_i = (int*)address;
    int old = *address_as_i;
    int expected;
    float old_val;
    do {
        expected = old;
        old_val = __int_as_float(expected);
        if (old_val <= val) return old_val;  // Return early if no update needed
        old = atomicCAS(address_as_i, expected, __float_as_int(val));
    } while (expected != old);
    return old_val;
}

__global__ void initKernel(Node* grid, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    node->x = idx;
    node->y = idy;
    node->g = FLT_MAX;
    node->h = FLT_MAX;
    node->parent_x = -1;
    node->parent_y = -1;
    node->status = 1;
}

__global__ void resetGridKernel(Node* grid, uint8_t* map, int width, int height, int goal_x, int goal_y) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    
    node->parent_x = -1;
    node->parent_y = -1;
    
    // Always mark obstacles as closed
    node->status = (map[index] >= 250) ? 0 : 1;
    
    if (idx == goal_x && idy == goal_y) {
        // Only set goal if it's traversable
        if (node->status == 1) {
            node->g = 0.0f;
        } else {
            node->g = FLT_MAX;
        }
    } else {
        node->g = FLT_MAX;
    }
    node->h = sqrtf(powf(idx - goal_x, 2) + powf(idy - goal_y, 2));
}

__global__ void resetAllKernel(Node* grid, uint8_t* map, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    node->x = idx;
    node->y = idy;
    node->g = FLT_MAX;
    node->h = FLT_MAX;
    node->parent_x = -1;
    node->parent_y = -1;
    node->status = 1;
    map[index] = 0;
}

__global__ void wavefrontKernel(Node* grid, int width, int height, int* d_updated) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;
    
    int index = idy * width + idx;
    Node* node = &grid[index];
    
    // Skip obstacles and goal node
    if (node->status != 1) return;
    if (node->g == 0.0f) return;

    float min_g = FLT_MAX;
    int best_px = -1;
    int best_py = -1;

    // Check 8 neighbors
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = idx + dx;
            int ny = idy + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            
            int nidx = ny * width + nx;
            Node* neighbor = &grid[nidx];
            
            // Skip obstacles
            if (neighbor->status != 1) continue;

            // Skip unvisited neighbors
            if (neighbor->g == FLT_MAX) continue;

            float cost = (dx != 0 && dy != 0) ? 1.4142f : 1.0f;
            float new_g = neighbor->g + cost;
            
            if (new_g < min_g) {
                min_g = new_g;
                best_px = nx;
                best_py = ny;
            }
        }
    }

    // Update if we found a better path
    if (min_g < node->g) {
        node->g = min_g;
        node->parent_x = best_px;
        node->parent_y = best_py;
        atomicOr(d_updated, 1);
    }
}

__global__ void reconstructPathKernel(Node* grid, int2* path, int* path_length, int start_x, int start_y, int goal_x, int goal_y, int width) {
    int x = start_x;
    int y = start_y;
    int count = 0;
    int max_length = width * width;  // Safeguard

    // Follow parent pointers from start to goal
    while (x != goal_x || y != goal_y) {
        if (count >= max_length) break;
        path[count++] = make_int2(x, y);
        
        int idx = y * width + x;
        int px = grid[idx].parent_x;
        int py = grid[idx].parent_y;
        
        if (px == -1 || py == -1) break;
        x = px;
        y = py;
    }
    
    // Add final goal point if reached
    if (x == goal_x && y == goal_y) {
        path[count++] = make_int2(x, y);
    }
    *path_length = count;
}

__global__ void adjustPointToOpenNode(uint8_t* grid, int width, int height, int x, int y, int* adjusted_point) {
    // Check if original point is open
    if (x >= 0 && x < width && y >= 0 && y < height && grid[y * width + x] < 250) {
        adjusted_point[0] = x;
        adjusted_point[1] = y;
        return;
    }

    int max_radius = max(width, height);
    // Spiral search for nearest open node
    for (int r = 1; r < max_radius; r++) {
        // Check top and bottom rows
        for (int dx = -r; dx <= r; dx++) {
            int nx = x + dx;
            // Top row
            int ny_top = y + r;
            if (nx >= 0 && nx < width && ny_top >= 0 && ny_top < height && 
                grid[ny_top * width + nx] < 250) {
                adjusted_point[0] = nx;
                adjusted_point[1] = ny_top;
                return;
            }
            
            // Bottom row
            int ny_bottom = y - r;
            if (nx >= 0 && nx < width && ny_bottom >= 0 && ny_bottom < height && 
                grid[ny_bottom * width + nx] < 250) {
                adjusted_point[0] = nx;
                adjusted_point[1] = ny_bottom;
                return;
            }
        }

        // Check left and right columns (skip corners)
        for (int dy = -r + 1; dy < r; dy++) {
            int ny = y + dy;
            // Right column
            int nx_right = x + r;
            if (nx_right >= 0 && nx_right < width && ny >= 0 && ny < height && 
                grid[ny * width + nx_right] < 250) {
                adjusted_point[0] = nx_right;
                adjusted_point[1] = ny;
                return;
            }
            
            // Left column
            int nx_left = x - r;
            if (nx_left >= 0 && nx_left < width && ny >= 0 && ny < height && 
                grid[ny * width + nx_left] < 250) {
                adjusted_point[0] = nx_left;
                adjusted_point[1] = ny;
                return;
            }
        }
    }

    // Fallback to original if no open node found
    adjusted_point[0] = x;
    adjusted_point[1] = y;
}

__global__ void samplePathKernel(int2* raw_path, int raw_path_length, float2* trajectory, int N, int grid_width, int grid_height, float r_m, float2 world_position) {
    int k = threadIdx.x + blockIdx.x * blockDim.x;
    if (k > N) return;

    // Handle case where raw path is too short
    if (raw_path_length <= 1) {
        if (raw_path_length == 1) {
            int2 point = raw_path[0];
            float world_x = (point.x - grid_width/2.0f) * r_m + world_position.x;
            float world_y = (point.y - grid_height/2.0f) * r_m + world_position.y;
            trajectory[k] = make_float2(world_x, world_y);
        } else {
            trajectory[k] = make_float2(0.0f, 0.0f);
        }
        return;
    }

    // Calculate position along path
    float t = static_cast<float>(k) / N;
    float path_index = t * (raw_path_length - 1);
    
    int idx1 = min(static_cast<int>(floorf(path_index)), raw_path_length - 1);
    int idx2 = min(idx1 + 1, raw_path_length - 1);
    float frac = path_index - idx1;

    // Get grid points
    int2 p1 = raw_path[idx1];
    int2 p2 = raw_path[idx2];

    // Interpolate
    float grid_x = p1.x + frac * (p2.x - p1.x);
    float grid_y = p1.y + frac * (p2.y - p1.y);

    // Convert to world coordinates
    float world_x = (grid_x - grid_width/2.0f) * r_m + world_position.x;
    float world_y = (grid_y - grid_height/2.0f) * r_m + world_position.y;

    trajectory[k] = make_float2(world_x, world_y);
}

__global__ void computeYawKernel(float2* trajectory, float* angles, int N) {
    int k = threadIdx.x + blockIdx.x * blockDim.x;
    if (k > N) return;  // Only process N+1 points
    
    if (k < N) {
        // Use next point for direction
        float2 current = trajectory[k];
        float2 next = trajectory[k+1];
        angles[k] = atan2f(next.y - current.y, next.x - current.x);
    } else if (k == N && N > 0) {
        // Use previous point for last point
        float2 current = trajectory[k];
        float2 prev = trajectory[k-1];
        angles[k] = atan2f(current.y - prev.y, current.x - prev.x);
    } else {
        // Single point case
        angles[k] = 0.0f;
    }
}

__global__ void smoothAnglesKernel(float* angles, int N, float smooth_factor) {
    int k = threadIdx.x + blockIdx.x * blockDim.x;
    if (k > N || k == 0) return;  // Start from k=1

    float prev_angle = angles[k-1];
    float current_angle = angles[k];
    
    // Normalize difference to [-π, π]
    float diff = current_angle - prev_angle;
    if (diff > M_PI) diff -= 2*M_PI;
    if (diff < -M_PI) diff += 2*M_PI;
    
    // Apply smoothing
    angles[k] = prev_angle + smooth_factor * diff;
}

__global__ void computeCurvatureKernel(int2* path, int path_length, float* curvatures) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= path_length) return;

    // Handle endpoints with zero curvature
    if (i == 0 || i == path_length - 1) {
        curvatures[i] = 0.0f;
        return;
    }

    // Get three consecutive points
    int2 p0 = path[i-1];
    int2 p1 = path[i];
    int2 p2 = path[i+1];

    // Calculate vectors
    float2 v1 = make_float2(p1.x - p0.x, p1.y - p0.y);
    float2 v2 = make_float2(p2.x - p1.x, p2.y - p1.y);

    // Calculate vector magnitudes
    float mag1 = sqrtf(v1.x*v1.x + v1.y*v1.y);
    float mag2 = sqrtf(v2.x*v2.x + v2.y*v2.y);

    // Calculate dot product and cross product
    float dot = v1.x*v2.x + v1.y*v2.y;
    float cross = v1.x*v2.y - v1.y*v2.x;

    // Calculate angle between vectors
    float cos_theta = dot / (mag1 * mag2);
    cos_theta = fmaxf(fminf(cos_theta, 1.0f), -1.0f);  // Clamp for numerical stability
    float theta = acosf(cos_theta);

    // Determine turn direction for sign
    float sign = (cross < 0) ? -1.0f : 1.0f;
    
    // Calculate curvature (angle per distance)
    float avg_distance = (mag1 + mag2) / 2.0f;
    curvatures[i] = sign * theta / (avg_distance + 1e-5f);  // Avoid division by zero
}

__global__ void adaptiveSamplePathKernel(int2* raw_path, int raw_path_length,  float* curvatures, float2* trajectory, int N, int grid_width, int grid_height, float r_m, float2 world_position, float spacing_factor) {
    int j = threadIdx.x + blockIdx.x * blockDim.x;
    if (j > N) return;

    // Handle degenerate paths
    if (raw_path_length <= 1) {
        if (raw_path_length == 1) {
            int2 point = raw_path[0];
            float world_x = (point.x - grid_width/2.0f) * r_m + world_position.x;
            float world_y = (point.y - grid_height/2.0f) * r_m + world_position.y;
            trajectory[j] = make_float2(world_x, world_y);
        } else {
            trajectory[j] = world_position;
        }
        return;
    }

    // Convert spacing_factor to integer step (n_val) and ensure min step=1
    int n_val = max(1, (int)spacing_factor);
    
    // Calculate index in raw_path: start at n_val, then n_val + j*n_val
    int index_in_path = n_val + j * n_val;
    
    // Clamp index to valid range [0, raw_path_length-1]
    if (index_in_path >= raw_path_length) {
        index_in_path = raw_path_length - 1;
    }

    // Get grid point and convert to world coordinates
    int2 point = raw_path[index_in_path];
    float world_x = (point.x - grid_width/2.0f) * r_m + world_position.x;
    float world_y = (point.y - grid_height/2.0f) * r_m + world_position.y;
    trajectory[j] = make_float2(world_x, world_y);
}

// __global__ void adaptiveSamplePathKernel(int2* raw_path, int raw_path_length,  float* curvatures, float2* trajectory, int N, int grid_width, int grid_height, float r_m, float2 world_position, float spacing_factor) {
//     int j = threadIdx.x + blockIdx.x * blockDim.x;
//     if (j > N) return;

//     // Handle degenerate paths
//     if (raw_path_length <= 1) {
//         if (raw_path_length == 1) {
//             int2 point = raw_path[0];
//             float world_x = (point.x - grid_width/2.0f) * r_m + world_position.x;
//             float world_y = (point.y - grid_height/2.0f) * r_m + world_position.y;
//             trajectory[j] = make_float2(world_x, world_y);
//         } else {
//             trajectory[j] = world_position;
//         }
//         return;
//     }

//     // Calculate adaptive spacing parameters
//     float min_spacing = 0.5f;
//     float max_spacing = 10.0f;
//     float curvature_threshold = 0.3f;  // Radians per grid cell

//     // Calculate base spacing (average spacing)
//     float total_length = 0.0f;
//     for (int i = 1; i < raw_path_length; i++) {
//         float dx = raw_path[i].x - raw_path[i-1].x;
//         float dy = raw_path[i].y - raw_path[i-1].y;
//         total_length += sqrtf(dx*dx + dy*dy);
//     }
//     float base_spacing = total_length / N;

//     // Determine spacing factor based on curvature
//     if (j < N) {
//         // Find the segment index for this trajectory point
//         int seg_idx = min(j * raw_path_length / N, raw_path_length - 2);
        
//         // Get curvature at the start of the segment
//         float curvature = fabsf(curvatures[seg_idx]);
        
//         // Adjust spacing based on curvature
//         if (curvature > curvature_threshold) {
//             // High curvature - use denser points
//             spacing_factor = 0.3f + 0.7f * (curvature_threshold / curvature);
//         } else {
//             // Low curvature - use sparser points
//             spacing_factor = 1.0f + 2.0f * (1.0f - curvature / curvature_threshold);
//         }
//     }

//     // Apply spacing limits
//     float spacing = base_spacing * spacing_factor;
//     spacing = fmaxf(min_spacing, fminf(spacing, max_spacing));

//     // Calculate path index with adaptive spacing
//     float path_index = j * spacing;
//     int idx1 = min(static_cast<int>(floorf(path_index)), raw_path_length - 1);
//     int idx2 = min(idx1 + 1, raw_path_length - 1);
//     float frac = path_index - idx1;

//     // Get grid points
//     int2 p1 = raw_path[idx1];
//     int2 p2 = raw_path[idx2];

//     // Interpolate between points
//     float grid_x = p1.x + frac * (p2.x - p1.x);
//     float grid_y = p1.y + frac * (p2.y - p1.y);

//     // Convert to world coordinates
//     float world_x = (grid_x - grid_width/2.0f) * r_m + world_position.x;
//     float world_y = (grid_y - grid_height/2.0f) * r_m + world_position.y;

//     trajectory[j] = make_float2(world_x, world_y);
// }
