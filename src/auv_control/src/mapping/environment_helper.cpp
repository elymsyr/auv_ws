#include "auv_control/mapping/environment.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>
#include <cfloat>
#include <cmath>

float2 move_to(float x1, float y1, float x2, float y2, float factor) {
    // Calculate direction vector
    float dx = x2 - x1;
    float dy = y2 - y1;
    // Compute the distance to the target point
    float length = std::sqrt(dx * dx + dy * dy);
    // Compute new point
    float proj_x = x1 + (dx / length) * factor;
    float proj_y = y1 + (dy / length) * factor;
    return make_float2(proj_x, proj_y);
}

float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

float heuristic(int2 a, int2 b) {
    // Using Euclidean distance as heuristic
    return std::sqrt(static_cast<float>((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)));
}

float angleBetweenPoints(float fromX, float fromY, float toX, float toY) {
    float deltaX = toX - fromX;
    float deltaY = toY - fromY;
    return std::atan2(deltaY, deltaX);
}

float2 createPath(int m, float k, float spacing, const EnvironmentMap& map, const Path& path) {
    float path_index = k * spacing;
    int idx1 = static_cast<int>(floor(path_index));
    int idx2 = static_cast<int>(ceil(path_index));
    float frac = path_index - idx1;
    
    // Ensure indices stay within bounds
    idx1 = std::min(idx1, m-1);
    idx2 = std::min(idx2, m-1);
    
    // Get grid coordinates
    int2 point1 = path.points[idx1];
    int2 point2 = path.points[idx2];
    
    // Linear interpolation between points
    float grid_x = point1.x + frac * (point2.x - point1.x);
    float grid_y = point1.y + frac * (point2.y - point1.y);
    
    // Convert to world coordinates
    float world_x = (grid_x - map.width_/2.0f) * map.r_m_ + map.world_position_.x;
    float world_y = (grid_y - map.height_/2.0f) * map.r_m_ + map.world_position_.y;
    return make_float2(world_x, world_y);
}

void drawDirection(EnvironmentMap& map, float x, float y, float angle, float length, float value) {
    float endX = x + length * std::cos(angle);
    float endY = y + length * std::sin(angle);
    
    // Simple Bresenham's line algorithm (or use your map's line-drawing method)
    int steps = static_cast<int>(std::max(std::abs(endX - x), std::abs(endY - y)) * 10);
    for (int i = 0; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        float lineX = x + t * (endX - x);
        float lineY = y + t * (endY - y);
        map.updateSinglePoint(lineX, lineY, value);
    }
}