#pragma once

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

typedef std::pair<int, int> Point;
typedef std::pair<Point, Point> Rectangle;  // {{min_y, min_x}, {max_y, max_x}}
typedef std::map<int, Rectangle> BoundingBoxMap;

BoundingBoxMap find_connected_components(const uint8_t* frame);
std::vector<std::pair<uint16_t, uint16_t>> find_blobs(const uint8_t* frame);