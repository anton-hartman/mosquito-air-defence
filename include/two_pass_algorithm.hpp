#pragma once

#include <map>
#include <utility>

typedef std::pair<int, int> Point;
typedef std::pair<Point, Point> Rectangle;  // {{min_y, min_x}, {max_y, max_x}}
typedef std::map<int, Rectangle> BoundingBoxMap;

BoundingBoxMap find_connected_components(const uint8_t* frame);
std::pair<uint16_t, uint16_t> find_laser_pos(const uint8_t* frame);