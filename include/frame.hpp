#pragma once

#include <cstdint>
#include <string>

extern const std::string width;
extern const std::string height;
extern const std::string left;
extern const std::string top;
extern const std::string right;
extern const std::string bottom;
extern const std::string croppped_width;
extern const std::string cropped_height;

extern const int COLS;
extern const int ROWS;

extern const std::string pipeline;

extern const double F_X;
extern const double F_Y;
extern const double C_X_DOUBLE;
extern const double C_Y_DOUBLE;
extern const uint16_t C_X;
extern const uint16_t C_Y;

extern const uint16_t TURRET_X_ORIGIN_PX;
extern const uint16_t TURRET_Y_ORIGIN_PX;

typedef struct pt_ {
  int x, y;
} Pt;

typedef struct blob_ {
  int min_x, max_x;
  int min_y, max_y;
  int cen_x, cen_y;
  int n_pixels;
  int ID;
} Blob;
