#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>

constexpr int DEBUG_OFF = 0;
constexpr int DEBUG_ON = 1;
constexpr int USER_DISPLAY = 2;
extern std::atomic<int> debug;

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

struct Pt_d {
  double x, y;
  int id = -1;

  Pt_d() {
    x = -1;
    y = -1;
  };

  Pt_d(double x, double y) {
    this->x = x;
    this->y = y;
  };

  Pt_d(double x, double y, int id) {
    this->x = x;
    this->y = y;
    this->id = id;
  };

  Pt_d(const Pt_d& other) {
    x = other.x;
    y = other.y;
    id = other.id;
  };

    Pt_d(const Pt_d& other, const int& id) {
    x = other.x;
    y = other.y;
    this->id = id;
  };

  Pt_d& operator=(const Pt_d& other) {
    x = other.x;
    y = other.y;
    id = other.id;
    return *this;
  };

  bool operator==(const Pt_d& other) const {
    return (x == other.x) && (y == other.y) && (id == other.id);
  };
};

struct Pt {
  int x, y;
  int id = -1;

  Pt() {
    x = -1;
    y = -1;
  };

  Pt(int x, int y) {
    this->x = x;
    this->y = y;
  };

  Pt(int x, int y, int id) {
    this->x = x;
    this->y = y;
    this->id = id;
  };

  Pt(const Pt& other) {
    x = other.x;
    y = other.y;
    id = other.id;
  };

  Pt& operator=(const Pt& other) {
    x = other.x;
    y = other.y;
    id = other.id;
    return *this;
  };

  bool operator==(const Pt& other) const {
    return (x == other.x) && (y == other.y) && (id == other.id);
  };

  Pt(const Pt_d& other) {
    x = static_cast<int>(std::round(other.x));
    y = static_cast<int>(std::round(other.y));
  };
};

struct Blob {
  int min_x, max_x;
  int min_y, max_y;
  int cen_x, cen_y;
  int n_pixels;
  int ID;
};
