#pragma once

class Mos {
 private:
  static int id_counter;
  int _id = -1;
  double _x, _y;

 public:
  int id(void) const { return this->_id; };
  double x(void) const { return this->_x; };
  double y(void) const { return this->_y; };

  Mos() {
    _x = -1;
    _y = -1;
  };

  Mos(double _x, double _y) {
    this->_x = _x;
    this->_y = _y;
  };

  Mos(double _x, double _y, int _id) {
    this->_x = _x;
    this->_y = _y;
    this->_id = _id;
  };

  Mos(const Mos& other) {
    _x = other._x;
    _y = other._y;
    _id = other._id;
  };

  Mos(const Mos& other, const int& _id) {
    _x = other._x;
    _y = other._y;
    this->_id = _id;
  };

  Mos& operator=(const Mos& other) {
    _x = other._x;
    _y = other._y;
    _id = other._id;
    return *this;
  };

  bool operator==(const Mos& other) const {
    return (_x == other._x) && (_y == other._y) && (_id == other._id);
  };
};