#include <iostream>
#include "include/hungarian.hpp"

int main(void) {
  // please use "-std=c++11" for this initialization of vector.
  // clang-format off
  vector<vector<double> > costMatrix = {{10, 19, 8, 15},
                                        {0 , 18, 7, 17},
                                        {12, 19, 8, 18}};
  // clang-format on

  HungarianAlgorithm HungAlgo;
  vector<int> assignment;

  double cost = HungAlgo.Solve(costMatrix, assignment);

  for (unsigned int x = 0; x < costMatrix.size(); x++)
    std::cout << x << "," << assignment[x] << "\t";

  std::cout << "\ncost: " << cost << std::endl;

  return 0;
}