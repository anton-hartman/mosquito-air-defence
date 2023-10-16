#include <iostream>
#include "include/frame.hpp"
#include "include/hungarian.hpp"
#include "include/kalman.hpp"
#include "include/tracking.hpp"

// int main(void) {
//   // please use "-std=c++11" for this initialization of vector.
//   // clang-format off
//   vector<vector<double> > costMatrix = {{10, 19, 8, 15},
//                                         {0 , 18, 7, 17},
//                                         {12, 19, 8, 18}};
//   // clang-format on

//   HungarianAlgorithm HungAlgo;
//   vector<int> assignment;

//   double cost = HungAlgo.Solve(costMatrix, assignment);

//   for (unsigned int x = 0; x < costMatrix.size(); x++)
//     std::cout << x << "," << assignment[x] << "\t";

//   std::cout << "\ncost: " << cost << std::endl;

//   return 0;
// }

int main(void) {
  Tracking tracker;
  std::vector<Pt> blob_centres;
  for (int i = 0; i < 10; i++) {
    blob_centres.push_back(Pt(i, i));
  }
  tracker.associate_and_update(blob_centres);

  return 0;
}