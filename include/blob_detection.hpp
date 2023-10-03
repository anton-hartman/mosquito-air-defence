#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

namespace bd {

typedef struct pt_ {
  int x, y;
} pt;
typedef struct blob_ {
  int min_x, max_x;
  int min_y, max_y;
  int cen_x, cen_y;
  int n_pixels;
  int ID;
} blob;

int get_blobs(Mat img, vector<blob>& blobs);

}  // namespace bd