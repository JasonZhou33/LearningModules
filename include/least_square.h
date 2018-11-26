
#if !defined(RM_LEAST_SQUARE_H_)
#define RM_LEAST_SQUARE_H_

#include <opencv2/core/core.hpp>

#define PI 3.1415926
// use least square to calculate the angle
class LeastSquare {
 public:
  bool is_kxb;
  float tx, ty;
  float k, b, kh, bh;                //斜率k,截距b
  LeastSquare(std::vector<cv::Point>& point)  //构造函数，输入点，得到斜率
  {
    std::vector<int> x;
    std::vector<int> y;
    for (unsigned int i = 0; i < point.size(); ++i) {
      x.push_back(point[i].x);
      y.push_back(point[i].y);
    }
    float t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0;
    for (unsigned int i = 0; i < x.size(); ++i) {
      t1 += x[i] * x[i];
      t2 += x[i];
      t3 += x[i] * y[i];
      t4 += y[i];
      t5 += y[i] * y[i];
    }
    k = (t3 * x.size() - t2 * t4) / (t1 * x.size() - t2 * t2);
    // b= (t1 * t4 - t2 * t3) / (t1 * x.size() - t2 * t2);
    kh = (t3 * x.size() - t2 * t4) / (t5 * x.size() - t4 * t4);
    // bh= (t5 * t2 - t4 * t3) / (t5 * x.size() - t4 * t4);
  }
  float getAngle() { return atan(k) * 180 / PI; }
  float getAngleh() { return 90 - atan(kh) * 180 / PI; }
  float getFinalAngle() {
    if (k < 1)
      return getAngleh();
    else
      return getAngle();
  }
};

#endif // RM_LEAST_SQUARE_H_
