// Copyright (c) 2018 JachinShen(jachinshen@foxmail.com)
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "armor.h"

using namespace cv;
using std::cout;
using std::endl;
using std::vector;

Armor::Armor()
    : kcf_tracker_(false, true, false, false),
      BORDER_IGNORE(10),
      BOX_EXTRA(10){};

void Armor::init() {
  // init uart
  uart_.init();

  // fsm_state_ machine
  fsm_state_ = FAST_EXPLORE;

  found_ctr_ = 0;
  unfound_ctr_ = 0;
#if BAYER_HACK == HACKING_OFF
  src_width_ = 640;
  src_height_ = 480;
#elif BAYER_HACK == HACKING_ON
  src_width_ = 320;
  src_height_ = 240;
#endif

#if BAYER_HACK == HACKING_OFF
  GRAY_THRESH = 230;
#elif BAYER_HACK == HACKING_ON
  GRAY_THRESH = 10;
#endif

  // select contours
#if BAYER_HACK == HACKING_OFF
  CONTOUR_AREA_MIN = 30;    // 20
  CONTOUR_AREA_MAX = 3000;  // 2000
  CONTOUR_LENGTH_MIN = 10;  // 20
#elif BAYER_HACK == HACKING_ON
  CONTOUR_AREA_MIN = 5;     // 20
  CONTOUR_AREA_MAX = 1000;  // 2000
  CONTOUR_LENGTH_MIN = 5;   // 20
#endif
  CONTOUR_HW_RATIO_MIN = 1.0;       // 2.5
  SLOW_CONTOUR_HW_RATIO_MIN = 3.0;  // 2.5
  CONTOUR_HW_RATIO_MAX = 15;
  SLOW_CONTOUR_HW_RATIO_MAX = 7;
  CONTOUR_ANGLE_MAX = 20.0;

  // pair light_blobs
  TWIN_ANGEL_MAX = 5.001;
  TWIN_LENGTH_RATIO_MAX = 2.0;
  SLOW_TWIN_LENGTH_RATIO_MAX = 1.2;
  TWIN_DISTANCE_N_MIN = 1.3;       // 1.7
  SLOW_TWIN_DISTANCE_N_MIN = 2.0;  // 1.7
  TWIN_DISTANCE_N_MAX = 3.8;       // 3.8
  SLOW_TWIN_DISTANCE_N_MAX = 2.6;  // 1.7
  TWIN_DISTANCE_T_MAX = 1.4;
  TWIN_AREA_MAX = 1.2;

  // fsm_state_ machine
  FAST_EXPLORE_TRACK_THRES = 1;
  FAST_EXPLORE_SEND_STOP_THRES = 5;
  FAST_TRACK_SLOW_THRES = 1;  // 3
  // FAST_TRACK_CHECK_RATIO       = 0.4;
  FAST_TRACK_EXPLORE_THRES = 1;  // 2

  SLOW_EXPLORE_TRACK_THRES = 1;
  SLOW_EXPLORE_SEND_STOP_THRES = 5;
  SLOW_TRACK_CHECK_THRES = 3;
  SLOW_TRACK_CHECK_RATIO = 0.4;
  SLOW_TRACK_EXPLORE_THRES = 3;

  armor_type_ = NOT_FOUND;
  armor_type_table_[NOT_FOUND_LEAVE] = 0xA3;
  armor_type_table_[NOT_FOUND] = 0xA4;
  armor_type_table_[SMALL_ARMOR] = 0xA6;
  armor_type_table_[LARGE_ARMOR] = 0xA8;
}

int Armor::run(Mat& src) {
  if (src.empty()) return -1;
  if (src.channels() != 1) {
    cvtColor(src, src, CV_BGR2GRAY);
  }
#if DRAW == SHOW_ALL
  imshow("src", src);
#endif
#if BAYER_HACK == HACKING_ON
  static Mat blue(src.rows / 2, src.cols / 2, CV_8UC1);
  static Mat red(src.rows / 2, src.cols / 2, CV_8UC1);
  splitBayerBG(src, blue, red);
  // src = blue - red;
  src = red - blue;
#endif

  if (fsm_state_ == FAST_EXPLORE) {
    if (fastExplore(src)) {
      ++found_ctr_;
      unfound_ctr_ = 0;
    } else {
      ++unfound_ctr_;
      found_ctr_ = 0;
    }

    if (found_ctr_ >= FAST_EXPLORE_TRACK_THRES) {
      uartSendTarget((armor_box_.x + armor_box_.width / 2),
                     (armor_box_.y + armor_box_.height / 2), armor_type_);
      // init track with this src
      // otherwise, if use next src, the area may change
      trackInit(src);
      armor_last_box_ = armor_box_;
      transferState(FAST_TRACK);
    }
    if (unfound_ctr_ >= FAST_EXPLORE_SEND_STOP_THRES) {
      uartSendTarget(src_width_ / 2, src_height_ / 2, NOT_FOUND);
      found_ctr_ = 0;
      unfound_ctr_ = 0;
    }
  } else if (fsm_state_ == FAST_TRACK) {
    if (track(src)) {
      int x = armor_box_.x + armor_box_.width / 2;
      int y = armor_box_.y + armor_box_.height / 2;
      int x_last = armor_last_box_.x + armor_last_box_.width / 2;
      int center_x = 2 * x - src_width_ / 2;
      int center_y = 2 * y - src_height_ / 2;
      // Assume the box run at const velocity
      // Predict if the center is still in box at next src
      if (armor_last_box_.x < center_x &&
          center_x < armor_last_box_.x + armor_last_box_.width &&
          armor_last_box_.y < center_y &&
          center_y < armor_last_box_.y + armor_last_box_.height) {
        // if center is in box, predict it run at const velocity
        uartSendTarget(2 * x - x_last, y, armor_type_);
      } else {
        uartSendTarget(x, y, armor_type_);
      }
      ++found_ctr_;
      unfound_ctr_ = 0;
      armor_last_box_ = armor_box_;
    } else {
      ++unfound_ctr_;
      found_ctr_ = 0;
    }

    if (found_ctr_ >= FAST_TRACK_SLOW_THRES) {
      // check whether the robot slows down
      if (src_width_ / 2 - 30 < armor_box_.x + armor_box_.width / 2 &&
          armor_box_.x + armor_box_.width / 2 < src_width_ / 2 + 30) {
        transferState(SLOW_EXPLORE);
      }

      // check whether tracking the wrong area
      Mat roi = src.clone()(armor_box_);
      threshold(roi, roi, GRAY_THRESH, 255, THRESH_BINARY);
      if (countNonZero(roi) < SLOW_TRACK_CHECK_RATIO * total_contour_area) {
        uartSendTarget(src_width_ / 2, src_height_ / 2, NOT_FOUND);
        transferState(FAST_EXPLORE);
      }

      // add for secure
      // if it stay FAST TRACK too long,
      // it means the target beyond the shooting range
      // send NOT FOUND LEAVE IMMEDIATELY during src 500~800, about 3 seconds
      if (found_ctr_ >= 500) {
        uartSendTarget(src_width_ / 2, src_height_ / 2, NOT_FOUND_LEAVE);
        if (found_ctr_ >= 800) {
          transferState(FAST_EXPLORE);
        }
      }
    }

    // sometimes, kcf_tracker_ only miss 1 src
    if (unfound_ctr_ >= FAST_TRACK_EXPLORE_THRES) {
      transferState(FAST_EXPLORE);
    }
#if DRAW == SHOW_ALL
    // Draw the tracked object
    rectangle(src, armor_box_, Scalar(255, 0, 0), 2, 1);
    // Display src.
    imshow("Tracking", src);
#endif
  } else if (fsm_state_ == SLOW_EXPLORE) {
    if (slowExplore(src)) {
      ++found_ctr_;
      unfound_ctr_ = 0;
    } else {
      ++unfound_ctr_;
      found_ctr_ = 0;
    }

    if (found_ctr_ >= SLOW_EXPLORE_TRACK_THRES) {
      cout << "Find: " << armor_type_ << endl;
      uartSendTarget((armor_box_.x + armor_box_.width / 2),
                     (armor_box_.y + armor_box_.height / 2), armor_type_);
      trackInit(src);
      armor_last_box_ = armor_box_;
      transferState(SLOW_TRACK);
    }
    if (unfound_ctr_ >= SLOW_EXPLORE_SEND_STOP_THRES) {
      uartSendTarget(src_width_ / 2, src_height_ / 2, NOT_FOUND_LEAVE);
      transferState(FAST_EXPLORE);
    }
  } else if (fsm_state_ == SLOW_TRACK) {
    if (track(src)) {
      int x = armor_box_.x + armor_box_.width / 2;
      int y = armor_box_.y + armor_box_.height / 2;
      int x_last = armor_last_box_.x + armor_last_box_.width / 2;
      int center_x = 2 * x - src_width_ / 2;
      int center_y = 2 * y - src_height_ / 2;
      // Assume the box run at const velocity
      // Predict if the center is still in box at next src
      if (armor_last_box_.x < center_x &&
          center_x < armor_last_box_.x + armor_last_box_.width &&
          armor_last_box_.y < center_y &&
          center_y < armor_last_box_.y + armor_last_box_.height) {
        // if center is in box, predict it run at const velocity
        uartSendTarget(2 * x - x_last, y, armor_type_);
      } else {
        uartSendTarget(x, y, armor_type_);
      }
      ++found_ctr_;
      unfound_ctr_ = 0;
      armor_last_box_ = armor_box_;
    } else {
      ++unfound_ctr_;
      found_ctr_ = 0;
    }

    // check if the box is still tracking armor
    if (found_ctr_ >= SLOW_TRACK_CHECK_THRES) {
      Mat roi = src.clone()(armor_box_);
      threshold(roi, roi, GRAY_THRESH, 255, THRESH_BINARY);
      if (countNonZero(roi) < SLOW_TRACK_CHECK_RATIO * total_contour_area) {
        uartSendTarget(src_width_ / 2, src_height_ / 2, NOT_FOUND);
        transferState(FAST_EXPLORE);
      }
    }

    // sometimes, kcf_tracker_ only miss 1 src
    if (unfound_ctr_ >= SLOW_TRACK_EXPLORE_THRES) {
      transferState(FAST_EXPLORE);
    }
#if DRAW == SHOW_ALL
    // Draw the tracked object
    rectangle(src, armor_box_, Scalar(255, 0, 0), 2, 1);
    // Display src.
    imshow("Tracking", src);
#endif
  }
  return 0;
}

void Armor::transferState(FSMState s) {
  found_ctr_ = 0;
  unfound_ctr_ = 0;
  fsm_state_ = s;
  cout << "Transfer to fsm_state_: " << s << endl;
}

void Armor::trackInit(Mat& src) {
#if DRAW == SHOW_ALL
  // Display bounding box.
  rectangle(src, armor_box_, Scalar(255, 0, 0), 2, 1);
  imshow("TrackInit", src);
#endif
  kcf_tracker_.init(armor_box_, src);
}

bool Armor::track(Mat& src) {
  // Update the tracking result
  bool ok = true;
  armor_box_ = kcf_tracker_.update(src);
  if (armor_box_.x < BORDER_IGNORE || armor_box_.y < BORDER_IGNORE ||
      armor_box_.x + armor_box_.width > src_width_ - BORDER_IGNORE ||
      armor_box_.y + armor_box_.height > src_height_ - BORDER_IGNORE) {
    ok = false;
  }
  return ok;
}
// find armors in the whole img when the robot moves fast
// return true if found
bool Armor::fastExplore(Mat& src) {
  vector<LightBlob> light_blobs;
  // find possible light blobs
  if (fastSelectContours(src, light_blobs) == false) {
    return false;
  }
  if (fastPairContours(light_blobs) == false) {
    return false;
  }
  return true;
}

bool Armor::fastSelectContours(Mat& src, vector<LightBlob>& light_blobs) {
  static Mat bin;
  threshold(src, bin, GRAY_THRESH, 255, THRESH_BINARY);
#if DRAW == SHOW_ALL
  imshow("gray", bin);
#endif
  vector<vector<Point> > contours;
  findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//只检测最外层轮廓，并保存轮廓上所有点：
  // select contours by area, length, width/height
  for (unsigned int i = 0; i < contours.size(); ++i) {
    float area = contourArea(contours.at(i));
    // cout << "area:" << area << endl;
    if (area > CONTOUR_AREA_MAX || area < CONTOUR_AREA_MIN) {
      continue;
    }
    RotatedRect rec = minAreaRect(contours.at(i));
    Size2f size = rec.size;//Size2f：尺寸類別，和Size用法雷同，只是width和height為float型態。
    // get length (longer) as length
    float length = size.height > size.width ? size.height : size.width;
    float width = size.height < size.width ? size.height : size.width;
    // cout << "length: " << length << endl;
    if (length < CONTOUR_LENGTH_MIN) {
      continue;
    }
    // check if it is thin
    // cout << "length / width: " << length / width << endl;
    if (length / width > CONTOUR_HW_RATIO_MAX ||
        length / width < CONTOUR_HW_RATIO_MIN) {
#if DRAW == SHOW_ALL
      drawContours(bin, contours, i, Scalar(100), CV_FILLED);
#endif
      continue;
    }

    // cout << "Area Ratio: " << area / size.area() << endl;
    if (area / size.area() < 0.6) continue;

    // cout << "RotatedRect: " << angle << endl;
    LeastSquare leasq(contours[i]);
    // cout << "LeastSquare: " << leasq.getAngle() << " | " << leasq.getAngleh()
    // << endl;
    float angle = leasq.getAngleh();
    if (angle > 120.0 || angle < 60.0) continue;
    // cout << "push back" << endl;
    light_blobs.push_back(LightBlob(rec, contours[i], angle, area));
  }
  if (light_blobs.size() < 2) {
    return false;
  }
  return true;
}

bool Armor::fastPairContours(vector<LightBlob>& light_blobs) {
  int light_blob_i = -1, light_blob_j = -1;
  float min_similarity = 3.0;
  sort(light_blobs.begin(), light_blobs.end());
  // cout << "light_blobs: " << light_blobs.size() << endl;
  // pair light_blobs by length, distance, angel
  for (unsigned int i = 0; i < light_blobs.size() - 1; ++i) {
    int j = i + 1;
    Point2f center_i = light_blobs.at(i).rect.center;
    Point2f center_j = light_blobs.at(j).rect.center;
    Size2f size_i = light_blobs.at(i).rect.size;
    Size2f size_j = light_blobs.at(j).rect.size;
    float length_i =
        size_i.height > size_i.width ? size_i.height : size_i.width;
    float length_j =
        size_j.height > size_j.width ? size_j.height : size_j.width;
    // length similar
    // cout << "Twin length: " << length_i/length_j << endl;
    if (length_i / length_j > TWIN_LENGTH_RATIO_MAX ||
        length_j / length_i > TWIN_LENGTH_RATIO_MAX) {
      continue;
    }

    float anglei = light_blobs[i].angle;
    float anglej = light_blobs[j].angle;
    // cout << "light(i) angle:" << anglei
    //<<" light(j) angle" << anglej <<endl;
    if (abs(anglei - anglej) > 10.0) {
      continue;
    }
    float similarity =
        matchShapes(light_blobs[i].contour, light_blobs[j].contour,
                    CV_CONTOURS_MATCH_I2, 0);
    // cout << "Similar: " << similarity << endl;
    if (similarity > min_similarity) {
      continue;
    }

    float distance_n =
        abs((center_i.x - center_j.x) * cos((anglei + 90) * PI / 180) +
            (center_i.y - center_j.y) * sin((anglei + 90) * PI / 180));
    // normal distance range in about 1 ~ 2 times of length
    // cout << "Distance n: " << distance_n / length_i << endl;
    // add the large armor on hero, which should be 3 ~ 4 times of length. Maybe
    // negative influence on small armor detection.
    if (distance_n < TWIN_DISTANCE_N_MIN * length_i ||
        distance_n > 2 * TWIN_DISTANCE_N_MAX * length_i ||
        distance_n < TWIN_DISTANCE_N_MIN * length_j ||
        distance_n > 2 * TWIN_DISTANCE_N_MAX * length_j) {
      continue;
    }
    min_similarity = similarity;
    light_blob_i = i;
    light_blob_j = j;
    armor_type_ = SMALL_ARMOR;
  }

  if (light_blob_i == -1 || light_blob_j == -1) return false;
  // cout << "min i:" << light_blob_i << " j:" << light_blob_j << " angel:" <<
  // min_angle << endl; get and extend box for track init
  Rect2d rect_i = light_blobs.at(light_blob_i).rect.boundingRect();
  Rect2d rect_j = light_blobs.at(light_blob_j).rect.boundingRect();
  float min_x, min_y, max_x, max_y;
  if (rect_i.x < rect_j.x) {
    min_x = rect_i.x;
    max_x = rect_j.x + rect_j.width;
  } else {
    min_x = rect_j.x;
    max_x = rect_i.x + rect_i.width;
  }
  if (rect_i.y < rect_j.y) {
    min_y = rect_i.y;
    max_y = rect_j.y + rect_j.height;
  } else {
    min_y = rect_j.y;
    max_y = rect_i.y + rect_i.height;
  }
  min_x -= BOX_EXTRA;
  max_x += BOX_EXTRA;
  min_y -= BOX_EXTRA;
  max_y += BOX_EXTRA;
  if (min_x < 0 || max_x > src_width_ || min_y < 0 ||
      max_y > src_height_ - 20) {
    return false;
  }
  // check width > height for armor box
  if ((max_y - min_y) > (max_x - min_x)) {
    return false;
  }
  armor_box_ = Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
  // store for check in track later
  // if the remaining light area in the track box is lower than half of total
  // contour area, the tracking may be wrong.
  total_contour_area =
      light_blobs.at(light_blob_i).area + light_blobs.at(light_blob_j).area;
  return true;
}

bool Armor::slowExplore(Mat& src) {
  vector<LightBlob> light_blobs;
  if (slowSelectContours(src, light_blobs) == false) {
    return false;
  }
  if (slowPairContours(light_blobs) == false) {
    return false;
  }
  return true;
}

bool Armor::slowSelectContours(Mat& src, vector<LightBlob>& light_blobs) {
  static Mat bin;
  threshold(src, bin, GRAY_THRESH, 255, THRESH_BINARY);
#if DRAW == SHOW_ALL
  imshow("gray", bin);
#endif
  vector<vector<Point> > contours;
  findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  // select contours by area, length, width/height
  for (unsigned int i = 0; i < contours.size(); ++i) {
    float area = contourArea(contours.at(i));
    // cout << "area:" << area << endl;
    if (area > CONTOUR_AREA_MAX || area < CONTOUR_AREA_MIN) {
      continue;
    }
    RotatedRect rec = minAreaRect(contours.at(i));
    Size2f size = rec.size;
    // get length (longer) as length
    float length = size.height > size.width ? size.height : size.width;
    float width = size.height < size.width ? size.height : size.width;
    // cout << "length: " << length << endl;
    if (length < CONTOUR_LENGTH_MIN) {
      continue;
    }
    // check if it is thin
    if (length / width > SLOW_CONTOUR_HW_RATIO_MAX ||
        length / width < SLOW_CONTOUR_HW_RATIO_MIN) {
      continue;
    }
    // cout << "length / width: " << length / width << endl;

    // cout << "Area Ratio: " << area / size.area() << endl;
    if (area / size.area() < 0.7) continue;

    LeastSquare leasq(contours[i]);
    // cout << "LeastSquare: " << leasq.getAngle() << " | " << leasq.getAngleh()
    // << endl;
    float angle = leasq.getAngleh();
    // float angle = -rec.angle;
    // cout << "RotatedRect: " << angle << endl;
    if (size.width < size.height) angle += 90.0;
    if (angle > 90.0 + CONTOUR_ANGLE_MAX || angle < 90.0 - CONTOUR_ANGLE_MAX)
      continue;
    // cout << "push back" << endl;
    light_blobs.push_back(LightBlob(rec, contours[i], angle, area));
    // areas.push_back(area);
  }
  if (light_blobs.size() < 2) return false;
  return true;
}

bool Armor::slowPairContours(vector<LightBlob>& light_blobs) {
  int light_blob_i = -1, light_blob_j = -1;
  float min_angle = TWIN_ANGEL_MAX;
  float min_distance_n = 2 * SLOW_TWIN_DISTANCE_N_MAX;
  sort(light_blobs.begin(), light_blobs.end());
  // cout << "light_blobs: " << light_blobs.size() << endl;
  // pair light_blobs by length, distance, angel
  for (unsigned int i = 0; i < light_blobs.size() - 1; ++i) {
    int j = i + 1;
    Point2f center_i = light_blobs.at(i).rect.center;
    Point2f center_j = light_blobs.at(j).rect.center;
    Size2f size_i = light_blobs.at(i).rect.size;
    Size2f size_j = light_blobs.at(j).rect.size;
    float length_i =
        size_i.height > size_i.width ? size_i.height : size_i.width;
    float length_j =
        size_j.height > size_j.width ? size_j.height : size_j.width;
    // length similar
    if (length_i / length_j > SLOW_TWIN_LENGTH_RATIO_MAX ||
        length_j / length_i > SLOW_TWIN_LENGTH_RATIO_MAX)
      continue;
    // cout << "Twin length: " << length_i/length_j << endl;

    float anglei = light_blobs[i].angle;
    float anglej = light_blobs[j].angle;
    cout << "light(i) angle:" << anglei << " light(j) angle" << anglej << endl;
    if (anglei > 90 && 90 > anglej) {
      continue;
    }
    if (anglei < 90 && 90 < anglej) {
      continue;
    }
    if (abs(anglei - anglej) > min_angle) {
      continue;
    }

    float distance_n =
        abs((center_i.x - center_j.x) * cos((anglei + 90) * PI / 180) +
            (center_i.y - center_j.y) * sin((anglei + 90) * PI / 180));
    // normal distance range in about 1 ~ 2 times of length
    // add the large armor on hero, which should be 3 ~ 4 times of length. Maybe
    // negative influence on small armor detection.
    length_i = (length_i + length_j) / 2;
    // cout << "Distance n: " << distance_n / length_i << endl;
    if (distance_n > min_distance_n * length_i) {
      continue;
    }
    if (distance_n < SLOW_TWIN_DISTANCE_N_MIN * length_i) {
      continue;
    }
    if (distance_n > SLOW_TWIN_DISTANCE_N_MAX * length_i &&
        distance_n < 2 * SLOW_TWIN_DISTANCE_N_MIN * length_i) {
      continue;
    }
    if (distance_n > 2 * SLOW_TWIN_DISTANCE_N_MAX * length_i) {
      continue;
    }

    // direction distance should be small
    float distance_t = abs((center_i.x - center_j.x) * cos((anglei)*PI / 180) +
                           (center_i.y - center_j.y) * sin((anglei)*PI / 180));
    // cout << "Distance t: " << distance_t / length_i << endl;
    if (distance_t > TWIN_DISTANCE_T_MAX * length_i ||
        distance_t > TWIN_DISTANCE_T_MAX * length_j) {
      continue;
    }

    light_blob_i = i;
    light_blob_j = j;
    min_angle = abs(anglei - anglej);
    min_distance_n = distance_n / length_i;
    if (distance_n > 1.0 * TWIN_DISTANCE_N_MAX * length_i) {
      armor_type_ = LARGE_ARMOR;
      cout << "Hero!" << endl;
      // cout << "Distance n: " << distance_n / length_i << endl;
    } else {
      armor_type_ = SMALL_ARMOR;
      cout << "Infanity!" << endl;
    }
  }

  if (light_blob_i == -1 || light_blob_j == -1) return false;
  // cout << "min i:" << light_blob_i << " j:" << light_blob_j << " angel:" <<
  // min_angle << endl; get and extend box for track init
  Rect2d rect_i = light_blobs.at(light_blob_i).rect.boundingRect();
  Rect2d rect_j = light_blobs.at(light_blob_j).rect.boundingRect();
  float min_x, min_y, max_x, max_y;
  if (rect_i.x < rect_j.x) {
    min_x = rect_i.x;
    max_x = rect_j.x + rect_j.width;
  } else {
    min_x = rect_j.x;
    max_x = rect_i.x + rect_i.width;
  }
  if (rect_i.y < rect_j.y) {
    min_y = rect_i.y;
    max_y = rect_j.y + rect_j.height;
  } else {
    min_y = rect_j.y;
    max_y = rect_i.y + rect_i.height;
  }
  min_x -= BOX_EXTRA;
  max_x += BOX_EXTRA;
  min_y -= BOX_EXTRA;
  max_y += BOX_EXTRA;
  if (min_x < 0 || max_x > src_width_ || min_y < 0 || max_y > src_height_) {
    return false;
  }
  armor_box_ = Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
  total_contour_area =
      light_blobs.at(light_blob_i).area + light_blobs.at(light_blob_j).area;
  return true;
}

// use the raw date with Bayer format to extract blue region and red region.
void Armor::splitBayerBG(Mat& src, Mat& blue, Mat& red) {
  uchar* data;
  uchar* bayer_data[2];
  for (int i = 0; i < src.rows; ++i) {
    data = src.ptr<uchar>(i);
    bayer_data[0] = blue.ptr<uchar>(i / 2);
    for (int j = 0; j < blue.cols; ++j, data += 2) {
      bayer_data[0][j] = *data;
    }
    data = src.ptr<uchar>(++i) + 1;
    bayer_data[1] = red.ptr<uchar>(i / 2);
    for (int j = 0; j < red.cols; ++j, data += 2) {
      bayer_data[1][j] = *data;
    }
  }
}

void Armor::uartSendTarget(int x, int y, ArmorType armor_type) {
#if BAYER_HACK == HACKING_OFF
  uart_.sendTarget(x, y, armor_type_table_[armor_type]);
#elif BAYER_HACK == HACKING_ON
  uart_.sendTarget(2 * x, 2 * y, armor_type_table_[armor_type]);
#endif
}
