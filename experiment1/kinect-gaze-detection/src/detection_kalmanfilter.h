#ifndef DETECTION_KALMANFILTER_H_
#define DETECTION_KALMANFILTER_H_

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;

Rect set_initial_rect(vector<Rect> first_detected);

Point set_initial_point(vector<Point> first_detected);

void set_kalman_filter_matrices(KalmanFilter &kf_object, double delta_t, int matrices_values[3]);

void rect_initialization(KalmanFilter &kf_rect, Rect initial, int length_x, int length_y);

void point_initialization(KalmanFilter &kf_point, Point initial);

Rect create_search_region(KalmanFilter &kf_object, int length_x, int length_y, double factor, Point frame_br);

#endif // DETECTION_KALMANFILTER_H_
