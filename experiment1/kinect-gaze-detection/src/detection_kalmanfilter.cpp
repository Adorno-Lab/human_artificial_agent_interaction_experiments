#include "detection_kalmanfilter.h"

#include <iostream>
#include <numeric>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;

/*!
 * \brief Calculates the mean of the first face features detected.
 *
 * \param first_detected - A vector with the first Rect detected.
 * \return A Rect for initialization of the Kalman filter.
 *
 * Function set_initial_rect() receives the first Rect detected and
 * calculates the mean of them to get the initial Rect. The result Rect is
 * used to initialize the Kalman filter.
 */
Rect set_initial_rect(vector<Rect> first_detected){
    double tlx = 0, tly = 0, brx = 0, bry = 0;

    for (Rect &element : first_detected){
      tlx = tlx + element.tl().x;
      tly = tly + element.tl().y;
      brx = brx + element.br().x;
      bry = bry + element.br().y;
    }
    tlx = tlx/first_detected.size();
    tly = tly/first_detected.size();
    brx = brx/first_detected.size();
    bry = bry/first_detected.size();

    Point point_tl(static_cast<int>(tlx),static_cast<int>(tly));
    Point point_br(static_cast<int>(brx),static_cast<int>(bry));

    Rect initial_rect(point_tl, point_br);

    return initial_rect;
}

/*!
 * \brief Calculates the mean of the first eye center points detected.
 *
 * \param first_detected - A vector with the first Point detected.
 * \return A Point for initialization of the Kalman filter.
 *
 * Function set_initial_point() receives the first Point detected and
 * calculates the mean of them to get the initial Point. The result Point is
 * used to initialize the Kalman filter.
 */
Point set_initial_point(vector<Point> first_detected){
    double point_x = 0, point_y = 0;

    for (Point &element : first_detected){
        point_x = point_x + element.x;
        point_y = point_y + element.y;
    }
    point_x = round(point_x/first_detected.size());
    point_y = round(point_y/first_detected.size());

    Point initial_point(static_cast<int>(point_x), static_cast<int>(point_y));

    return initial_point;
}

/*!
 * \brief Sets Kalman filter matrices.
 *
 * \param kf_object - A pointer to the Kalman filter object.
 * \param delta_t - Sampling interval.
 * \param matrices_values - An array with the values for initialization of the
 *        filters matrices [kProcessNoise, kMeasurementNoise, kErrorPost].
 *
 * Function set_kalman_filter_matrices sets the following matrices for the filter:
 *      Transition matrix;
 *      Measurement matrix;
 *      Process noise covariance matrix;
 *      Measurement noise covariance matrix;
 *      Initial posteriori error estimate covariance matrix.
 * Note that if kProcessNoise > kMeasurementNoise, then the measurement is more
 * reliable than the model.
 */
void set_kalman_filter_matrices(KalmanFilter &kf_object, double delta_t, int matrices_values[3]){
    /* Sets transition matrix as: [1 delta_t 0    0;
     *                             0   1     0    0;
     *                             0   0     1 delta_t;
     *                             0   0     0    1]. */
    kf_object.transitionMatrix = (Mat_<float>(4, 4) << 1, delta_t, 0, 0,
                                                       0, 1, 0, 0,
                                                       0, 0, 1, delta_t,
                                                       0, 0, 0, 1);

    // Sets measurement matrix as: [1 0 0 0; 0 0 1 0].
    kf_object.measurementMatrix = (Mat_<float>(2, 4) << 1, 0, 0, 0,
                                                        0, 0, 1, 0);

    // Sets process noise covariance matrix as kProcessNoise*I, where I is the identity matrix.
    int kProcessNoise = matrices_values[0];
    setIdentity(kf_object.processNoiseCov, Scalar::all(kProcessNoise));

    // Sets measurement noise covariance matrix as kMeasurementNoise*I, where I is the identity matrix.
    int kMeasurementNoise = matrices_values[1];
    setIdentity(kf_object.measurementNoiseCov, Scalar::all(kMeasurementNoise));

    // Sets initial posteriori error estimate covariance matrix as kErrorPost*I, where I is the identity matrix.
    int kErrorPost = matrices_values[2];
    setIdentity(kf_object.errorCovPost, Scalar::all(kErrorPost));

}

/*!
 * \brief Sets the initial states for the Kalman filter of Rect objects.
 *
 * \param &kf_object - A pointer to the Kalman filter object.
 * \param initial - A Rect object with the initial rectangle.
 * \param length_x - An int for the length in x coordinate.
 * \param length_y - An int for the lenght in y coordinate.
 *
 * Function rect_initialization() is used to set the initial states of the eyes and mouth,
 * that are given by Rect objects. The points passed to the filter are the centers of the
 * rectangles and the velocities are set as zero.
 */
void rect_initialization(KalmanFilter &kf_object, Rect initial, int length_x, int length_y){
    // The statePost parameter is set because the first step of the filter will be prediction.
    kf_object.statePost.at<float>(0) = initial.tl().x + static_cast<int>(length_x/2);
    kf_object.statePost.at<float>(1) = 0;
    kf_object.statePost.at<float>(2) = initial.tl().y + static_cast<int>(length_y/2);
    kf_object.statePost.at<float>(3) = 0;
}

/*!
 * \brief Sets the initial states for the Kalman filter of Point objects.
 *
 * \param &kf_object - A pointer to the Kalman filter object.
 * \param initial - A Point object with the initial point.
 *
 * Function point_initialization() is used to set the initial states of the eye centers,
 * that are given by Point objects. The velocities are set as zero.
 */
void point_initialization(KalmanFilter &kf_point, Point initial){
    kf_point.statePost.at<float>(0) = initial.x;
    kf_point.statePost.at<float>(1) = 0;
    kf_point.statePost.at<float>(2) = initial.y;
    kf_point.statePost.at<float>(3) = 0;
}

/*!
 * \brief Creates the next search region for detection of face features.
 *
 * \param kf_object - A pointer to the Kalman filter object.
 * \param length_x - An int for the length in x coordinate.
 * \param length_y - An int for the length in y coordinate.
 * \param factor - A double for the increase factor of the search region related to the last detected rectangle.
 * \return A Rect object with the next search region.
 */
Rect create_search_region(KalmanFilter &kf_object, int length_x, int length_y, double factor, Point frame_br){
    Point point_tl, point_br;
    point_tl.x = static_cast<int>(kf_object.statePre.at<float>(0) - factor*length_x/2);
    point_tl.y = static_cast<int>(kf_object.statePre.at<float>(2) - factor*length_y/2);
    point_br.x = static_cast<int>(kf_object.statePre.at<float>(0) + factor*length_x/2);
    point_br.y = static_cast<int>(kf_object.statePre.at<float>(2) + factor*length_y/2);

    // Keeping the search regions inside the image frame.
    if (point_tl.x < 0) {
      int shift = point_br.x - point_tl.x;
      point_tl.x = 0;
      point_br.x = shift;
    }
    if (point_tl.y < 0) {
      int shift = point_br.y - point_tl.y;
      point_tl.y = 0;
      point_br.y = shift;
    }
    if (point_br.x > frame_br.x) {
      int shift = point_br.x - point_tl.x;
      point_br.x = frame_br.x;
      point_tl.x = frame_br.x - shift;
    }
    if (point_br.y > frame_br.y) {
      int shift = point_br.y - point_tl.y;
      point_br.y = frame_br.y;
      point_tl.y = frame_br.y - shift;
    }


    Rect region(point_tl, point_br);

    return region;
}
