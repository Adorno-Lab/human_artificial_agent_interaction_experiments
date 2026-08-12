/*! -----------------------------------------------------------
 * Face features detection library using OpenCV.
 *
 * Instructions:
 * - Using QtCreator, insert the following lines in the .pro file:
 *      INCLUDEPATH += /usr/include/opencv
 *      LIBS += -L/usr/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_objdetect -lopencv_imgcodecs -lopencv_videoio -lopencv_video
 *      LD_FLAGS=$(OPENCV_LIBS)
 *
 * - Copy folder ./haarcascade into the same directory of the library files.
 *      Most of the files can be downloaded from https://github.com/opencv/opencv/tree/master/data/haarcascades.
 *      The mouth's file can be downloaded from https://github.com/opencv/opencv_contrib/tree/master/modules/face/data/cascades.
 *
 * Contributors:
 *      Ana Christina Almada Campos
 * ------------------------------------------------------------
 */

#include "detection.h"

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
 * \brief Loads training model and creates a cascade object related to it.
 *
 * \param file - A const char* with the path to the haarcascade file.
 * \return The corresponding CascadeClassifier object.
 */
CascadeClassifier load_training_models(string file){
  CascadeClassifier cascade_classifier;

  if(!cascade_classifier.load(file)){
    cerr << "Could not load detector" << endl;
  }

  return cascade_classifier;
}

/*!
 * \brief Draws a blue rectangle for detected face.
 *
 * \param frame - A Mat object where to draw the rectangle.
 * \param face - A Rect object with detected face.
 */
void face_rectangle(Mat frame, Rect face){
  rectangle(frame, face.tl(), face.br(), Scalar(255, 0, 0), 2);
}

/*!
 * \brief Draws a green rectangle for detected eye.
 *
 * \param frame - A Mat object where to draw the rectangle.
 * \param region - A Rect object with the region where the eye was found.
 * \param eye - A Rect object with detected eye in given region.
 */
void eye_rectangle(Mat frame, Rect region, Rect eye){
  rectangle(frame, region.tl() + eye.tl(), region.tl() + eye.br(), Scalar(0, 255, 0), 2);
}

/*!
 * \brief Draws a red rectangle for detected mouth.
 *
 * \param frame - A Mat object where to draw the rectangle.
 * \param region - A Rect object with the region where the mouth was found.
 * \param mouth - A Rect object with detected mouth in given region.
 */
void mouth_rectangle(Mat frame, Rect region, Rect mouth){
  rectangle(frame, region.tl() + mouth.tl(), region.tl() + mouth.br(), Scalar(0, 0, 255), 2);
}

/*!
 * \brief Detects faces in given frame using CascadeClassifier object.
 *
 * \param frame - A Mat object where to look for faces.
 * \param face_cascade - CascadeClassifier object related to face detection.
 * \return A vector of Rect objects with detected faces.
 *
 * Function detect_faces() manipulates the original image and uses
 * CascadeClassifier::detectMultiScale() to detect faces.
 */
vector<Rect> detect_faces(Mat frame, CascadeClassifier face_cascade){
  Mat grayscale;
  cvtColor(frame, grayscale, COLOR_BGR2GRAY);   // Converts image to grayscale.
  equalizeHist(grayscale, grayscale);        // Enhances image contrast.

  vector<Rect> faces;
  face_cascade.detectMultiScale(grayscale, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(150,150));

  return faces;
}

/*!
 * \brief Detects eyes in given frame and detected face using CascadeClassifier object.
 *
 * \param frame - A Mat object containing the original image for detection.
 * \param region - A Rect object with the region where to look for eyes.
 * \param eye_cascade - CascadeClassifier object related to eye detection.
 * \return A vector of Rect objects with detected eyes.
 *
 * Function detect_eyes() uses CascadeClassifier::detectMultiScale() to detect eyes.
 */
vector<Rect> detect_eyes(Mat frame, Rect region, CascadeClassifier eye_cascade){
  Mat face_region = frame(region);

  vector<Rect> eyes;
  eye_cascade.detectMultiScale(face_region, eyes, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(50,50));

  return eyes;
}

/*!
 * \brief Chooses two most probably correct eyes among all eyes detected.
 *
 * \param detected_eyes - A vector of Rect objects with all detected eyes.
 * \return A vector of Rect objects with the two most probably correct eyes.
 *
 * Function choose_eyes() gets all detected eyes and chooses the two of them
 * that have the least distance according to their vertical coordinates. These
 * are the eyes considered the most probably correct.
 */
vector<Rect> choose_eyes(vector<Rect> detected_eyes){
  vector<Rect> most_probable_eyes;
  Rect most_probable_eye1, most_probable_eye2;
  double min_distance_y = 999999;

  // Separate in possible left and right eyes.
  /*vector<Rect> left_eyes, right_eyes;
  if (!detected_eyes.empty()) {
    for (Rect &eye : detected_eyes) {
      if (abs(eye.tl().x - face.tl().x) < abs(eye.br().x - face.br().x)) {
        left_eyes.push_back(eye);
      } else {
        right_eyes.push_back(eye);
      }
    }
  }

  if (!left_eyes.empty() && !right_eyes.empty()) {
    for (Rect &left : left_eyes) {
      for (Rect &right : right_eyes) {
        if (norm(left.tl().y - right.tl().y) < min_distance_y) {
          min_distance_y = norm(left.tl().y - right.tl().y);
          most_probable_eye1 = left;
          most_probable_eye2 = right;
        }
      }
    }
  }*/

  if (!detected_eyes.empty()) {
    for (Rect &eye1 : detected_eyes) {
      for (Rect &eye2 : detected_eyes) {
        if (eye1 != eye2) {
          if (norm(eye1.tl().y - eye2.tl().y) < min_distance_y) {
            min_distance_y = norm(eye1.tl().y - eye2.tl().y);
            most_probable_eye1 = eye1;
            most_probable_eye2 = eye2;
          }
        }
      }
    }
  }

  most_probable_eyes.push_back(most_probable_eye1);
  most_probable_eyes.push_back(most_probable_eye2);

  return most_probable_eyes;
}

/*!
 * \brief Detects mouth in given frame and detected face using CascadeClassifier object.
 *
 * \param frame - A Mat object containing the original image for detection.
 * \param region - A Rect object with the region where to look for mouths.
 * \param mouth_cascade - CascadeClassifier object related to mouth detection.
 * \return A vector of Rect objects with detected mouths.
 *
 * Function detect_mouth() uses CascadeClassifier::detectMultiScale() to detect mouths.
 */
vector<Rect> detect_mouth(Mat frame, Rect region, CascadeClassifier mouth_cascade){
  Mat face_region = frame(region);
  vector<Rect> mouth;
  mouth_cascade.detectMultiScale(face_region, mouth, 1.2, 22, (25, 25));
  //mouthCascade.detectMultiScale(face_region, mouth, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(50,50));

  return mouth;
}

/*!
 * \brief Chooses one most probably correct mouth among all mouths detected.
 *
 * \param eyes - A vector of Rect objects with chosen eyes.
 * \param detected_mouth - A vector of Rect objects with all detected mouths.
 * \return A Rect object with the most probably correct mouth.
 *
 * Function choose_mouth() gets all detected mouths and chooses the one that it has
 * the greatest distance from the chosen eyes according to their vertical coordinates.
 * This is the mouth considered the most probably correct.
 */
Rect choose_mouth(vector<Rect> eyes, vector<Rect> detected_mouths){
  Rect mouth;
  double max_distance_y = -999999;

  if (!detected_mouths.empty()) {
    for (Rect &m : detected_mouths) {
      if (norm(m.br().y - eyes[0].tl().y) > max_distance_y) {
        max_distance_y = norm(m.br().y - eyes[0].tl().y);
        mouth = m;
      }
    }
  }

  return mouth;
}

/*!
 * \brief Calculates the threshold for gradients that will be considered.
 *
 * \param gradient_x - A Mat object with gradient in coordinate x for each pixel.
 * \param gradient_y - A Mat object with gradient in coordinate y for each pixel.
 * \return A double value with the minimum value for the gradient.
 *
 * In order to decrease the number of points used in the sum for the objective function
 * for each possible center, only gradient values bigger than a threshold will be considered,
 * ignoring gradient in homogeneous regions. Function gradient_threshold() gets the maximum
 * gradient value and the threshold will be 90% of it.
 */
double gradient_threshold(Mat gradient_x, Mat gradient_y){
  double max_gradient = -999999;
  double gradient_norm;

  for (int i = 0; i < gradient_x.rows; i++) {
    for (int j = 0; j < gradient_x.cols; j++) {
      cv::Scalar grad_x = gradient_x.at<short>(i,j);
      cv::Scalar grad_y = gradient_y.at<short>(i,j);
      gradient_norm = sqrt(pow(grad_x.val[0],2) + pow(grad_y.val[0],2));
      if (gradient_norm > max_gradient)
        max_gradient = gradient_norm;
    }
  }
  return 0.9*max_gradient;
}

/*!
 * \brief Calculates the sum for the objective function.
 *
 * \param possible_center_x - A int coordinate x of the possible center analyzed.
 * \param possible_center_y - A int coordinate y of the possible center analyzed.
 * \param gradient_x - A Mat object with gradient in coordinate x for each pixel.
 * \param gradient_y - A Mat object with gradient in coordinate y for each pixel.
 * \param weight - A Mat object with the inverted gray values for each pixel.
 * \param grad_threshold - A double value for gradient threshold.
 * \return A double value for the sum.
 */
double objective_sum(int possible_center_x, int possible_center_y, Mat gradient_x, Mat gradient_y, Mat weight, double grad_threshold){
  double total_sum = 0, auxiliary_norm = 0;
  double distance_x = 0, distance_y = 0;
  Scalar w = 0;

  for (int i = 0; i < gradient_x.rows; i++) {
    for (int j = 0; j < gradient_x.cols; j++) {
      distance_y = i - possible_center_y;
      distance_x = j - possible_center_x;
      auxiliary_norm = sqrt(pow(distance_x,2) + pow(distance_y,2));
      if (auxiliary_norm != 0.0) {
        distance_x = distance_x/auxiliary_norm;
        distance_y = distance_y/auxiliary_norm;
      }
      cv::Scalar grad_x = gradient_x.at<short>(i,j);
      cv::Scalar grad_y = gradient_y.at<short>(i,j);
      auxiliary_norm = sqrt(pow(grad_x.val[0],2) + pow(grad_y.val[0],2));
      if (auxiliary_norm != 0.0) {
        if (auxiliary_norm > grad_threshold) {
          grad_x = grad_x.val[0]/auxiliary_norm;
          grad_y = grad_y.val[0]/auxiliary_norm;

          w = weight.at<uchar>(possible_center_y, possible_center_x);
          total_sum = total_sum + w.val[0]*(pow(distance_x*grad_x.val[0] + distance_y*grad_y.val[0],2));
        }
      }
    }
  }

  return total_sum;
}

/*!
 * \brief Implements the objective function of the method for eye center localization.
 *
 * \param weight - A Mat object with the weights for each pixel.
 * \param gradient_x - A Mat object with gradient in coordinate x for each pixel.
 * \param gradient_y - A Mat object with gradient in coordinate y for each pixel.
 * \param grad_threshold - A double value for gradient threshold.
 * \return A Point with the detected center of the eye.
 */
Point objective_function(Mat weight, Mat gradient_x, Mat gradient_y, double grad_threshold){
  double max_sum = 0, total_sum = 0;
  int center_x = 0, center_y = 0;
  int n_cols = gradient_x.cols;
  int n_rows = gradient_x.rows;
  int N = n_rows*n_cols;

  /* Only possible centers that are not in any of the borders are considered. */
  for (int possible_center_y = 1; possible_center_y < n_rows-1; possible_center_y++) {
    for (int possible_center_x = 1; possible_center_x < n_cols-1; possible_center_x++) {
      total_sum = objective_sum(possible_center_x,possible_center_y,gradient_x,gradient_y,weight,grad_threshold);
      total_sum = total_sum/N;
      if (total_sum > max_sum) {
        center_x = possible_center_x;
        center_y = possible_center_y;
        max_sum = total_sum;
      }
    }
  }
  Point center(center_x, center_y);

  return center;
}

/*!
 * \brief Finds the center of the eye using Timm and Barth's (2011) method.
 *
 * \param region - A Mat object with the region where the eye was found.
 * \param eye - A Rect object with an eye from the face.
 * \return A Point with the detected center of the eye.
 *
 * Function locate_eye_center() gets the center of the eye using the method
 * proposed in the following work:
 *
 * F. Timm and E. Barth, “Accurate Eye Centre Localisation by Means of
 * Gradients,” in Proceedings of the International Conference on Computer
 * Vision Theory and Applications. SciTePress - Science and and Technology
 * Publications, 2011, pp. 125–130.
 */
Point locate_eye_center(Mat region, Rect eye){
  Mat filtered, weight;
  double grad_threshold;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  Mat gradient_x, gradient_y;
  Point center;

  // Filters the image.
  GaussianBlur(region(eye), filtered, Size(3,3), 0, 0, BORDER_DEFAULT);
  cvtColor(filtered, filtered, COLOR_BGR2GRAY);

  // Gets gradient values for each pixel from the region of the eye.
  Sobel(filtered, gradient_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  Sobel(filtered, gradient_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

  // Gets the center of the eye.
  weight = ~filtered; // Smoothed and inverted image for the weights.
  grad_threshold = gradient_threshold(gradient_x,gradient_y); // Gradient threshold.
  center = objective_function(weight,gradient_x,gradient_y,grad_threshold);

  return center;
}

