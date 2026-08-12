/**
This is the main class for detection and tracking of facial attributes.

Contributors:
- Ana Christina Almada Campos (anachristinaac@gmail.com)
*/

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>

#include "detection.h"
#include "detection_kalmanfilter.h"

/**
 * \brief The Detector class.
 *
 * This class is responsible for the detection and tracking of face
 * attributes, the eyes and mouth. The methods used here are implemented in
 * detection.cpp and detection_kalmanfilter.cpp.
 */
class Detector {
private:
  // Classifiers for detection.
  CascadeClassifier face_cascade;
  CascadeClassifier profile_cascade;
  CascadeClassifier eye_cascade;
  CascadeClassifier mouth_cascade;

  // Vectors of Rect objects for the detected faces, eyes and mouths.
  vector<Rect> faces, eyes, mouths;
  // Vector os Rect objects for the two eyes chosen among all the ones detected.
  vector<Rect> chosen_eyes;
  // Rect object for the mouth chosen among all the ones detected.
  Rect chosen_mouth;

  // Number of centers that will be stored for the moving average.
  // The moving average will use the stored values and the new one obtained.
  static const int kNumCenters = 4;
  // Counters for the number of centers that already had been stored.
  int countlast_lefteye = 0;
  int countlast_righteye = 0;

  // Arrays of Point to store the last center points for each of the eyes.
  Point lastcenters_lefteye[kNumCenters];
  Point lastcenters_righteye[kNumCenters];

  // Point for the centers of the pupils after the moving average.
  Point center;

  int count_eye = 0;

  // Counter to initialize Kalman filters. KF is initialized when eyes and mouth
  // are detected at the same time. If this happens, flag_detection = 2;
  int flag_detection = 0;

  // Lengths in x and y coordinates of the eyes' and mouth' rectangles.
  int lengthx_lefteye = 0, lengthy_lefteye = 0;
  int lengthx_righteye = 0, lengthy_righteye = 0;
  int lengthx_mouth = 0, lengthy_mouth = 0;

  // Counts the number of failures in detection of mouth and eyes.
  int countfail_mouth = 0, countfail_lefteye = 0, countfail_righteye = 0;
  int const kMaxFailures = 5;

  // Counts the number of failures and the number of detections of faces.
  int countfail_face = 0;
  int countdetected_face = 0;
  int const kMaxFace = 10;
  //int const kMaxFace = (1/kDeltaT)*60;

  // A flag to indicates if a face is being detected or not.
  int face_flag = 0;

  // A flag to indicate if the result should be shown or not.
  int show;

  // A flag to indicate if the detection shoud run (1) or not (0).
  int running;

  // Number of frames that should be used in initialization.
  int initialization_frames;

  // Counts the number of frames already used in initialization.
  int count_init_frame = 0;

public:
  // Points for the references and measurements of pupils and mouth.
  Point ref_left;
  Point ref_right;
  Point center_left;
  Point center_right;
  Point ref_mouth;
  Point point_mouth;

  // Sampling interval.
  double kDeltaT;

  // Values for initialization of the posteriori error estimate covariance matrices.
  double kPostError_leye;
  double kPostError_reye;
  double kPostError_mouth;
  double kPostError_lpupil;
  double kPostError_rpupil;

  /** Kalman filters for each of the face features.
   * States are [x vx y vy], where:
   *  x = x coordinate of the center of the rectangle for eyes and mouth
   *      x coordinate for the pupil centers
   *  vx = velocity of the x coordinate of the center of the rectangle for eyes and mouth
   *       velocity of the x coordinate for the pupil centers
   *  y = y coordinate of the center of the rectangle for eyes and mouth
   *      y coordinate for the pupil centers
   *  vy = velocity of the y coordinate of the center of the rectangle for eyes and mouth
   *       velocity of the y coordinate for the pupil centers
   *
   * Four dynamic parameters, two measurement parameters and zero control parameters.
   */
  KalmanFilter kf_lefteye{KalmanFilter(4,2,0)};
  KalmanFilter kf_righteye{KalmanFilter(4,2,0)};
  KalmanFilter kf_leftpupil{KalmanFilter(4,2,0)};
  KalmanFilter kf_rightpupil{KalmanFilter(4,2,0)};
  KalmanFilter kf_mouth{KalmanFilter(4,2,0)};

  // Publishers for the detected points and the face flag.
  ros::Publisher ref_left_pub;
  ros::Publisher ref_right_pub;
  ros::Publisher left_eye_pub;
  ros::Publisher right_eye_pub;
  ros::Publisher ref_mouth_pub;
  ros::Publisher mouth_pub;
  ros::Publisher face_pub;

  // Subscriber to get commands for the application.
  ros::Subscriber gaze_commands_sub;

  /**
   * \brief Callback function to get commands for the application.
   * \param msg - A message of the type std_msgs::Int16::ConstPtr.
   *
   * When a message arrives, updates the running flag.
   */
  void human_gaze_callback(const std_msgs::Int16::ConstPtr& msg) {
    running = msg->data;
  }

  /**
   * \brief Callback function of the Kinect image subscriber.
   * \param msg - A message of the type sensor_msgs::ImageConstPtr.
   *
   * When a new frame is received, we convert the ROS image to an image format
   * that works for OpenCV. This is done by creating a copy of the image, that
   * can be edited.
   */
  void image_callback (const sensor_msgs::ImageConstPtr& msg) {
    // After the conversion, cv_ptr->image will contain the Mat object.
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      Mat frame = cv_ptr->image;
    detection(frame);
    //detection(frame, 0);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  /**
   * \brief Sets geometry_msgs::Point message with the point p.
   * \param p - Point to be sent in the message
   * \return The message to be sent.
   */
  geometry_msgs::Point set_msg (Point p) {
    geometry_msgs::Point msg;

    msg.x = p.x;
    msg.y = p.y;
    msg.z = 0;

    return msg;
    }

  /**
   * \brief Function that tries to initialize the Kalman filters.
   * \param frame - A Mat object with the frame.
   *
   * The Kalman filters are initialized and the tracking starts after mouth and
   * two different eyes are detected in the same frame.
   */
  void initialization(Mat frame) {
    // Look for the features in the whole face.
    if (!faces.empty()) {
      for (Rect &face : faces) {
        eyes = detect_eyes(frame,face,eye_cascade);

        if (eyes.size() >= 2) {
          // If more than two eyes are detected, calls the function to choose
          // the most probably correct eyes.
          chosen_eyes.clear();
          if (eyes.size() > 2) {
            chosen_eyes = choose_eyes(eyes);
          } else {
            chosen_eyes = eyes;
          }

          // Check if there are two different eyes or not.
          if (abs(chosen_eyes[0].tl().x - chosen_eyes[1].tl().x) < face.width/4) {
            // If the two eyes are probably the same, return without initialization.
            flag_detection = 0;
            return;
          } else {
            // If there are two probably different eyes, continue.
            flag_detection = 1;
          }

          // If there are two different eyes, then searches for the mouth.
          mouths = detect_mouth(frame,face,mouth_cascade);

          // The filters are initialized only if eyes and mouth are detected.
          if (mouths.empty()) {
            flag_detection = 0;
          } else {
            flag_detection = 2;
            count_init_frame = count_init_frame + 1;

            // Calls the function to choose the most probably correct mouth.
            chosen_mouth = choose_mouth(chosen_eyes,mouths);

            // Points in absolute coordinates (related to the tl() point of the frame).
            Rect mouth_rect(face.tl() + chosen_mouth.tl(),face.tl() + chosen_mouth.br());
            Rect lefteye_rect, righteye_rect;

            // Save the first measurement of mouth to be used as reference.
            ref_mouth.x = mouth_rect.tl().x + chosen_mouth.width/2;
            ref_mouth.y = mouth_rect.tl().y + chosen_mouth.height/2;

            point_mouth.x = ref_mouth.x;
            point_mouth.y = ref_mouth.y;

            // Defines left and right eye.
            if (chosen_eyes[0].tl().x <= chosen_eyes[1].tl().x) {
              Rect auxiliary_left(face.tl() + chosen_eyes[0].tl(),face.tl() + chosen_eyes[0].br());
              lefteye_rect = auxiliary_left;
              Rect auxiliary_right(face.tl() + chosen_eyes[1].tl(),face.tl() + chosen_eyes[1].br());
              righteye_rect = auxiliary_right;
            } else {
              Rect auxiliary_left(face.tl() + chosen_eyes[1].tl(),face.tl() + chosen_eyes[1].br());
              lefteye_rect = auxiliary_left;
              Rect auxiliary_right(face.tl() + chosen_eyes[0].tl(),face.tl() + chosen_eyes[0].br());
              righteye_rect = auxiliary_right;
            }

            // Getting the lengths of the rectangles for mouth and eyes.
            lengthx_lefteye = lefteye_rect.width;
            lengthy_lefteye = lefteye_rect.height;
            lengthx_righteye = righteye_rect.width;
            lengthy_righteye = righteye_rect.height;
            lengthx_mouth = chosen_mouth.width;
            lengthy_mouth = chosen_mouth.height;

            // Setting the initial states for the Kalman filters.
            rect_initialization(kf_lefteye, lefteye_rect, lengthx_lefteye, lengthy_lefteye);
            rect_initialization(kf_righteye, righteye_rect, lengthx_righteye, lengthy_righteye);
            rect_initialization(kf_mouth, mouth_rect, lengthx_mouth, lengthy_mouth);

            // Left pupil detection.
            Rect eye;
            if (chosen_eyes[0].tl().x <= chosen_eyes[1].tl().x) {
              eye = chosen_eyes[0];
              count_eye = 1;
            } else {
              eye = chosen_eyes[1];
              count_eye = 0;
            }

            //if (show == 1)
              //eye_rectangle(frame, face, eye);
            Mat fc = frame(face);
            Point center_new = locate_eye_center(fc,eye);

            Point sum_center = center_new;
            for (int j = 0; j < kNumCenters; j++)
              sum_center = sum_center + lastcenters_lefteye[j];

            center.x = sum_center.x/(kNumCenters+1);
            center.y = sum_center.y/(kNumCenters+1);

            center_left = face.tl() + eye.tl() + center;

            //if (show == 1)
              //drawMarker(frame, center_left, Scalar(255, 255, 255), MARKER_CROSS, 10, 1);

            if (countlast_lefteye < kNumCenters) {
              lastcenters_lefteye[countlast_lefteye] = center_new;
              countlast_lefteye = countlast_lefteye + 1;
            } else {
              for (int k = 0; k < kNumCenters-1; k++)
                lastcenters_lefteye[k] = lastcenters_lefteye[k+1];

              lastcenters_lefteye[kNumCenters-1] = center_new;
            }

            // Setting the initial state for the Kalman filter.
            point_initialization(kf_leftpupil, center_left);

            // Save the first measurement to be used as reference.
            ref_left = center_left;

            // Right pupil detection.
            eye = chosen_eyes[count_eye];
            //if (show == 1)
              //eye_rectangle(frame, face, eye);
            fc = frame(face);
            center_new = locate_eye_center(fc,eye);

            sum_center = center_new;
            for (int j = 0; j < kNumCenters; j++)
              sum_center = sum_center + lastcenters_righteye[j];

            center.x = sum_center.x/(kNumCenters+1);
            center.y = sum_center.y/(kNumCenters+1);

            center_right = face.tl() + eye.tl() + center;

            //if (show == 1)
              //drawMarker(frame, center_right, Scalar(255, 255, 255), MARKER_CROSS, 10, 1);

            if (countlast_righteye < kNumCenters) {
              lastcenters_righteye[countlast_righteye] = center_new;
              countlast_righteye = countlast_righteye + 1;
            } else {
              for (int k = 0; k < kNumCenters-1; k++)
                lastcenters_righteye[k] = lastcenters_righteye[k+1];

              lastcenters_righteye[kNumCenters-1] = center_new;
            }

            // Setting the initial state for the Kalman filter.
            point_initialization(kf_rightpupil, center_right);

            // Save the first measurement to be used as reference.
            ref_right = center_right;

          }
        }
      }
    }
  }

  /**
   * \brief Function for detection and tracking of the face features.
   * \param frame - A Mat object with the frame.
   */
  void detection (Mat frame) {
    Point frame_br(frame.cols, frame.rows);
    Mat print = frame.clone();

    // Flag to indicate if the face was detected with frontal face file (0) or
    // the profile face file (1).
    int face_pose = 0;

    faces = detect_faces(frame,face_cascade);
    if (faces.empty()) {
      face_pose = 1;
      faces = detect_faces(frame,profile_cascade);
    }

    if (faces.empty()) {
      // If faces are not detected kMaxFace times, restart the filters and flags.
      countfail_face = countfail_face + 1;

      if (countfail_face >= kMaxFace){
        countdetected_face = 0;
        face_flag = 0;
        flag_detection = 0;
        count_init_frame = 0;

        setIdentity(kf_lefteye.errorCovPost, Scalar::all(kPostError_leye));
        setIdentity(kf_righteye.errorCovPost, Scalar::all(kPostError_reye));
        setIdentity(kf_mouth.errorCovPost, Scalar::all(kPostError_mouth));
        setIdentity(kf_leftpupil.errorCovPost, Scalar::all(kPostError_lpupil));
        setIdentity(kf_rightpupil.errorCovPost, Scalar::all(kPostError_rpupil));
      }
    } else {
      // If faces are detected kMaxFace times, update flags.
      countdetected_face = countdetected_face + 1;

      if (countdetected_face >= kMaxFace) {
        countfail_face = 0;
        face_flag = 1;
      }
    }

    std_msgs::Int16 msg;
    msg.data = face_flag;
    face_pub.publish(msg);

    if (running == 0) {
        flag_detection = 0;
        count_init_frame = 0;
    }

    if (flag_detection < 2) {
      // If no feature was detected yet, call the function to try initialization.
      // Initialization will be done only with a face detected using the frontal face file.
      if (face_pose == 0 and running == 1) {
        initialization(frame);
      }
    } else {
      // If flag_detection == 2, it means that the filters already started.

      // Kalman filter prediction step.
      kf_lefteye.predict();
      kf_righteye.predict();
      kf_mouth.predict();
      kf_leftpupil.predict();
      kf_rightpupil.predict();

      // Creates new regions where to look for the features in measurement step.
      Rect region_lefteye = create_search_region(kf_lefteye, lengthx_lefteye, lengthy_lefteye, 1.5, frame_br);
      if (show == 1)
        rectangle(print, region_lefteye, Scalar(0, 255, 255), 2);
      Rect region_righteye = create_search_region(kf_righteye, lengthx_righteye, lengthy_righteye, 1.5, frame_br);
      if (show == 1)
        rectangle(print, region_righteye, Scalar(0, 255, 255), 2);
      Rect mouth_region = create_search_region(kf_mouth, lengthx_mouth, lengthy_mouth, 2, frame_br);
      if (show == 1)
        rectangle(print, mouth_region, Scalar(0, 255, 255), 2);

      // Gets measurements.
      // Calls Kalman filter update step only if there is valid measurement.
      Mat_<float> measurement(2,1);

      // Auxiliary variables to define if current measurement could be the other eye.
      int len_x, same_eyes;
      float ms_x;

      // Left eye measurement.
      vector<Rect> measurement_lefteye = detect_eyes(frame,region_lefteye,eye_cascade);

      // Checks if the current measurement could be the other eye.
      same_eyes = 0;
      if (!measurement_lefteye.empty()) {
        len_x = measurement_lefteye[0].br().x - measurement_lefteye[0].tl().x;
        ms_x = region_lefteye.tl().x + measurement_lefteye[0].tl().x + len_x/2;
        if (abs(ms_x - static_cast<int>(kf_righteye.statePost.at<float>(0))) <= len_x/3) {
          // If the detected eye is too close horizontally to the other one,
          // they are probably the same.
          same_eyes = 1;
        }
      }

      if (!measurement_lefteye.empty() & same_eyes == 0) {
        if (show == 1)
          eye_rectangle(print,region_lefteye,measurement_lefteye[0]);

        lengthx_lefteye = measurement_lefteye[0].br().x - measurement_lefteye[0].tl().x;
        lengthy_lefteye = measurement_lefteye[0].br().y - measurement_lefteye[0].tl().y;

        measurement(0) = region_lefteye.tl().x + measurement_lefteye[0].tl().x + lengthx_lefteye/2;
        measurement(1) = region_lefteye.tl().y + measurement_lefteye[0].tl().y + lengthy_lefteye/2;
        kf_lefteye.correct(measurement);
        countfail_lefteye = 0;

      } else {
        countfail_lefteye = countfail_lefteye + 1;

        /* If the maximum number of detection failures allowed in consecutive
         * frames is reached, initialize the Kalman filter with the detection
         * of the face again. */
        if (countfail_lefteye  >= kMaxFailures) {
          //set_kalman_filter_matrices(kf_eye1, kDeltaT, 100, 5, .1);
          setIdentity(kf_lefteye.errorCovPost, Scalar::all(kPostError_leye));

          faces = detect_faces(frame,face_cascade);
          if (faces.empty()) {
            faces = detect_faces(frame,profile_cascade);
          }

          if (!faces.empty()) {
            for (Rect &face : faces) {
              if (show == 1)
                face_rectangle(print,face);
              Point point_tl, point_br;
              point_tl = face.tl();
              point_br.x = face.tl().x + 2*face.width/3;
              point_br.y = face.tl().y + face.height;
              Rect region(point_tl,point_br);

              measurement_lefteye = detect_eyes(frame,region,eye_cascade);

              // Checks if the current measurement could be the other eye.
              same_eyes = 0;
              if (!measurement_lefteye.empty()) {
                len_x = measurement_lefteye[0].br().x - measurement_lefteye[0].tl().x;
                ms_x = region_lefteye.tl().x + measurement_lefteye[0].tl().x + len_x/2;
                if (abs(ms_x - static_cast<int>(kf_righteye.statePost.at<float>(0))) <= len_x/3) {
                  // If the detected eye is too close horizontally to the other one,
                  // they are probably the same.
                  same_eyes = 1;
                }
              }

              if (!measurement_lefteye.empty() & same_eyes == 0) {
                // If the current measurement is probably the other eye, ignores it.
                if (show == 1)
                  eye_rectangle(print,region,measurement_lefteye[0]);

                lengthx_lefteye = measurement_lefteye[0].br().x - measurement_lefteye[0].tl().x;
                lengthy_lefteye = measurement_lefteye[0].br().y - measurement_lefteye[0].tl().y;

                measurement(0) = region.tl().x + measurement_lefteye[0].tl().x + lengthx_lefteye/2;
                measurement(1) = region.tl().y + measurement_lefteye[0].tl().y + lengthy_lefteye/2;
                kf_lefteye.correct(measurement);
                countfail_lefteye = countfail_lefteye - 1;
              }
            }
          }
        }
      }

      // Right eye measurement.
      vector<Rect> measurement_righteye = detect_eyes(frame,region_righteye,eye_cascade);

      // Checks if the current measurement could be the other eye.
      same_eyes = 0;
      if (!measurement_righteye.empty()) {
        len_x = measurement_righteye[0].br().x - measurement_righteye[0].tl().x;
        ms_x = region_righteye.tl().x + measurement_righteye[0].tl().x + len_x/2;
        if (abs(ms_x - static_cast<int>(kf_lefteye.statePost.at<float>(0))) <= len_x/3) {
          // If the detected eye is too close horizontally to the other one,
          // they are probably the same.
          same_eyes = 1;
        }
      }

      if (!measurement_righteye.empty() & same_eyes == 0) {
        if (show == 1)
          eye_rectangle(print,region_righteye,measurement_righteye[0]);

        lengthx_righteye = measurement_righteye[0].br().x - measurement_righteye[0].tl().x;
        lengthy_righteye = measurement_righteye[0].br().y - measurement_righteye[0].tl().y;

        measurement(0) = region_righteye.tl().x + measurement_righteye[0].tl().x + lengthx_righteye/2;
        measurement(1) = region_righteye.tl().y + measurement_righteye[0].tl().y + lengthy_righteye/2;
        kf_righteye.correct(measurement);
        countfail_righteye = 0;

      } else {
        countfail_righteye = countfail_righteye + 1;

        /* If the maximum number of detection failures allowed in consecutive
         * frames is reached, initialize the Kalman filter with the detection
         * of the face again. */
        if (countfail_righteye >= kMaxFailures) {
          //set_kalman_filter_matrices(kf_eye2, kDeltaT, 100, 5, .1);
          setIdentity(kf_righteye.errorCovPost, Scalar::all(kPostError_reye));

          faces = detect_faces(frame,face_cascade);
          if (faces.empty()) {
            faces = detect_faces(frame,profile_cascade);
          }

          if (!faces.empty()) {
            for (Rect &face : faces) {
              if (show == 1)
                face_rectangle(print,face);
              Point point_tl, point_br;
              point_tl.x = face.tl().x + face.width/3;
              point_tl.y = face.tl().y;
              point_br = face.br();
              Rect region(point_tl,point_br);

              measurement_righteye = detect_eyes(frame,region,eye_cascade);

              // Checks if the current measurement could be the other eye.
              same_eyes = 0;
              if (!measurement_righteye.empty()) {
                len_x = measurement_righteye[0].br().x - measurement_righteye[0].tl().x;
                ms_x = region_righteye.tl().x + measurement_righteye[0].tl().x + len_x/2;
                if (abs(ms_x - static_cast<int>(kf_lefteye.statePost.at<float>(0))) <= len_x/3) {
                  // If the detected eye is too close horizontally to the other one,
                  // they are probably the same.
                  same_eyes = 1;
                }
              }

              if (!measurement_righteye.empty() & same_eyes == 0) {
                // If the current measurement is probably the other eye, ignores it.
                if (show == 1)
                  eye_rectangle(print,region,measurement_righteye[0]);

                lengthx_righteye = measurement_righteye[0].br().x - measurement_righteye[0].tl().x;
                lengthy_righteye = measurement_righteye[0].br().y - measurement_righteye[0].tl().y;

                measurement(0) = region.tl().x + measurement_righteye[0].tl().x + lengthx_righteye/2;
                measurement(1) = region.tl().y + measurement_righteye[0].tl().y + lengthy_righteye/2;
                kf_righteye.correct(measurement);
                countfail_righteye = countfail_righteye - 1;
              }
            }
          }
        }
      }

      // If the eyes detected are probably the same, forces reinitialization.
      if (countfail_lefteye == 0 & countfail_righteye == 0) {
        if (abs(kf_righteye.statePost.at<float>(0) - kf_lefteye.statePost.at<float>(0)) <= lengthx_lefteye/3) {
          countfail_lefteye = kMaxFailures;
          countfail_righteye = kMaxFailures;
        }
      }

      Point center_new, sum_center;
      // Left pupil measurement.
      Rect region_leftpupil = create_search_region(kf_lefteye, lengthx_lefteye, lengthy_lefteye, 0.5, frame_br);
      if (show == 1)
        rectangle(print, region_leftpupil, Scalar(255, 0, 0), 2);
      center_new = locate_eye_center(frame,region_leftpupil);
      sum_center = center_new;
      for (int j = 0; j < kNumCenters; j++)
        sum_center = sum_center + lastcenters_lefteye[j];

      center.x = sum_center.x/(kNumCenters+1);
      center.y = sum_center.y/(kNumCenters+1);

      if (countlast_lefteye < kNumCenters) {
        lastcenters_lefteye[countlast_lefteye] = center_new;
        countlast_lefteye = countlast_lefteye + 1;
      } else {
        for (int k = 0; k < kNumCenters-1; k++)
          lastcenters_lefteye[k] = lastcenters_lefteye[k+1];

        lastcenters_lefteye[kNumCenters-1] = center_new;
      }

      measurement(0) = region_leftpupil.tl().x + center.x;
      measurement(1) = region_leftpupil.tl().y + center.y;
      kf_leftpupil.correct(measurement);
      center_left.x = static_cast<int>(kf_leftpupil.statePost.at<float>(0));
      center_left.y = static_cast<int>(kf_leftpupil.statePost.at<float>(2));
      if (show == 1)
        drawMarker(print, center_left, Scalar(255, 255, 255), MARKER_CROSS, 10, 1);

      // Right pupil measurement.
      Rect region_rightpupil = create_search_region(kf_righteye, lengthx_righteye, lengthy_righteye, 0.5, frame_br);
      if (show == 1)
        rectangle(print, region_rightpupil, Scalar(255, 0, 0), 2);
      center_new = locate_eye_center(frame,region_rightpupil);
      sum_center = center_new;
      for (int j = 0; j < kNumCenters; j++)
        sum_center = sum_center + lastcenters_righteye[j];

      center.x = sum_center.x/(kNumCenters+1);
      center.y = sum_center.y/(kNumCenters+1);

      if (countlast_righteye < kNumCenters) {
        lastcenters_righteye[countlast_righteye] = center_new;
        countlast_righteye = countlast_righteye + 1;
      } else {
        for (int k = 0; k < kNumCenters-1; k++)
          lastcenters_righteye[k] = lastcenters_righteye[k+1];

        lastcenters_righteye[kNumCenters-1] = center_new;
      }

      measurement(0) = region_rightpupil.tl().x + center.x;
      measurement(1) = region_rightpupil.tl().y + center.y;
      kf_rightpupil.correct(measurement);
      center_right.x = static_cast<int>(kf_rightpupil.statePost.at<float>(0));
      center_right.y = static_cast<int>(kf_rightpupil.statePost.at<float>(2));
      if (show == 1)
        drawMarker(print, center_right, Scalar(255, 255, 255), MARKER_CROSS, 10, 1);

      // Mouth measurement.
      vector<Rect> mouth_measurement = detect_mouth(frame,mouth_region,mouth_cascade);
      if (!mouth_measurement.empty()) {
        if (show == 1)
          mouth_rectangle(print,mouth_region,mouth_measurement[0]);

        lengthx_mouth = mouth_measurement[0].br().x - mouth_measurement[0].tl().x;
        lengthy_mouth = mouth_measurement[0].br().y - mouth_measurement[0].tl().y;

        measurement(0) = mouth_region.tl().x + mouth_measurement[0].tl().x + lengthx_mouth/2;
        measurement(1) = mouth_region.tl().y + mouth_measurement[0].tl().y + lengthy_mouth/2;
        kf_mouth.correct(measurement);

        countfail_mouth = 0;
      } else {
        countfail_mouth = countfail_mouth + 1;

        /* If the maximum number of detection failures allowed in consecutive
         * frames is reached, initialize the Kalman filter with the detection
         * of the face again. */
        if (countfail_mouth >= kMaxFailures) {
          //set_kalman_filter_matrices(kf_mouth, kDeltaT, 100, 5, .1);
          setIdentity(kf_mouth.errorCovPost, Scalar::all(kPostError_mouth));

          faces = detect_faces(frame,face_cascade);
          if (faces.empty()) {
            faces = detect_faces(frame,profile_cascade);
          }

          if (!faces.empty()) {
            for (Rect &face : faces) {
              if (show == 1)
                face_rectangle(print, face);
              Point point_tl, point_br;
              point_tl.x = face.tl().x;
              point_tl.y = face.tl().y + face.height/2;
              point_br = face.br();
              Rect region(point_tl,point_br);

              mouth_measurement = detect_mouth(frame,region,mouth_cascade);
              if (!mouth_measurement.empty()) {
                if (show == 1)
                  mouth_rectangle(print,region,mouth_measurement[0]);

                lengthx_mouth = mouth_measurement[0].br().x - mouth_measurement[0].tl().x;
                lengthy_mouth = mouth_measurement[0].br().y - mouth_measurement[0].tl().y;

                measurement(0) = region.tl().x + mouth_measurement[0].tl().x + lengthx_mouth/2;
                measurement(1) = region.tl().y + mouth_measurement[0].tl().y + lengthy_mouth/2;
                kf_mouth.correct(measurement);

                countfail_mouth = countfail_mouth - 1;
              }
            }
          }
        }
      }
      point_mouth.x = kf_mouth.statePost.at<float>(0);
      point_mouth.y = kf_mouth.statePost.at<float>(2);

      // Only starts publishing after the indicated number of initialization
      // frames. The reference points are the mean of the first detections.
      if (count_init_frame == initialization_frames) {
        ref_left_pub.publish(set_msg(ref_left));
        ref_right_pub.publish(set_msg(ref_right));
        left_eye_pub.publish(set_msg(center_left));
        right_eye_pub.publish(set_msg(center_right));
        ref_mouth_pub.publish(set_msg(ref_mouth));
        mouth_pub.publish(set_msg(point_mouth));
      } else {
        count_init_frame = count_init_frame + 1;
        ref_left.x = ref_left.x + (center_left.x - ref_left.x)/count_init_frame;
        ref_left.y = ref_left.y + (center_left.y - ref_left.y)/count_init_frame;
        ref_right.x = ref_right.x + (center_right.x - ref_right.x)/count_init_frame;
        ref_right.y = ref_right.y + (center_right.y - ref_right.y)/count_init_frame;
        ref_mouth.x = ref_mouth.x + (point_mouth.x - ref_mouth.x)/count_init_frame;
        ref_mouth.y = ref_mouth.y + (point_mouth.y - ref_mouth.y)/count_init_frame;
      }
    }

    if (show == 1) {
      //char caminho[100];
      //sprintf(caminho,"/home/ana/frames/%d.png", num);
      //imwrite(caminho,print);

      imshow("result",print);
      waitKey(30);
    }
  }

  /*!
   * \brief Detector constructor.
   * \param frame_rate - A double with the frame rate of the camera.
   * \param face_file - Path to the face cascade file.
   * \param profile_file - Path to the profile face cascade file.
   * \param eyes_file - Path to the eyes cascade file.
   * \param mouth_file - Path to the mouth cascade file.
   * \param kCov - An array with the values for reinitialization of posteriori
   *               error estimate covariance matrices.
   * \param flag_show - An int to indicate if results should be displayed.
   * \param start - An int to indicate if detection should start at the beginning or not.
   * \param init_frames - An int with the number of frames that should be used in initialization.
   * \param nh - A ros::NodeHandle object.
   *
   * The constructor of the object loads the cascade files, sets the sampling
   * interval, according to the given frame rate, sets the values for
   * reinitialization of KF filters and the flag to indicate if results should
   * be displayed.
   */
  Detector(double frame_rate, string face_file, string profile_file,
           string eyes_file, string mouth_file,
           int kCov[5], int flag_show, int start, int init_frames,
           ros::NodeHandle &nh) {

    // Loading cascade files.
    face_cascade = load_training_models(face_file);
    profile_cascade = load_training_models(profile_file);
    eye_cascade = load_training_models(eyes_file);
    mouth_cascade = load_training_models(mouth_file);

    kDeltaT = 1/frame_rate;

    kPostError_leye = kCov[0];
    kPostError_reye = kCov[1];
    kPostError_mouth = kCov[2];
    kPostError_lpupil = kCov[3];
    kPostError_rpupil = kCov[4];

    center_left.x = 0;
    center_left.y = 0;
    center_right.x = 0;
    center_right.y = 0;

    show = flag_show;

    running = start;

    initialization_frames = init_frames;

    ref_left_pub = nh.advertise<geometry_msgs::Point>("ref_left", 1);
    ref_right_pub = nh.advertise<geometry_msgs::Point>("ref_right", 1);
    left_eye_pub = nh.advertise<geometry_msgs::Point>("left_eye", 1);
    right_eye_pub = nh.advertise<geometry_msgs::Point>("right_eye", 1);
    ref_mouth_pub = nh.advertise<geometry_msgs::Point>("ref_mouth", 1);
    mouth_pub = nh.advertise<geometry_msgs::Point>("mouth", 1);
    face_pub = nh.advertise<std_msgs::Int16>("face", 1);

    gaze_commands_sub = nh.subscribe("human_gaze_commands", 1,
                                     &Detector::human_gaze_callback, this);

  }
};
