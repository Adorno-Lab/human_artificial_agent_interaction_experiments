#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "detection.h"
#include "detection_kalmanfilter.h"
#include "Detector.cpp"

using namespace std;
using namespace cv;

/**
 * \brief Converts the string str in the int array arr.
 * \param str - The string with the numbers.
 * \param arr - The array where to save the numbers got from the string.
 *
 * The string format should be "X, Y, Z".
 */
void get_matrices_values(string str, int arr[3]) {
  int index = 0;

  for (int i = 0; str[i] != '\0'; i++) {
    if(str[i] == ',') {
      i++;
      index++;
    } else {
      arr[index] = arr[index] * 10 + (str[i] - 48);
    }
  }
}

int main (int argc, char** argv) {

  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;

  // Frame rate of the camera.
  double frame_rate;
  // Path to the haarcascade files.
  string files_path;
  // Flag to indicate the source of the image.
  int camera; // 0 for Kinect, 1 for webcam, 2 for recorded video.
  // The topic where to get the image from if Kinect is being used.
  string topic_kinect = "";
  // Values for initialization of the Kalman filters matrices.
  string read_values;
  int lefteye_matrices[3] = {0};
  int righteye_matrices[3] = {0};
  int mouth_matrices[3] = {0};
  int leftpupil_matrices[3] = {0};
  int rightpupil_matrices[3] = {0};
  // Flag to indicate if results should be displayed.
  int show = 0; 
  // Flag to indicate if results should be saved.
  int results = 0;  
  // Path to a recorded video.
  string video_path = "";
  // Path where to save the result files.
  string results_path;
  // Flag to indicate if detection should start at the beginning.
  int start = 0;
  // Number of frames that should be used in initialization.
  int initialization_frames = 1;

  // Getting parameters from launch file.
  nh.getParam("frame_rate", frame_rate);
  nh.getParam("files_path", files_path);
  nh.getParam("camera", camera);
  nh.getParam("topic_kinect", topic_kinect);
  nh.getParam("lefteye_matrices", read_values);
  get_matrices_values(read_values, lefteye_matrices);
  nh.getParam("righteye_matrices", read_values);
  get_matrices_values(read_values, righteye_matrices);
  nh.getParam("mouth_matrices", read_values);
  get_matrices_values(read_values, mouth_matrices);
  nh.getParam("leftpupil_matrices", read_values);
  get_matrices_values(read_values, leftpupil_matrices);
  nh.getParam("rightpupil_matrices", read_values);
  get_matrices_values(read_values, rightpupil_matrices);
  nh.getParam("show", show);
  nh.getParam("results", results);
  nh.getParam("video_path", video_path);
  nh.getParam("results_path", results_path);
  nh.getParam("start", start);
  nh.getParam("initialization_frames", initialization_frames);

  // File to save the results.
  ofstream results_file;
  if (results == 1)
    results_file.open(results_path);

  // Values for reinitialization of posteriori error estimate covariance matrices.
  int kCov[5] = {lefteye_matrices[2], righteye_matrices[2],
                    mouth_matrices[2], leftpupil_matrices[2],
                    rightpupil_matrices[2]};

  Detector d(frame_rate,
             string(files_path + "/haarcascade_frontalface_alt.xml"),
             string(files_path + "/haarcascade_profileface.xml"),
             string(files_path + "/haarcascade_eye.xml"),
             string(files_path + "/haarcascade_mcs_mouth.xml"),
             kCov, show, start, initialization_frames, nh);

  // Configuring the Kalman filter matrices.
  set_kalman_filter_matrices(d.kf_lefteye, d.kDeltaT, lefteye_matrices);
  set_kalman_filter_matrices(d.kf_righteye, d.kDeltaT, righteye_matrices);
  set_kalman_filter_matrices(d.kf_mouth, d.kDeltaT, mouth_matrices);
  set_kalman_filter_matrices(d.kf_leftpupil, d.kDeltaT, leftpupil_matrices);
  set_kalman_filter_matrices(d.kf_rightpupil, d.kDeltaT, rightpupil_matrices);

  // For Kinect image.
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);

  // For webcam image or video.
  VideoCapture cap;
  if (camera == 1)
    cap.open(0);
  if (camera == 2)
    cap.open(video_path);

  if (camera == 1 || camera == 2) {
    if (!cap.isOpened()) {
      cout << "Error in video capture" << "\n";
    }
  }

  //int num = 1;

  // The while loop reads data from camera, calls the detection function and
  // publishes important data.
  if (camera == 0) {
    while (ros::ok()) {
      image_transport::Subscriber image_sub;
      image_sub = it.subscribe(topic_kinect, 1,
                               &Detector::image_callback, &d);

      if (results == 1) {
        results_file << d.center_left.x << "\t" << d.center_left.y << "\t";
        results_file << d.center_right.x << "\t" << d.center_right.y << "\t";
        results_file << d.point_mouth.x << "\t" << d.point_mouth.y << "\n";
      }

      ros::spin();
    }
  } else {
    while (ros::ok()) {
      Mat frame;
      cap >> frame;
      if (frame.empty())
        break;

      d.detection(frame);
      //d.detection(frame, num);
      //num = num + 1;

      if (results == 1) {
        results_file << d.center_left.x << "\t" << d.center_left.y << "\t";
        results_file << d.center_right.x << "\t" << d.center_right.y << "\t";
        results_file << d.point_mouth.x << "\t" << d.point_mouth.y << "\n";
      }
    }
  }

  results_file.close();
  return 0;
}
