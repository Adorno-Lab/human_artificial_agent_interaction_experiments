#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

class Data {
public:
  string path = "";
  string camera = "";
  int count = 1;
  char datetime[23];
  ofstream file;

  void image_callback (const sensor_msgs::ImageConstPtr& msg) {
    // Getting the date and time from the message header.
    std_msgs::Header h = msg->header;
    time_t rawtime = h.stamp.sec;
    strftime(datetime, 23, "%d/%m/%Y %H:%M:%S", localtime(&rawtime));

    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Converting image.
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      Mat frame = cv_ptr->image;

      // Saving image file.
      char num[6];
      sprintf(num, "%06d", count);
      string image_name = path + "temp_images/" + camera + "/" + num + ".png";
      imwrite(image_name, frame);

      // Adding date and time to the file.
      file << to_string(count) << "\t" << datetime << endl;

      count++;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  Data(string camera, string path) {
    this->camera = camera;
    this->path = path;

    string file_name = path + camera + ".txt";
    file.open(file_name, ios::out);
  }
};
    
int main (int argc, char** argv) {
  // argv[1]: camera name.
  // argv[2]: topic name.
  // argv[3]: path to files.

  if (argc != 4) {
    cout << "ERROR: Invalid number of arguments." << endl;
    cout << "Arguments should be: camera name, image topic name, and path to "
            "the image files." << endl;
    return 0;
  }

  string node_name1("image_saver_");
  string node_name2(argv[1]);
  string node_name = node_name1 + node_name2;
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  image_transport::ImageTransport it = image_transport::ImageTransport(nh);

  string topic_kinect = argv[2];

  Data d(argv[1], argv[3]);

  while (ros::ok()) {
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe(topic_kinect, 1, &Data::image_callback, &d);

    ros::spin();
  }

  d.file.close();

  return 0;
}
