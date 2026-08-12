#ifndef DETECTION_H_
#define DETECTION_H_

#include <opencv2/objdetect/objdetect.hpp>

using namespace std;
using namespace cv;

CascadeClassifier load_training_models(string file);

void face_rectangle(Mat frame, Rect face);

void eye_rectangle(Mat frame, Rect region, Rect eyes);

void mouth_rectangle(Mat frame, Rect region, Rect mouth);

vector<Rect> detect_faces(Mat frame, CascadeClassifier face_cascade);

vector<Rect> detect_eyes(Mat frame, Rect region, CascadeClassifier eye_cascade);

vector<Rect> choose_eyes(vector<Rect> detected_eyes);

vector<Rect> detect_mouth(Mat frame, Rect region, CascadeClassifier mouth_cascade);

Rect choose_mouth(vector<Rect> eyes, vector<Rect> detected_mouths);

double gradient_threshold(Mat gradient_x, Mat gradient_y);

double objective_sum(int possible_center_x, int possible_center_y, Mat gradient_x, Mat gradient_y, Mat weight, double thr);

Point objective_function(Mat weight, Mat gradient_x, Mat gradient_y, double thr);

Point locate_eye_center(Mat region, Rect eye);

#endif // DETECTION_H_
