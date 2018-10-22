#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

void setup(std::string planFilename, std::string inputName, std::string outputName, bool _use_mappedMemory) {
  ROS_INFO("mock setup");
}
void destroy(void) {
}
void infer(cv::Mat image, float* out) {
  ROS_INFO("mock infer");
  for (int i = 0; i < 4; i++) {
    out[i] = 0.0;
  }
  out[3] = 5.0;
}
void test(void) {
  ROS_INFO("cuda not enabled");
}
