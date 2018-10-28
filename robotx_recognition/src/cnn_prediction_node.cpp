#include <cnn_prediction.h>

/* main */
// http://blog-sk.com/ubuntu/ros_cvbridge/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cnn_prediction_node");
  cnn_predictor cnn_predictor;
  ros::spin();
  return 0;
}

