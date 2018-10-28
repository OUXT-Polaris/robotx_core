/* detector
 *
 * @node_name: cnn_prediction
 * @in: wam_v/front_camera/front_image_raw (Image)
 * @in: object_bbox_extractor_node/object_roi (OROIA)
 * @out: cnn_prediction_node/object_roi (ObjectRegionOfInterestArr)
 * ref: https://ipx.hatenablog.com/entry/2018/05/21/102659
 */
// message
#include <cnn_prediction.h>
#include <ros/ros.h>
#include <string>

// infer.cu(CUDA使用時), infer.cpp(CUDA使わない時)の関数を呼び出す
extern int convert(std::string uffFilename, std::string planFilename, std::string inputName, std::string outputName);
extern int setup(std::string planFilename, std::string inputName, std::string outputName, bool _use_mappedMemory);
extern void destroy(void);
extern void infer(cv::Mat image, float* out);
extern void test(void);

// constructor
cnn_predictor::cnn_predictor() :
  _it(_nh),
  _params(),
  _image_sub(_it, _params.image_topic, 1),
  _roi_sub(_nh,   _params.roi_topic, 1),
  _sync(SyncPolicy(1), _image_sub, _roi_sub)
{
  // publisher
  _roi_pub   = _nh.advertise<robotx_msgs::ObjectRegionOfInterestArray>("cnn_prediction_node/object_roi", 1);

  /* subscriber callbacks
   * message_filterについては
   * https://garaemon.github.io/blog/ros/2014/10/19/message-filters.html
   * https://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/  itと組み合わせる
   * http://robonchu.hatenablog.com/entry/2017/06/11/121000   approximatetimeについて
   * 参照
   */
  _sync.registerCallback(boost::bind(&cnn_predictor::callback, this, _1, _2));

  // tensorrt 初期化
  int plan_outputNum = setup(_params.model_filename, _params.model_inputName, _params.model_outputName, _params.use_mapped_memory);
  if (plan_outputNum <= 0) {
    ROS_INFO("error while initializing tensorrt");
  }

  ROS_INFO("detector initialized %d, %s", _params.model_outputNum, _params.image_topic.c_str());
}

// destructor
cnn_predictor::~cnn_predictor() {
  destroy();
}

// callbacks
void cnn_predictor::callback(
    const sensor_msgs::ImageConstPtr& image_msg,
    const robotx_msgs::ObjectRegionOfInterestArrayConstPtr& rois_msg) {
  // 両方のが揃った時

  // 画像 image_msg
  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB, 3);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  robotx_msgs::ObjectRegionOfInterestArray res = _image_recognition(*rois_msg, image);
  // 判定結果を送信する
  _roi_pub.publish(res);
}

// 画像認識: 基本的にはroisのアップデートをする感じ (rois, image) -> (rois)
robotx_msgs::ObjectRegionOfInterestArray cnn_predictor::_image_recognition(const robotx_msgs::ObjectRegionOfInterestArray rois, const cv::Mat image) {
  robotx_msgs::ObjectRegionOfInterestArray res;
  res.header = rois.header;
  for (int i = 0; i < rois.object_rois.size(); i++) {
    robotx_msgs::ObjectRegionOfInterest roi = rois.object_rois[i];
    // 矩形領域の切り出し
    int roi_h = roi.roi_2d.height,
        roi_w = roi.roi_2d.width,
        roi_x = roi.roi_2d.x_offset,
        roi_y = roi.roi_2d.y_offset,
        img_h = image.rows,
        img_w = image.cols;
    ROS_INFO("h:%d, w:%d, x:%d, y:%d, H:%d, W:%d", roi_h, roi_w, roi_x, roi_y, img_h, img_w);
    if (roi_x >= 0 && roi_y >= 0 && roi_x + roi_w <= img_w && roi_y + roi_h <= img_h && roi_w > 0 && roi_h > 0) {
      // ROI is valid
      cv::Rect rect(cv::Point(roi_x, roi_y), cv::Size(roi_w, roi_h));
      cv::Mat subimage = image(rect);
      // 切り出した部分をCNNのinputサイズに合わせる
      // 切り出したところについてtensorrtで確率を計算
      float result[_params.model_outputNum];
      infer(subimage, result);
      float max_prob = -1000;
      int index = -1;
      for (int i = 0; i < _params.model_outputNum; i++) {
        if (max_prob < result[i]) {
          max_prob = result[i];
          index = i;
        }
      }
      switch(index) {
        case 0:
          roi.object_type.ID = roi.object_type.OTHER;
          break;
        case 1:
          roi.object_type.ID = roi.object_type.GREEN_BUOY;
          break;
        case 2:
          roi.object_type.ID = roi.object_type.RED_BUOY;
          break;
        case 3:
          roi.object_type.ID = roi.object_type.WHITE_BUOY;
          break;
      }
      roi.objectness = result[index];
    } else {
      // invalid ROI
      roi.objectness = 0.0;
      roi.object_type.ID = roi.object_type.OTHER;
    }
    res.object_rois.push_back(roi);
  }
  return res;
}
