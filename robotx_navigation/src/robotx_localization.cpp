#include <robotx_localization.h>

robotx_localization::robotx_localization() : params_() {
  initialize_particle_filter_();
  robot_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  init_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/origin/fix", 1);
  reset_sub_ = nh_.subscribe(params_.reset_topic, 1, &robotx_localization::reset_callback_, this);
  fix_sub_ = nh_.subscribe(params_.fix_topic, 1, &robotx_localization::fix_callback_, this);
  twist_sub_ = nh_.subscribe(params_.twist_topic, 1, &robotx_localization::twist_callback_, this);
  imu_sub_ = nh_.subscribe(params_.imu_topic, 1, &robotx_localization::imu_callback_, this);
  thread_update_frame_ = boost::thread(boost::bind(&robotx_localization::update_frame_, this));
}

robotx_localization::~robotx_localization() { thread_update_frame_.join(); }

void robotx_localization::initialize_particle_filter_(){
  yaw_ = 0;
  Eigen::VectorXd init_value = Eigen::VectorXd::Ones(3);
  init_value;
  pfilter_ptr_ = new particle_filter(3, params_.num_particles, init_value);
  fix_recieved_ = false;
  twist_received_ = false;
  imu_recieved_ = false;
  return;
}

void robotx_localization::update_frame_() {
  ros::Rate rate(params_.publish_rate);
  while (is_sensor_ready_() == false) {
    rate.sleep();
  }
  while (ros::ok()) {
    std::lock(fix_mutex_, twist_mutex_, imu_mutex_);
    // critical section start
    pfilter_ptr_->resample(params_.ess_threshold);
    Eigen::VectorXd control_input(3);
    Eigen::VectorXd state = pfilter_ptr_->get_state();
    control_input(0) = (std::cos(state(2)) * last_twist_message_.linear.x -
                        std::sin(state(2)) * last_twist_message_.linear.y) /
                       params_.publish_rate;
    control_input(1) = (std::sin(state(2)) * last_twist_message_.linear.x +
                        std::cos(state(2)) * last_twist_message_.linear.y) /
                       params_.publish_rate;
    control_input(2) = last_twist_message_.angular.z / params_.publish_rate;
    pfilter_ptr_->add_system_noise(control_input, 0.0);
    Eigen::MatrixXd states = pfilter_ptr_->get_states();
    double measurement_x = (last_fix_message_.longitude - init_measurement_.longitude) * 111263.283;
    double measurement_y = (last_fix_message_.latitude - init_measurement_.latitude) * 6378150 *
                           std::cos(last_fix_message_.longitude / 180 * M_PI) * 2 * M_PI / (360 * 60 * 60);
    Eigen::VectorXd weights(params_.num_particles);
    for (int i = 0; i < params_.num_particles; i++) {
      double error =
          std::sqrt(std::pow(states(0, i) - measurement_x, 2) + std::pow(states(1, i) - measurement_y, 2) + 100*std::pow(states(2, i) - yaw_, 2));
      double threashold = 0.001;
      // avoid zero division
      if (std::fabs(error) < threashold) error = threashold;
      weights(i) = 1 / error;
    }
    pfilter_ptr_->set_weights(weights);
    Eigen::VectorXd predicted_pos = pfilter_ptr_->get_state();
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = params_.publish_frame;
    transform_stamped.child_frame_id = params_.robot_frame;
    transform_stamped.transform.translation.x =
        predicted_pos(0);
    transform_stamped.transform.translation.y =
        predicted_pos(1);
    transform_stamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, predicted_pos(2));
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    broadcaster_.sendTransform(transform_stamped);
    geometry_msgs::PoseStamped robot_pose_msg;
    robot_pose_msg.header = transform_stamped.header;
    robot_pose_msg.pose.position.x = predicted_pos(0);
    robot_pose_msg.pose.position.y = predicted_pos(1);
    robot_pose_msg.pose.position.z = 0;
    robot_pose_msg.pose.orientation.x = q.x();
    robot_pose_msg.pose.orientation.y = q.y();
    robot_pose_msg.pose.orientation.z = q.z();
    robot_pose_msg.pose.orientation.w = q.w();
    robot_pose_pub_.publish(robot_pose_msg);
    nav_msgs::Odometry odom_msg;
    odom_msg.header = robot_pose_msg.header;
    odom_msg.child_frame_id = params_.robot_frame;
    odom_msg.pose.pose = robot_pose_msg.pose;
    odom_msg.twist.twist = last_twist_message_;
    odom_pub_.publish(odom_msg);
    init_fix_pub_.publish(init_measurement_);
    // critical section end
    fix_mutex_.unlock();
    twist_mutex_.unlock();
    imu_mutex_.unlock();
    rate.sleep();
  }
  return;
}

void robotx_localization::reset_callback_(std_msgs::Empty msg){
  std::lock(fix_mutex_, twist_mutex_, imu_mutex_);
  initialize_particle_filter_();
  fix_mutex_.unlock();
  twist_mutex_.unlock();
  imu_mutex_.unlock();
  return;
}

bool robotx_localization::is_sensor_ready_(){
  if(fix_recieved_ == true && twist_received_ == true && imu_recieved_ == true){
    return true;
  }
  else{
    return false;
  }
}

void robotx_localization::fix_callback_(sensor_msgs::NavSatFix msg) {
  std::lock_guard<std::mutex> lock(fix_mutex_);
  if (fix_recieved_ == false) {
    init_measurement_ = msg;
  }
  last_fix_message_ = msg;
  fix_recieved_ = true;
  return;
}

void robotx_localization::twist_callback_(geometry_msgs::Twist msg) {
  std::lock_guard<std::mutex> lock(twist_mutex_);
  last_twist_message_ = msg;
  twist_received_ = true;
  return;
}

void robotx_localization::imu_callback_(sensor_msgs::Imu msg){
  std::lock_guard<std::mutex> lock(imu_mutex_);
  double roll;
  double pitch;
  double yaw;
  tf2::Quaternion quat(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  if(imu_recieved_ == false){
    init_yaw_ = yaw;
  }
  yaw_ = yaw - init_yaw_;
  imu_recieved_ = true;
  return;
}