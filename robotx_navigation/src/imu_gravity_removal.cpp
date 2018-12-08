#include <imu_gravity_removal.h>

imu_gravity_removal::imu_gravity_removal()
    : params_(){

    start_time_ = std::chrono::system_clock::now(); // 計測開始時間
    rm_gravity_pub_ = 
        nh_.advertise<geometry_msgs::TwistStamped>(ros::this_node::getName() + "/imu_rm_gravity", 1);
    raw_imu_sub_ =
      nh_.subscribe(params_.input_imu_topic, 1, &imu_gravity_removal::imu_CB_, this);

    vel_.x = 0.0;
    vel_.y = 0.0;
    vel_.z = 0.0;
    grav_.x = 0.0;
    grav_.y = 0.0;
    grav_.z = 0.0;
    old_acc_.x = 0.0;
    old_acc_.y = 0.0;
    old_acc_.z = 0.0;
    old_raw_acc_.x = 0.0;
    old_raw_acc_.y = 0.0;
    old_raw_acc_.z = 0.0;
}

imu_gravity_removal::~imu_gravity_removal() {}

void imu_gravity_removal::imu_CB_(const sensor_msgs::Imu msg) {
    geometry_msgs::TwistStamped pub_geo;
    end_time_ = std::chrono::system_clock::now();
    sec_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_-start_time_).count() / 1000.0;
    pub_geo.header.stamp = ros::Time::now();
    pub_geo.header.frame_id = params_.frame_id;

    /*速度を求める*/
    // 重力加速度を求める
    grav_.x = params_.LPF_const_value * old_raw_acc_.x + (1.0 - params_.LPF_const_value) * msg.linear_acceleration.x;
    grav_.y = params_.LPF_const_value * old_raw_acc_.y + (1.0 - params_.LPF_const_value) * msg.linear_acceleration.y;
    grav_.z = params_.LPF_const_value * old_raw_acc_.z + (1.0 - params_.LPF_const_value) * msg.linear_acceleration.z;

    old_raw_acc_.x = msg.linear_acceleration.x;
    old_raw_acc_.y = msg.linear_acceleration.y;
    old_raw_acc_.z = msg.linear_acceleration.z;

    ROS_INFO("vec length = %f",sqrt(pow(grav_.x,2)+pow(grav_.y,2)+pow(grav_.z,2)));

    ROS_INFO("grav_.x = %f",grav_.x);
    ROS_INFO("grav_.y = %f",grav_.y);
    ROS_INFO("grav_.z = %f",grav_.z);

    // 補正した加速度
    // pub_geo.twist.linear.x = msg.linear_acceleration.x - grav_.x;
    // pub_geo.twist.linear.y = msg.linear_acceleration.y - grav_.y;
   // pub_geo.twist.linear.z = msg.linear_acceleration.z - grav_.z;

    ROS_INFO("ccorrectioned vec length = %f",sqrt(pow(pub_geo.twist.linear.x,2)+pow(pub_geo.twist.linear.y,2)+pow(pub_geo.twist.linear.z,2)));

    //vel_.x += ((pub_geo.twist.linear.x + old_acc_.x) * sec_) / 2.0  ;
    //vel_.y += ((pub_geo.twist.linear.y + old_acc_.y) * sec_) / 2.0  ;
    //vel_.z -= ((pub_geo.twist.linear.z + old_acc_.z) * sec_) / 2.0  ;
 
    vel_.x += (pub_geo.twist.linear.x * sec_);
    vel_.y += (pub_geo.twist.linear.y * sec_);
    // vel_.z += (pub_geo.twist.linear.z * sec_);

    old_acc_.x = pub_geo.twist.linear.x;
    old_acc_.y = pub_geo.twist.linear.y;
    old_acc_.z = pub_geo.twist.linear.z;

    ROS_INFO("sec_ = %f",sec_);


    pub_geo.twist.linear.x = vel_.x;
    pub_geo.twist.linear.y = vel_.y;
    pub_geo.twist.linear.z = 0;
    pub_geo.twist.angular.x = msg.angular_velocity.x;
    pub_geo.twist.angular.y = msg.angular_velocity.y;
    pub_geo.twist.angular.z = msg.angular_velocity.z;
    rm_gravity_pub_.publish(pub_geo.twist);
    start_time_ = std::chrono::system_clock::now();

}

