#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "builtin_interfaces/msg/time.hpp"

#include <cmath> // For M_PI
#include <chrono>
#include <sstream>
class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher()
  : Node("framePublisher")
  {
    this->declare_parameter<float>("angvel", 0.5);
    this->declare_parameter<float>("linvel", 0.5);

    this->get_parameter("angvel", angvel_);
    this->get_parameter("linvel", linvel_);

    RCLCPP_INFO(this->get_logger(), "linear velocity: %f", linvel_);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Publish static transforms once at startup
    const char* base_link_frame[7] = {"base_link", "0", "0", "1", "0", "0", "0"};
    this->make_transforms("map", base_link_frame);

    const char* camera_frame[7] = {"camera_frame", "0.1", "0", "0", "0", "0", "0"};
    this->make_transforms("base_link", camera_frame);
    
    const char* camera_optical_frame[7] = {"camera_optical_frame", "0", "0", "0", "-1.57", "0", "-1.57"}; //both transformation are reffed to the camera frame
    //so, I have to rotate the camera frame of 90 degree counterclockise about x, and then, ALWAYS with respect the camera frame of 90 counterclockwise about z. Remember, it will rotate x first, y then and z finally
    //+ clockwise
    //- counterclockwise
    // all the rotation will be done with respect the previous frame.
    this->make_transforms("camera_frame", camera_optical_frame);    
    while(rclcpp::ok()){
      this->publishRotatingFrame(angvel_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  void make_transforms(const char * parent_name, const char * transformation[])
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_name;
    t.child_frame_id = transformation[0];

    t.transform.translation.x = atof(transformation[1]);
    t.transform.translation.y = atof(transformation[2]);
    t.transform.translation.z = atof(transformation[3]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[4]),
      atof(transformation[5]),
      atof(transformation[6]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  void publishRotatingFrame(float omega_z)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    angle += omega_z*float(0.01);
    if(angle > 2 * M_PI){
      angle -= 2 * M_PI;
    }

    if(changelinVel){
      linvel_ = -linvel_;
      changelinVel = false;
    }

    z_pos += linvel_*float(0.01);

    if (z_pos > 2 || z_pos < 0){
      changelinVel = true;
    }

    //RCLCPP_INFO(this->get_logger(), "Current position along z: [%s]", std::to_string(z_pos).c_str());
    //RCLCPP_INFO(this->get_logger(), "Current orientation about z: [%s]", std::to_string(angle).c_str());

    t.header.frame_id = "map";
    t.child_frame_id = "rotating_frame";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = z_pos;

    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
     
    tf_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  builtin_interfaces::msg::Time _dt;
  double angle; double z_pos;
  float angvel_;
  float linvel_;
  bool changelinVel = false;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");
 
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}