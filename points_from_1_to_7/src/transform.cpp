#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include "builtin_interfaces/msg/time.hpp"

#include <cmath> // For M_PI
#include <chrono>
#include <sstream>
class FrameListener : public rclcpp::Node
{
public:
    explicit FrameListener():
    Node("FrameListener")
  {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&FrameListener::on_timer, this));
  }

  void printMatrix(const tf2::Matrix3x3& matrix) {
      std::ostringstream oss;
      for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
              oss << matrix[i][j] << " ";
          }
          oss << "\n";
      }
      RCLCPP_INFO(rclcpp::get_logger("matrix_logger"), "%s", oss.str().c_str());
  }
  void printTranslation(const tf2::Vector3 & vector) {
      std::ostringstream oss;
      for (int i = 0; i < 3; ++i) {
        oss << vector[i] << " ";
        oss << "\n";
      }
      RCLCPP_INFO(rclcpp::get_logger("translation: "), "%s", oss.str().c_str());
  }
private:

    void on_timer()
    {     
        std::string fromFrameRel = "rotating_frame";
        std::string toFrameRel = "camera_optical_frame";

        geometry_msgs::msg::TransformStamped t;

        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        tf2::Vector3 v( t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        
        printMatrix(m);
        printTranslation(v);
  }

 
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
 
 };

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");
 
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}