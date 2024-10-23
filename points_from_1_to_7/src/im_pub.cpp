#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>

class Publish_images : public rclcpp::Node {
public:
    explicit Publish_images(): 
        Node("im_pub"),node_handle_(std::shared_ptr<Publish_images>(this)),
            it_(node_handle_)
    {
       
        pub_ = it_.advertise("camera/Image", 10);
        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Publish_images::timer_callback, this));
     }

    void timer_callback(){
        //cv::Mat image = cv::imread("/home/claudio/amaris_ws/src/interview_homework/img/lena.png", cv::IMREAD_COLOR);

        std::string package_path =  ament_index_cpp::get_package_share_directory("points_from_1_to_7");

        std::string image_path = package_path + "/img/lena.png";

        //RCLCPP_INFO(this->get_logger(), "Published camera info: %s", package_path.c_str());  
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        cv::Size newSize(1024, 768);  
  
        // Resize the image
        cv::Mat resizedImage;
        cv::resize(image, resizedImage, newSize);
        std_msgs::msg::Header hdr;
        auto stamp = this->get_clock()->now();
        hdr.stamp = stamp;
        hdr.frame_id = "camera_optical_frame"; 
        sensor_msgs::msg::Image::SharedPtr msg;
        msg = cv_bridge::CvImage(hdr, "bgr8", resizedImage).toImageMsg(); //from cv to ros_msg
        pub_.publish(msg);

        auto msg_cam_info = sensor_msgs::msg::CameraInfo();
        
        //msg_cam_info.header.stamp = this->get_clock()->now();
        msg_cam_info.header.stamp = stamp; 
        msg_cam_info.header.frame_id = "camera_optical_frame";

        msg_cam_info.width = 1024; 
        msg_cam_info.height = 768;


        int horizontal_fov_degrees = 30;
        double horizontal_fov_radians = horizontal_fov_degrees * M_PI / 180.0;
 
        double focal_length = (msg_cam_info.width / 2) / tan(horizontal_fov_radians / 2);

        msg_cam_info.k = { focal_length, 0.0, msg_cam_info.width/2, 0.0, focal_length, msg_cam_info.height/2, 0.0, 0.0, 1.0 }; // Intrinsic matrix
        msg_cam_info.d = { 0.1, -0.25, 0.0, 0.0, 0.0 }; // Distortion coefficients
        msg_cam_info.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }; // Rectification matrix
        msg_cam_info.p = { 600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0 }; // Projection matrix

        publisher_->publish(msg_cam_info);
        
    }


private:

    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publish_images>());
    rclcpp::shutdown();
    return 0;
}
