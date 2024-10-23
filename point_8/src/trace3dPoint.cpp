#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>

#include <chrono>

using namespace std::chrono_literals;

class Trace3dRay : public rclcpp::Node {
public:
    explicit Trace3dRay(): 
        Node("ray_from_image")
    {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10, std::bind(&Trace3dRay::cameraSubscriber, this, std::placeholders::_1));

        pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/ray", 10);

    }

    static void mouseCallback(int event, int x, int y, int, void* params)
    {   
        Trace3dRay* instance = (Trace3dRay*)(params);
          if (event == cv::EVENT_LBUTTONDOWN){
                double ndc_x = (x - instance->cx_) / instance->fx_;
                double ndc_y = (y - instance->cy_) / instance->fy_;
                std::cout<<"point coordinate: "<<ndc_x<<" "<<ndc_y<<std::endl;
                Eigen::Vector3d ray_direction(ndc_x, ndc_y, 1.0);
                ray_direction.normalize();  

                int arbitraryScaling = 200;
                ray_direction *= arbitraryScaling;
                instance->projectPoint(ray_direction);

        }
    }

    void projectPoint(Eigen::Vector3d & ray_direction){
 
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = ray_direction.x();
        pose_msg.position.y = ray_direction.y();
        pose_msg.position.z = ray_direction.z();
        pose_msg.orientation.w = 1.0; 
        
        pose_pub->publish(pose_msg);
        
 
    }


    void cameraSubscriber(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info){

        int height = cam_info->height;
        int width = cam_info->height;
        fx_ = cam_info->k[0]; 
        fy_ = cam_info->k[4]; 
        cx_ = cam_info->k[2]; 
        cy_ = cam_info->k[5];
    }

private:
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;

    Eigen::Vector3d ray_direction_ = Eigen::Vector3d::Zero();

    double fx_;  
    double fy_; 
    double cx_;  
    double cy_;

};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    cv::namedWindow("Image");
    std::string package_path =  ament_index_cpp::get_package_share_directory("point_8");
    std::string image_path = package_path + "/img/lena.png";
    cv::imshow("Image", cv::Mat(440, 440, CV_8UC3, cv::Scalar(255, 0, 255)));

    std::shared_ptr<Trace3dRay> node = std::make_shared<Trace3dRay>();

    cv::setMouseCallback("Image", Trace3dRay::mouseCallback, node.get());

    
    while (rclcpp::ok() && cv::waitKey(1)) {
        std::this_thread::sleep_for(100ms);
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
