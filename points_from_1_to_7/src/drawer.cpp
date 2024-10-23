#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>

#include <chrono>

using namespace std::chrono_literals;

class FrameDrawer_on_image : public rclcpp::Node {
public:
    explicit FrameDrawer_on_image(): 
        Node("drawer"), node_handle_(std::shared_ptr<FrameDrawer_on_image>(this)),
            it_(node_handle_)
    {

        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); //connect tf2_listner to the /tf topic
       
        /* 
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/Image", 10, std::bind(&FrameDrawer_on_image::ImageSub, this, std::placeholders::_1));*/

        sub_ = it_.subscribeCamera("camera/Image", 10,
            std::bind(&FrameDrawer_on_image::cameraSubscriber, this, std::placeholders::_1, std::placeholders::_2));

        /*typedef message_filters::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MySyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), image_sub_, info_sub_);
        
        /*subscriber_temp1_.subscribe(this, "/camera/Image", rmw_qos_profile);
        temp_sync_->registerCallback(std::bind(&FrameDrawer_on_image::cameraSubscriber, this, std::placeholders::_1, std::placeholders::_2));*/
    }

 
    void cameraSubscriber(const sensor_msgs::msg::Image::ConstSharedPtr& raw_image,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info){
        
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        input_bridge = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8); //from ros_msg to cv
        image = input_bridge->image;          

        cam_model_.fromCameraInfo(cam_info);
   
        //Tranfor the coordinates
        std::string frame_id = "rotating_frame";
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(cam_info->header.frame_id, frame_id, tf2::TimePointZero);

        tf2::Vector3 pt(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3 m(q);

        //Set the point in the image
        cv::Point3d pt_cv(pt.x(), pt.y(), pt.z()); // Create OpenCV Point3d
        cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
        std::cout << "UV coordinates: (" << uv.x << ", " << uv.y << ")" << std::endl;
        static const int RADIUS = 10;


        cv::circle(image, uv, RADIUS, cv::Scalar(255, 0, 0), cv::FILLED);
         
        int baseline = 0;
        std::string text = "camera_optical_frame";
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);

        // Position the text above the circle
        cv::Point text_origin(uv.x - text_size.width / 2,
                         uv.y - RADIUS - baseline - 3);
        // Draw the text
        cv:putText(image, frame_id.c_str(), text_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,0,255));

        cv::namedWindow("Image", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image", 1024, 768);  
        cv::imshow("Image", image);
        
        cv::waitKey(1);
    }

private:
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    /*tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;*/
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    image_geometry::PinholeCameraModel cam_model_;
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_; 
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::CameraSubscriber sub_; //be aware with image_transport::Subscriber
    image_transport::Subscriber sub_img_;

    message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_temp1_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> subscriber_temp2_;

    float cx_;
    float cy_;
    float fx_;    
    float fy_;


    //std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> temp_sync_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameDrawer_on_image>());
    rclcpp::shutdown();
    return 0;
}
