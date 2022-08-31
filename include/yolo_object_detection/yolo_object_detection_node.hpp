#ifndef YOLO_OBJECT_DETECTION_NODE_HPP
#define YOLO_OBJECT_DETECTION_NODE_HPP

#include <iostream>
#include <fstream>
#include <opencv2/imgproc.hpp>

#include <time.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "yolo_v2_class.hpp"

#include <scale_truck_control/yolo_flag.h>
#include <yolo_object_detection/bounding_box.h>

namespace yolo_object_detection
{

class YoloObjectDetectionNode
{

public:
    explicit YoloObjectDetectionNode(ros::NodeHandle nh);
    ~YoloObjectDetectionNode();

private:
    bool readParameters();
    void init();
    static std::vector<std::string> objectNames(std::string const filename);
    void runYoloCallback(const scale_truck_control::yolo_flag &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void publishInThread(std::vector<bbox_t> objects);
    void drawBoxes(cv::Mat mat_img, std::vector<bbox_t> objects);

    // ROS nh, sub, pub
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSubscriber_;
    ros::Subscriber runYoloSubscriber_;
    ros::Publisher boundingBoxPublisher_;

    cv::Mat camImageCopy_;
    bool imageStatus_ = false;

    Detector *yoloDetector_; // use smart ptr instead
    std::vector<std::string> objectNames_;
    std::vector<bbox_t> objects_;

    std::string names_;
    std::string cfg_;
    std::string weights_;

    bool run_yolo_ = false;
    bool viewImage_;
    bool enableConsoleOutput_;
    int waitKeyDelay_;

    int width_;
    int height_;
};

} // namespace yolo_object_detection

#endif // YOLO_OBJECT_DETECTION_NODE_HPP
