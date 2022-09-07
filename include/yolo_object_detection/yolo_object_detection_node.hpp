#ifndef YOLO_OBJECT_DETECTION_NODE_HPP
#define YOLO_OBJECT_DETECTION_NODE_HPP

#include <iostream>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <string>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <string>

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
    void precCamImgCallback(const sensor_msgs::ImageConstPtr &msg);
    void frontCamImgCallback(const sensor_msgs::ImageConstPtr &msg);
    void publishInThread(std::vector<bbox_t> objects, std::string obj_name);
    void drawBoxes(cv::Mat mat_img, std::vector<bbox_t> objects);
    void recordData(struct timeval startTime);
    void detectInThread();

    // ROS nh, sub, pub
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber precCamImgSubscriber_;
    image_transport::Subscriber frontCamImgSubscriber_;
    ros::Subscriber runYoloSubscriber_;
    ros::Publisher boundingBoxPublisher_;

    cv::Mat precCamImageCopy_;
    cv::Mat frontCamImageCopy_;
    bool imageStatus_ = false;

    std::thread detectThread_;
    std::mutex prec_cam_mutex_, front_cam_mutex_;

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

    struct timeval startTime_;
    double delay_ = 0.0;

    double sec_ = 0.0, nsec_ = 0.0;
};

} // namespace yolo_object_detection

#endif // YOLO_OBJECT_DETECTION_NODE_HPP
