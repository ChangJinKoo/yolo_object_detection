#include <ros/ros.h>
#include <yolo_object_detection/yolo_object_detection_node.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_object_detection_node");
  ros::NodeHandle nodeHandle("~");
  yolo_object_detection::YoloObjectDetectionNode YOD(nodeHandle);

  ros::spin();
  return 0;
}
