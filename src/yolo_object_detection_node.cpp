#include "yolo_object_detection/yolo_object_detection_node.hpp"

namespace yolo_object_detection
{

YoloObjectDetectionNode::YoloObjectDetectionNode(ros::NodeHandle nh)
    : nodeHandle_(nh), imageTransport_(nodeHandle_)
{
  if (!readParameters()){
    ros::requestShutdown();
  }

  init();
}

YoloObjectDetectionNode::~YoloObjectDetectionNode()
{
  delete yoloDetector_;
}

bool YoloObjectDetectionNode::readParameters(){
  std::string names_file, cfg_file, weights_file;
  std::string names_path, cfg_path, weights_path;

  // Flag to run YOLO
  nodeHandle_.param("run_yolo", run_yolo_, false);

  // Path to names file
  nodeHandle_.param("yolo_model/names_file/name", names_file, std::string("obj.names"));
  nodeHandle_.param("names_path", names_path, std::string("/default"));
  names_ = names_path + "/" + names_file;

  // Path to cfg file
  nodeHandle_.param("yolo_model/cfg_file/name", cfg_file, std::string("yolov3-tiny-custom.cfg"));
  nodeHandle_.param("cfg_path", cfg_path, std::string("/default"));
  cfg_ = cfg_path + "/" + cfg_file;

  // Path to weights file
  nodeHandle_.param("yolo_model/weights_file/name", weights_file, std::string("yolov3-tiny-tractor.weights"));
  nodeHandle_.param("weights_path", weights_path, std::string("/default"));
  weights_ = weights_path + "/" + weights_file;

  // Image parameters
  nodeHandle_.param("image/width", width_, 640);
  nodeHandle_.param("image/height", height_, 480);

  // Load common parameters
  nodeHandle_.param("image_view/enable_opencv", viewImage_, false);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 1);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  return true;
}

void YoloObjectDetectionNode::init(){
  gettimeofday(&startTime_, NULL);

  // Initialize publisher and subscriber
  std::string precCamTopicName;
  int precCamQueueSize;
  std::string frontCamTopicName;
  int frontCamQueueSize;
  std::string runYoloTopicName;
  int runYoloQueueSize;

  std::string boundingBoxTopicName;
  int boundingBoxQueueSize;

  nodeHandle_.param("subscribers/prec_camera_reading/topic", precCamTopicName, std::string("/preceding_truck_image"));
  nodeHandle_.param("subscribers/prec_camera_reading/queue_size", precCamQueueSize, 1);
  nodeHandle_.param("subscribers/front_camera_reading/topic", frontCamTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/front_camera_reading/queue_size", frontCamQueueSize, 1);
  nodeHandle_.param("subscribers/run_yolo_/topic", runYoloTopicName, std::string("/run_yolo_flag"));
  nodeHandle_.param("subscribers/run_yolo_/queue_size", runYoloQueueSize, 1);
  nodeHandle_.param("publishers/bounding_box/topic", boundingBoxTopicName, std::string("/yolo_object_detection/bounding_box"));
  nodeHandle_.param("publishers/bounding_box/queue_size", boundingBoxQueueSize, 1);

  yoloDetector_ = new Detector(cfg_, weights_, 0.2f/* thresh*/);
  objectNames_ = objectNames(names_);

  precCamImgSubscriber_ = imageTransport_.subscribe(precCamTopicName, precCamQueueSize, &YoloObjectDetectionNode::precCamImgCallback, this);
  frontCamImgSubscriber_ = imageTransport_.subscribe(frontCamTopicName, frontCamQueueSize, &YoloObjectDetectionNode::frontCamImgCallback, this);
  runYoloSubscriber_ = nodeHandle_.subscribe(runYoloTopicName, runYoloQueueSize, &YoloObjectDetectionNode::runYoloCallback, this); 
  boundingBoxPublisher_ = nodeHandle_.advertise<yolo_object_detection::bounding_box>(boundingBoxTopicName, boundingBoxQueueSize);

  cv::Mat img_for_init(width_, height_, CV_8UC3, cv::Scalar(0,0,0)); 
  yoloDetector_->detect(img_for_init);

  detectThread_ = std::thread(&YoloObjectDetectionNode::detectInThread, this);
}

std::vector<std::string> YoloObjectDetectionNode::objectNames(std::string const filename)
{
  std::ifstream file(filename);
  std::vector<std::string> file_lines;
  if (!file.is_open()) return file_lines;
  for(std::string line; getline(file, line);) file_lines.push_back(line);
  std::cout << "object names loaded\n";

  return file_lines;
}

void YoloObjectDetectionNode::runYoloCallback(const scale_truck_control::yolo_flag &msg){
  if (!run_yolo_){
    run_yolo_ = msg.run_yolo;
  }
}

void YoloObjectDetectionNode::precCamImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (run_yolo_){
    cv_bridge::CvImageConstPtr cv_ptr;
  
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
    if (cv_ptr) {
      std::scoped_lock lock(prec_cam_mutex_);
      precCamImageCopy_ = cv_ptr->image.clone();
      resize(precCamImageCopy_, precCamImageCopy_, cv::Size(width_, height_));
      sec_ = msg->header.stamp.sec;
      nsec_ = msg->header.stamp.nsec;
    }
  }
}

void YoloObjectDetectionNode::frontCamImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (run_yolo_){
    cv_bridge::CvImageConstPtr cv_ptr;
  
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
    if (cv_ptr) {
      std::scoped_lock lock(front_cam_mutex_);
      frontCamImageCopy_ = cv_ptr->image.clone();
      resize(frontCamImageCopy_, frontCamImageCopy_, cv::Size(width_, height_));
    }
  }
}

void YoloObjectDetectionNode::detectInThread()
{
  struct timeval endTime;
  static double time = 0.0;
//  static int cnt = 0;
  uint8_t mark = 0;
  while(ros::ok()){
    objects_.clear();
    if (run_yolo_){
      {
        std::scoped_lock lock(prec_cam_mutex_, front_cam_mutex_);
        std::string obj_name;
        if (!precCamImageCopy_.empty()) {
          objects_ = yoloDetector_->detect(precCamImageCopy_);
//          gettimeofday(&endTime, NULL);
//          cnt++;
//          time += ((endTime.tv_sec - sec) * 1000.0) + ((endTime.tv_usec - nsec) / 1000.0); //ms
//          delay_ = time / (double)cnt;
//          if (cnt > 3000){
//            time = 0.0;
//            cnt = 0;
//          }
//          delay_ = ((endTime.tv_sec - sec_) * 1000.0) + ((endTime.tv_usec - nsec_) / 1000.0); //ms   
          mark = 1;
        }
        else if (!frontCamImageCopy_.empty()) {
          objects_ = yoloDetector_->detect(frontCamImageCopy_);
	  mark = 2;
        }

        for(auto &i : objects_){
          if(objectNames_.size() > i.obj_id){
            obj_name = objectNames_[i.obj_id];
	  }
	}

        publishInThread(objects_, obj_name);
    
        if (viewImage_ && mark != 0) {
          cv::Mat draw_img;
          if (mark == 1) draw_img = precCamImageCopy_.clone();
	  else if (mark == 2) draw_img = frontCamImageCopy_.clone();
          drawBoxes(draw_img, objects_);
          cv::namedWindow("YOLO");
          cv::moveWindow("YOLO", 1280,520);
          if (!draw_img.empty()) {
            cv::imshow("YOLO", draw_img);
            cv::waitKey(waitKeyDelay_);
          }
        }
      }
//      recordData(startTime_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void YoloObjectDetectionNode::publishInThread(std::vector<bbox_t> objects, std::string obj_name)
{
  yolo_object_detection::bounding_box msg;
  unsigned int max_bbox_size = 0;
  bbox_t bbox;

  // Find max size bbox
  for (auto &i : objects) {
    if (max_bbox_size < i.w * i.h) {
      max_bbox_size = i.w * i.h;
      bbox = i;
    }
  }

  msg.name = obj_name;
  msg.x = bbox.x;
  msg.y = bbox.y;
  msg.w = bbox.w;
  msg.h = bbox.h;

  boundingBoxPublisher_.publish(msg);
}

void YoloObjectDetectionNode::drawBoxes(cv::Mat mat_img, std::vector<bbox_t> objects)
{
  int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

  for(auto &i : objects)
  {
    cv::Scalar color = obj_id_to_color(i.obj_id);
    cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);

    if(objectNames_.size() > i.obj_id)
    {
      std::string obj_name = objectNames_[i.obj_id];
      if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
      cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
      int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
      max_width = std::max(max_width, (int)i.w + 2);

      cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                    cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)), color, CV_FILLED, 8, 0);
      putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
    }
  }
}

void YoloObjectDetectionNode::recordData(struct timeval startTime){
  struct timeval currentTime;
  char file_name[] = "YOLO_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  double diff_time;
  std::ifstream read_file;
  std::ofstream write_file;
  std::string log_path = "/home/jetson/catkin_ws/logfiles/";
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[8] = i/10 + '0';  //ASCII
      file_name[9] = i%10 + '0';
      sprintf(file, "%s%s", log_path.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    write_file << "time,lvReqtoYoloDelay" << std::endl; //seconds, miliseconds
    flag = true;
  }
  else{
    gettimeofday(&currentTime, NULL);
    diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
    {
      std::scoped_lock lock(prec_cam_mutex_);
      sprintf(buf, "%.10e, %.10e", diff_time, delay_);
    }
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

} // namespace yolo_object_detection
