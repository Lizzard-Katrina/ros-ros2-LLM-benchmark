/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>

#include <algorithm>
#include <utility>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros
{

char * cfg;
char * weights;
char * data;
char ** detectionNames;

YoloObjectDetector::YoloObjectDetector(const rclcpp::NodeOptions & options)
: Node("yolo_object_detector", options),
  imageTransport_(this),
  numClasses_(0),
  classLabels_(0),
  rosBoxes_(0),
  rosBoxCounter_(0)
{
  RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    rclcpp::shutdown();
  }

  init();
}

YoloObjectDetector::~YoloObjectDetector()
{
  {
    std::unique_lock<std::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Load common parameters.
  viewImage_ = this->declare_parameter<bool>("image_view.enable_opencv", true);
  waitKeyDelay_ = this->declare_parameter<int>("image_view.wait_key_delay", 3);
  enableConsoleOutput_ = this->declare_parameter<bool>("image_view.enable_console_output", false);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(nullptr)) {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Xserver is running.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // Set vector sizes.
  classLabels_ = this->declare_parameter<std::vector<std::string>>(
    "yolo_model.detection_classes.names", std::vector<std::string>{});
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_>>(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init()
{
  RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh = this->declare_parameter<float>("yolo_model.threshold.value", 0.3f);

  // Path to weights file.
  weightsModel = this->declare_parameter<std::string>("yolo_model.weight_file.name", "yolov2-tiny.weights");
  weightsPath = this->declare_parameter<std::string>("weights_path", "/default");
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  configModel = this->declare_parameter<std::string>("yolo_model.config_file.name", "yolov2-tiny.cfg");
  configPath = this->declare_parameter<std::string>("config_path", "/default");
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char **)realloc((void *)detectionNames, (numClasses_ + 1) * sizeof(char *));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0);
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;

  cameraTopicName = this->declare_parameter<std::string>("subscribers.camera_reading.topic", "/camera/image_raw");
  cameraQueueSize = this->declare_parameter<int>("subscribers.camera_reading.queue_size", 1);
  objectDetectorTopicName = this->declare_parameter<std::string>("publishers.object_detector.topic", "found_object");
  objectDetectorQueueSize = this->declare_parameter<int>("publishers.object_detector.queue_size", 1);
  objectDetectorLatch = this->declare_parameter<bool>("publishers.object_detector.latch", false);
  boundingBoxesTopicName = this->declare_parameter<std::string>("publishers.bounding_boxes.topic", "bounding_boxes");
  boundingBoxesQueueSize = this->declare_parameter<int>("publishers.bounding_boxes.queue_size", 1);
  boundingBoxesLatch = this->declare_parameter<bool>("publishers.bounding_boxes.latch", false);
  detectionImageTopicName = this->declare_parameter<std::string>("publishers.detection_image.topic", "detection_image");
  detectionImageQueueSize = this->declare_parameter<int>("publishers.detection_image.queue_size", 1);
  detectionImageLatch = this->declare_parameter<bool>("publishers.detection_image.latch", true);

  imageSubscriber_ = image_transport::create_subscription(
    this,
    cameraTopicName,
    std::bind(&YoloObjectDetector::cameraCallback, this, std::placeholders::_1),
    "raw");

  rclcpp::QoS object_qos(objectDetectorQueueSize);
  if (objectDetectorLatch) {
    object_qos.transient_local();
  }
  objectPublisher_ = this->create_publisher<darknet_ros_msgs::msg::ObjectCount>(objectDetectorTopicName, object_qos);

  rclcpp::QoS boxes_qos(boundingBoxesQueueSize);
  if (boundingBoxesLatch) {
    boxes_qos.transient_local();
  }
  boundingBoxesPublisher_ = this->create_publisher<darknet_ros_msgs::msg::BoundingBoxes>(boundingBoxesTopicName, boxes_qos);

  rclcpp::QoS image_qos(detectionImageQueueSize);
  if (detectionImageLatch) {
    image_qos.transient_local();
  }
  detectionImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>(detectionImageTopicName, image_qos);

  // Action servers.
  std::string checkForObjectsActionName;
  checkForObjectsActionName = this->declare_parameter<std::string>("actions.camera_reading.topic", "check_for_objects");
  checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(shared_from_this(), checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(std::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(std::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (!cam_image) {
    return;
  }

  {
    std::lock_guard<std::mutex> lockImageCallback(mutexImageCallback_);
    camImageCopy_ = cam_image->image.clone();
    imageHeader_ = msg->header;
  }

  {
    std::lock_guard<std::mutex> lockImageStatus(mutexImageStatus_);
    imageStatus_ = true;
  }

  frameWidth_ = cam_image->image.size().width;
  frameHeight_ = cam_image->image.size().height;
}

void YoloObjectDetector::checkForObjectsActionGoalCB()
{
  RCLCPP_DEBUG(this->get_logger(), "[YoloObjectDetector] Start check for objects action.");

  auto imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::msg::Image imageAction = imageActionPtr->image;

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      std::unique_lock<std::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
      imageHeader_ = imageAction.header;
    }
    {
      std::unique_lock<std::shared_mutex> lockImageCallback(mutexActionStatus_);
      actionId_ = imageActionPtr->id;
    }
    {
      std::unique_lock<std::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}

void YoloObjectDetector::checkForObjectsActionPreemptCB()
{
  RCLCPP_DEBUG(this->get_logger(), "[YoloObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

bool YoloObjectDetector::isCheckingForObjects() const
{
  return (rclcpp::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested());
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat & detectionImage)
{
  if (detectionImagePublisher_->get_subscription_count() < 1) {
    return false;
  }
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = this->now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_->publish(*cvImage.toImageMsg());
  RCLCPP_DEBUG(this->get_logger(), "Detection image has been published.");
  return true;
}

int YoloObjectDetector::sizeNetwork(network * net)
{
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network * net)
{
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection * YoloObjectDetector::avgPredictions(network * net, int * nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for (j = 0; j < demoFrame_; ++j) {
    axpy_cpu(demoTotal_, 1. / demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection * dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}

void * YoloObjectDetector::detectInThread()
{
  running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float * X = buffLetter_[(buffIndex_ + 2) % 3].data;
  network_predict(net_, X);

  rememberNetwork(net_);
  detection * dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) {
    do_nms_obj(dets, nboxes, l.classes, nms);
  }

  if (enableConsoleOutput_) {
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f\n", fps_);
    printf("Objects:\n\n");
  }
  image display = buff_[(buffIndex_ + 2) % 3];
  draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

  int i, j;
  int count = 0;
  for (i = 0; i < nboxes; ++i) {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if (xmin < 0) xmin = 0;
    if (ymin < 0) ymin = 0;
    if (xmax > 1) xmax = 1;
    if (ymax > 1) ymax = 1;

    for (j = 0; j < demoClasses_; ++j) {
      if (dets[i].prob[j]) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          roiBoxes_[count].x = x_center;
          roiBoxes_[count].y = y_center;
          roiBoxes_[count].w = BoundingBox_width;
          roiBoxes_[count].h = BoundingBox_height;
          roiBoxes_[count].Class = j;
          roiBoxes_[count].prob = dets[i].prob[j];
          count++;
        }
      }
    }
  }

  if (count == 0) {
    roiBoxes_[0].num = 0;
  } else {
    roiBoxes_[0].num = count;
  }

  free_detections(dets, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  running_ = 0;
  return 0;
}

void * YoloObjectDetector::fetchInThread()
{
  {
    std::shared_lock<std::shared_mutex> lock(mutexImageCallback_);
    CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();
    free_image(buff_[buffIndex_]);
    buff_[buffIndex_] = mat_to_image(imageAndHeader.image);
    headerBuff_[buffIndex_] = imageAndHeader.header;
    buffId_[buffIndex_] = actionId_;
  }
  rgbgr_image(buff_[buffIndex_]);
  letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  return 0;
}

void * YoloObjectDetector::displayInThread(void * ptr)
{
  int c = show_image(buff_[(buffIndex_ + 1) % 3], "YOLO", 1);
  if (c != -1) c = c % 256;
  if (c == 27) {
    demoDone_ = 1;
    return 0;
  } else if (c == 82) {
    demoThresh_ += .02;
  } else if (c == 84) {
    demoThresh_ -= .02;
    if (demoThresh_ <= .02) demoThresh_ = .02;
  } else if (c == 83) {
    demoHier_ += .02;
  } else if (c == 81) {
    demoHier_ -= .02;
    if (demoHier_ <= .0) demoHier_ = .0;
  }
  (void)ptr;
  return 0;
}

void * YoloObjectDetector::displayLoop(void * ptr)
{
  while (1) {
    displayInThread(0);
  }
  (void)ptr;
}

void * YoloObjectDetector::detectLoop(void * ptr)
{
  while (1) {
    detectInThread();
  }
  (void)ptr;
}

void YoloObjectDetector::setupNetwork(
  char * cfgfile, char * weightfile, char * datafile, float thresh, char ** names, int classes, int delay,
  char * prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
{
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image ** alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
  (void)w;
  (void)h;
  (void)frames;
}

void YoloObjectDetector::yolo()
{
  const auto wait_duration = std::chrono::milliseconds(2000);
  while (!getImageStatus()) {
    printf("Waiting for image.\n");
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread detect_thread;
  std::thread fetch_thread;

  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float **)calloc(demoFrame_, sizeof(float *));
  for (i = 0; i < demoFrame_; ++i) {
    predictions_[i] = (float *)calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float *)calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_ *)calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  {
    std::shared_lock<std::shared_mutex> lock(mutexImageCallback_);
    CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();
    buff_[0] = mat_to_image(imageAndHeader.image);
    headerBuff_[0] = imageAndHeader.header;
  }
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  headerBuff_[1] = headerBuff_[0];
  headerBuff_[2] = headerBuff_[0];
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  disp_ = image_to_mat(buff_[0]);

  int count = 0;
  if (!demoPrefix_ && viewImage_) {
    cv::namedWindow("YOLO", cv::WINDOW_NORMAL);
    if (fullScreen_) {
      cv::setWindowProperty("YOLO", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    } else {
      cv::moveWindow("YOLO", 0, 0);
      cv::resizeWindow("YOLO", 640, 480);
    }
  }

  demoTime_ = what_time_is_it_now();

  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
    if (!demoPrefix_) {
      fps_ = 1. / (what_time_is_it_now() - demoTime_);
      demoTime_ = what_time_is_it_now();
      if (viewImage_) {
        displayInThread(0);
      } else {
        generate_image(buff_[(buffIndex_ + 1) % 3], disp_);
      }
      publishInThread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demoPrefix_, count);
      save_image(buff_[(buffIndex_ + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }
}

CvMatWithHeader_ YoloObjectDetector::getCvMatWithHeader()
{
  CvMatWithHeader_ header = {.image = camImageCopy_, .header = imageHeader_};
  return header;
}

bool YoloObjectDetector::getImageStatus(void)
{
  std::shared_lock<std::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  std::shared_lock<std::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void * YoloObjectDetector::publishInThread()
{
  darknet_ros_msgs::msg::BoundingBoxes boundingBoxesMsg;
  darknet_ros_msgs::msg::ObjectCount objectCountMsg;

  std_msgs::msg::Header local_header;
  std::vector<darknet_ros_msgs::msg::BoundingBox> local_boxes;

  {
    std::lock_guard<std::mutex> lock(mutexImageCallback_);

    local_header = imageHeader_;
    const int num = std::max(0, roiBoxes_[0].num);
    local_boxes.reserve(static_cast<size_t>(num));

    for (int i = 0; i < num; ++i) {
      const auto & rb = roiBoxes_[i];

      float xmin_n = rb.x - rb.w / 2.0f;
      float xmax_n = rb.x + rb.w / 2.0f;
      float ymin_n = rb.y - rb.h / 2.0f;
      float ymax_n = rb.y + rb.h / 2.0f;

      xmin_n = std::clamp(xmin_n, 0.0f, 1.0f);
      xmax_n = std::clamp(xmax_n, 0.0f, 1.0f);
      ymin_n = std::clamp(ymin_n, 0.0f, 1.0f);
      ymax_n = std::clamp(ymax_n, 0.0f, 1.0f);

      darknet_ros_msgs::msg::BoundingBox bbox;
      bbox.xmin = static_cast<int>(xmin_n * static_cast<float>(frameWidth_));
      bbox.xmax = static_cast<int>(xmax_n * static_cast<float>(frameWidth_));
      bbox.ymin = static_cast<int>(ymin_n * static_cast<float>(frameHeight_));
      bbox.ymax = static_cast<int>(ymax_n * static_cast<float>(frameHeight_));
      bbox.probability = rb.prob;
      bbox.id = rb.Class;
      if (rb.Class >= 0 && rb.Class < static_cast<int>(classLabels_.size())) {
        bbox.class_id = classLabels_[rb.Class];
      } else {
        bbox.class_id = "unknown";
      }

      local_boxes.emplace_back(std::move(bbox));
    }
  }

  boundingBoxesMsg.header = local_header;
  boundingBoxesMsg.header.stamp = imageHeader_.stamp;
  boundingBoxesMsg.image_header = local_header;
  boundingBoxesMsg.image_header.stamp = imageHeader_.stamp;
  boundingBoxesMsg.bounding_boxes = std::move(local_boxes);

  objectCountMsg.header = boundingBoxesMsg.header;
  objectCountMsg.count = static_cast<int64_t>(boundingBoxesMsg.bounding_boxes.size());

  boundingBoxesPublisher_->publish(std::move(boundingBoxesMsg));
  objectPublisher_->publish(std::move(objectCountMsg));

  if (!publishDetectionImage(disp_)) {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] No subscribers for detection image.");
  }

  return 0;
}

}  // namespace darknet_ros