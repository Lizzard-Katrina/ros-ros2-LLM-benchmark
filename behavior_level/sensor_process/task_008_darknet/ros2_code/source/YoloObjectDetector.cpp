/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 *  Migrated to ROS 2 Humble.
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>

#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char* cfg;
char* weights;
char* data;
char** detectionNames;

YoloObjectDetector::YoloObjectDetector(rclcpp::Node::SharedPtr nh)
    : node_(nh), numClasses_(0), classLabels_(0), rosBoxes_(0), rosBoxCounter_(0) {
  RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    rclcpp::shutdown();
  }

  init();
}

YoloObjectDetector::~YoloObjectDetector() {
  {
    std::lock_guard<std::mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

rclcpp::Logger YoloObjectDetector::get_logger() const {
  return node_->get_logger();
}

rclcpp::Time YoloObjectDetector::now() const {
  return node_->now();
}

bool YoloObjectDetector::readParameters() {
  // Load common parameters.
  node_->declare_parameter("image_view/enable_opencv", true);
  node_->declare_parameter("image_view/wait_key_delay", 3);
  node_->declare_parameter("image_view/enable_console_output", false);

  node_->get_parameter("image_view/enable_opencv", viewImage_);
  node_->get_parameter("image_view/wait_key_delay", waitKeyDelay_);
  node_->get_parameter("image_view/enable_console_output", enableConsoleOutput_);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Xserver is running.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // Set vector sizes.
  node_->declare_parameter("yolo_model/detection_classes/names", std::vector<std::string>(0));
  node_->get_parameter("yolo_model/detection_classes/names", classLabels_);
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init() {
  RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  node_->declare_parameter("yolo_model/threshold/value", 0.3f);
  node_->get_parameter("yolo_model/threshold/value", thresh);

  // Path to weights file.
  node_->declare_parameter("yolo_model/weight_file/name", std::string("yolov2-tiny.weights"));
  node_->declare_parameter("weights_path", std::string("/default"));
  node_->get_parameter("yolo_model/weight_file/name", weightsModel);
  node_->get_parameter("weights_path", weightsPath);
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  node_->declare_parameter("yolo_model/config_file/name", std::string("yolov2-tiny.cfg"));
  node_->declare_parameter("config_path", std::string("/default"));
  node_->get_parameter("yolo_model/config_file/name", configModel);
  node_->get_parameter("config_path", configPath);
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**)realloc((void*)detectionNames, (numClasses_ + 1) * sizeof(char*));
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
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;

  node_->declare_parameter("subscribers/camera_reading/topic", std::string("/camera/image_raw"));
  node_->declare_parameter("subscribers/camera_reading/queue_size", 1);
  node_->declare_parameter("publishers/object_detector/topic", std::string("found_object"));
  node_->declare_parameter("publishers/object_detector/queue_size", 1);
  node_->declare_parameter("publishers/bounding_boxes/topic", std::string("bounding_boxes"));
  node_->declare_parameter("publishers/bounding_boxes/queue_size", 1);
  node_->declare_parameter("publishers/detection_image/topic", std::string("detection_image"));
  node_->declare_parameter("publishers/detection_image/queue_size", 1);

  node_->get_parameter("subscribers/camera_reading/topic", cameraTopicName);
  node_->get_parameter("subscribers/camera_reading/queue_size", cameraQueueSize);
  node_->get_parameter("publishers/object_detector/topic", objectDetectorTopicName);
  node_->get_parameter("publishers/object_detector/queue_size", objectDetectorQueueSize);
  node_->get_parameter("publishers/bounding_boxes/topic", boundingBoxesTopicName);
  node_->get_parameter("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize);
  node_->get_parameter("publishers/detection_image/topic", detectionImageTopicName);
  node_->get_parameter("publishers/detection_image/queue_size", detectionImageQueueSize);

  imageSubscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
      cameraTopicName, cameraQueueSize,
      std::bind(&YoloObjectDetector::cameraCallback, this, std::placeholders::_1));

  boundingBoxesPublisher_ =
      node_->create_publisher<darknet_ros_msgs::msg::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize);
  objectPublisher_ =
      node_->create_publisher<darknet_ros_msgs::msg::ObjectCount>(objectDetectorTopicName, objectDetectorQueueSize);
  detectionImagePublisher_ =
      node_->create_publisher<sensor_msgs::msg::Image>(detectionImageTopicName, detectionImageQueueSize);
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      std::lock_guard<std::mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
    }
    {
      std::lock_guard<std::mutex> lockImageStatus(mutexImageStatus_);
      imageHeader_ = msg->header;
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage) {
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = imageHeader_.stamp;
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_->publish(*cvImage.toImageMsg());
  RCLCPP_DEBUG(this->get_logger(), "Detection image has been published.");
  return true;
}

int YoloObjectDetector::sizeNetwork(network* net) {
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

void YoloObjectDetector::rememberNetwork(network* net) {
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

detection* YoloObjectDetector::avgPredictions(network* net, int* nboxes) {
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
  detection* dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}

void* YoloObjectDetector::detectInThread() {
  running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float* X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float* prediction = network_predict(net_, X);

  rememberNetwork(net_);
  detection* dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  if (enableConsoleOutput_) {
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f\n", fps_);
    printf("Objects:\n\n");
  }
  image display = buff_[(buffIndex_ + 2) % 3];
  draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

  // extract the bounding boxes and send them to ROS
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

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < demoClasses_; ++j) {
      if (dets[i].prob[j]) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
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

  // create array to store found bounding boxes
  // if no object detected, make sure that ROS knows that num = 0
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

void* YoloObjectDetector::fetchInThread() {
  {
    std::lock_guard<std::mutex> lock(mutexImageCallback_);
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

void* YoloObjectDetector::displayInThread(void* ptr) {
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
  return 0;
}

void* YoloObjectDetector::displayLoop(void* ptr) {
  while (1) {
    displayInThread(0);
  }
}

void* YoloObjectDetector::detectLoop(void* ptr) {
  while (1) {
    detectInThread();
  }
}

void YoloObjectDetector::setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes, int delay,
                                      char* prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen) {
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image** alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo() {
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
  predictions_ = (float**)calloc(demoFrame_, sizeof(float*));
  for (i = 0; i < demoFrame_; ++i) {
    predictions_[i] = (float*)calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float*)calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_*)calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  {
    std::lock_guard<std::mutex> lock(mutexImageCallback_);
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

CvMatWithHeader_ YoloObjectDetector::getCvMatWithHeader() {
  CvMatWithHeader_ header = {.image = camImageCopy_, .header = imageHeader_};
  return header;
}

bool YoloObjectDetector::getImageStatus(void) {
  std::lock_guard<std::mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void) {
  std::lock_guard<std::mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void* YoloObjectDetector::publishInThread() {
  // Protect access to roiBoxes_ with mutex
  std::lock_guard<std::mutex> lockRoiBoxes(mutexImageCallback_);

  // Read the number of detected objects
  int num = roiBoxes_[0].num;

  // Prepare BoundingBoxes message
  auto boundingBoxesMsg = std::make_unique<darknet_ros_msgs::msg::BoundingBoxes>();
  boundingBoxesMsg->header.stamp = imageHeader_.stamp;
  boundingBoxesMsg->header.frame_id = "detection";

  // Prepare ObjectCount message
  auto objectCountMsg = std::make_unique<darknet_ros_msgs::msg::ObjectCount>();
  objectCountMsg->header.stamp = imageHeader_.stamp;
  objectCountMsg->header.frame_id = "detection";
  objectCountMsg->count = num;

  if (num > 0) {
    for (int i = 0; i < num; i++) {
      darknet_ros_msgs::msg::BoundingBox boundingBox;

      boundingBox.class_id = classLabels_[roiBoxes_[i].Class];
      boundingBox.probability = roiBoxes_[i].prob;

      // Scale normalized coordinates to pixel coordinates
      boundingBox.xmin = (roiBoxes_[i].x - roiBoxes_[i].w / 2.0) * frameWidth_;
      boundingBox.ymin = (roiBoxes_[i].y - roiBoxes_[i].h / 2.0) * frameHeight_;
      boundingBox.xmax = (roiBoxes_[i].x + roiBoxes_[i].w / 2.0) * frameWidth_;
      boundingBox.ymax = (roiBoxes_[i].y + roiBoxes_[i].h / 2.0) * frameHeight_;

      boundingBoxesMsg->bounding_boxes.push_back(boundingBox);
    }
  }

  boundingBoxesPublisher_->publish(std::move(boundingBoxesMsg));
  objectPublisher_->publish(std::move(objectCountMsg));

  if (num == 0) {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] No objects detected.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[YoloObjectDetector] Detected %d object(s).", num);
  }

  return 0;
}

} /* namespace darknet_ros*/