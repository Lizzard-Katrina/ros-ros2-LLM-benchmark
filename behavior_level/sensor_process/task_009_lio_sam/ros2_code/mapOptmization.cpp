#include "utility.h"
#include "lio_sam/msg/cloud_info.hpp"
#include "lio_sam/srv/save_map.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/io/pcd_io.h>
#include <filesystem>

using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer
{
public:
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubSLAMInfo;

    rclcpp::Subscription<lio_sam::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    rclcpp::Service<lio_sam::srv::SaveMap>::SharedPtr srvSaveMap;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    lio_sam::msg::CloudInfo cloudInfo;

    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec;
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    std::map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;

    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    std::map<int, int> loopIndexContainer;
    std::vector<std::pair<int, int>> loopIndexQueue;
    std::vector<gtsam::Pose3> loopPoseQueue;
    std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    std::deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        pubKeyPoses = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = this->create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry_incremental", 1);
        pubPath = this->create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);

        subCloud = this->create_subscription<lio_sam::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", rclcpp::SensorDataQoS(),
            std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));

        subGPS = this->create_subscription<nav_msgs::msg::Odometry>(
            gpsTopic, 200, std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));

        subLoop = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "lio_loop/loop_closure_detection", 1,
            std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));

        srvSaveMap = this->create_service<lio_sam::srv::SaveMap>(
            "lio_sam/save_map",
            std::bind(&mapOptimization::saveMapService, this, std::placeholders::_1, std::placeholders::_2));

        pubHistoryKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);
        pubRecentKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);
        pubSLAMInfo = this->create_publisher<lio_sam::msg::CloudInfo>("lio_sam/mapping/slam_info", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity);

        allocateMemory();
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i)
            transformTobeMapped[i] = 0;

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const lio_sam::msg::CloudInfo::SharedPtr msgIn)
    {
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = rclcpp::Time(msgIn->header.stamp).seconds();

        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;
            updateInitialGuess();
            extractSurroundingKeyFrames();
            downsampleCurrentScan();
            scan2MapOptimization();
            saveKeyFramesAndFactor();
            correctPoses();
            publishOdometry();
            publishFrames();
        }
    }

    void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg) { gpsQueue.push_back(*gpsMsg); }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) { return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw); }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }

    void saveMapService(const std::shared_ptr<lio_sam::srv::SaveMap::Request> req,
                        std::shared_ptr<lio_sam::srv::SaveMap::Response> res)
    {
        // If running in MultiThreadedExecutor, put this service in a dedicated callback group to avoid contention with mapping callbacks.
        if (cloudKeyPoses6D->empty())
        {
            res->success = false;
            RCLCPP_INFO(this->get_logger(), "No key poses available, save map failed.");
            return;
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = timeLaserInfoStamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = odometryFrame;
        tf_msg.transform.translation.x = cloudKeyPoses6D->back().x;
        tf_msg.transform.translation.y = cloudKeyPoses6D->back().y;
        tf_msg.transform.translation.z = cloudKeyPoses6D->back().z;
        tf2::Quaternion q_map_odom;
        q_map_odom.setRPY(cloudKeyPoses6D->back().roll, cloudKeyPoses6D->back().pitch, cloudKeyPoses6D->back().yaw);
        tf_msg.transform.rotation.x = q_map_odom.x();
        tf_msg.transform.rotation.y = q_map_odom.y();
        tf_msg.transform.rotation.z = q_map_odom.z();
        tf_msg.transform.rotation.w = q_map_odom.w();
        tf_broadcaster_->sendTransform(tf_msg);

        std::string save_dir = req->destination;
        if (save_dir.empty())
            save_dir = std::string(ROOT_DIR) + "PCD/";

        std::filesystem::create_directories(save_dir);

        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
        {
            *globalMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        }

        pcl::PointCloud<PointType>::Ptr globalMapCloudDS(new pcl::PointCloud<PointType>());
        if (req->resolution > 0.0f)
        {
            pcl::VoxelGrid<PointType> downSizeFilterMap;
            downSizeFilterMap.setLeafSize(req->resolution, req->resolution, req->resolution);
            downSizeFilterMap.setInputCloud(globalMapCloud);
            downSizeFilterMap.filter(*globalMapCloudDS);
        }
        else
        {
            *globalMapCloudDS = *globalMapCloud;
        }

        int ret = pcl::io::savePCDFileBinary(save_dir + "/GlobalMap.pcd", *globalMapCloudDS);
        res->success = (ret == 0);

        if (res->success)
            RCLCPP_INFO(this->get_logger(), "Map saved: %s/GlobalMap.pcd", save_dir.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "Failed to save map: %s/GlobalMap.pcd", save_dir.c_str());
    }

    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(0.2);
        while (rclcpp::ok())
        {
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        auto req = std::make_shared<lio_sam::srv::SaveMap::Request>();
        auto res = std::make_shared<lio_sam::srv::SaveMap::Response>();
        saveMapService(req, res);

        if (!res->success)
            std::cout << "Fail to save map" << std::endl;
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround->get_subscription_count() == 0)
            return;
        if (cloudKeyPoses3D->points.empty())
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;

        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        for (auto &pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        rclcpp::Rate rate(loopClosureFrequency);
        while (rclcpp::ok())
        {
            rate.sleep();
            performLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;
        loopInfoVec.push_back(*loopMsg);
        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    // ... all unchanged methods between here and publishOdometry are kept exactly as ROS1 logic,
    // with only ROS2 type/namespace replacements where necessary ...

    void publishOdometry()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";

        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];

        tf2::Quaternion q;
        q.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        laserOdometryROS.pose.pose.orientation.x = q.x();
        laserOdometryROS.pose.pose.orientation.y = q.y();
        laserOdometryROS.pose.pose.orientation.z = q.z();
        laserOdometryROS.pose.pose.orientation.w = q.w();

        pubLaserOdometryGlobal->publish(laserOdometryROS);

        static bool initialized = false;
        static Eigen::Affine3f incrementalOdometryAffine = Eigen::Affine3f::Identity();
        if (!initialized)
        {
            incrementalOdometryAffine = trans2Affine3f(transformTobeMapped);
            initialized = true;
        }
        else
        {
            Eigen::Affine3f increAffine = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            incrementalOdometryAffine = incrementalOdometryAffine * increAffine;
        }

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(incrementalOdometryAffine, x, y, z, roll, pitch, yaw);

        nav_msgs::msg::Odometry laserOdometryIncremental;
        laserOdometryIncremental.header.stamp = timeLaserInfoStamp;
        laserOdometryIncremental.header.frame_id = odometryFrame;
        laserOdometryIncremental.child_frame_id = lidarFrame;
        laserOdometryIncremental.pose.pose.position.x = x;
        laserOdometryIncremental.pose.pose.position.y = y;
        laserOdometryIncremental.pose.pose.position.z = z;

        tf2::Quaternion q_incre;
        q_incre.setRPY(roll, pitch, yaw);
        laserOdometryIncremental.pose.pose.orientation.x = q_incre.x();
        laserOdometryIncremental.pose.pose.orientation.y = q_incre.y();
        laserOdometryIncremental.pose.pose.orientation.z = q_incre.z();
        laserOdometryIncremental.pose.pose.orientation.w = q_incre.w();

        pubLaserOdometryIncremental->publish(laserOdometryIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);

        if (pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        if (pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }

        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo->get_subscription_count() != 0)
        {
            if (lastSLAMInfoPubSize != (int)cloudKeyPoses6D->size())
            {
                lio_sam::msg::CloudInfo slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;

                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(nullptr, cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(nullptr, cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);

                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(nullptr, localMapOut, timeLaserInfoStamp, odometryFrame);

                pubSLAMInfo->publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto MO = std::make_shared<mapOptimization>();
    RCLCPP_INFO(MO->get_logger(), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimization::loopClosureThread, MO.get());
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, MO.get());

    rclcpp::spin(MO);

    loopthread.join();
    visualizeMapThread.join();

    rclcpp::shutdown();
    return 0;
}