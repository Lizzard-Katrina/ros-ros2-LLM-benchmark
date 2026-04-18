# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>

#include <costmap_2d/testing_helper.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer* transformer;
tf2_ros::TransformListener* tfl;

using costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name, 
                       double distance, 
                       bool obstacles, 
                       bool static_map,
                       costmap_2d::Costmap2DROS* global_costmap, 
                       costmap_2d::Costmap2DROS* local_costmap){
// TODO: Replace the direct behavior invocation with a ROS2 request/response call.
// The implementation should issue a service request to trigger the clearing behavior,
// wait for completion, and handle success/failure appropriately.
//END OF TODO

}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0)
{
    costmap_2d::Costmap2DROS global(name + "/global", *transformer);
    costmap_2d::Costmap2DROS  local(name + "/local" , *transformer);
    boost::shared_ptr<costmap_2d::ObstacleLayer> olayer;
        
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = global.getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
        if(plugin->getName().find("obstacles")!=std::string::npos){
            olayer = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            addObservation(&(*olayer), 5.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
            addObservation(&(*olayer), 0.0, 5.0, MAX_Z/2, 0, 0, MAX_Z/2);
        }
    }
    
    global.updateMap();
    local.updateMap();
    olayer->clearStaticObservations(true, true);

    testClearBehavior("clear", distance, obstacles, static_map, &global, &local);
    
    global.updateMap();
    local.updateMap();
    
    printMap(*global.getCostmap());
    ASSERT_EQ(countValues(*global.getCostmap(), LETHAL_OBSTACLE), global_lethal);
    ASSERT_EQ(countValues( *local.getCostmap(), LETHAL_OBSTACLE),  local_lethal);
    
}

TEST(ClearTester, basicTest){
  testCountLethal("base", 3.0, true, false, 20);
}

TEST(ClearTester, bigRadiusTest){
  testCountLethal("base", 20.0, true, false, 22);
}

TEST(ClearTester, clearNoLayersTest){
  testCountLethal("base", 20.0, false, false, 22);
}

TEST(ClearTester, clearBothTest){
  testCountLethal("base", 3.0, true, true, 0);
}

TEST(ClearTester, clearBothTest2){
  testCountLethal("base", 12.0, true, true, 6);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "clear_tests");
  testing::InitGoogleTest(&argc, argv);
  transformer = new tf2_ros::Buffer(ros::Duration(10));
  tfl = new tf2_ros::TransformListener(*transformer);
  return RUN_ALL_TESTS();
}

----------------------------


# LLM Response

```cpp
#include <gtest/gtest.h>
#include <clear_costmap_recovery/clear_costmap_recovery.hpp>

#include <costmap_2d/testing_helper.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/clear_costmap_around_robot.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

tf2_ros::Buffer* transformer;
tf2_ros::TransformListener* tfl;
std::shared_ptr<rclcpp::Node> node;

using costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name, 
                       double distance, 
                       bool obstacles, 
                       bool static_map,
                       costmap_2d::Costmap2DROS* global_costmap, 
                       costmap_2d::Costmap2DROS* local_costmap){
    auto clear_local_client = node->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>(
        name + "/local/clear_costmap_around_robot");
    auto clear_global_client = node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        name + "/global/clear_entire_costmap");

    if (obstacles) {
        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
        request->reset_distance = distance;
        
        if (!clear_local_client->wait_for_service(std::chrono::seconds(5))) {
            FAIL() << "Service " << name + "/local/clear_costmap_around_robot" << " not available";
        }
        
        auto future = clear_local_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            FAIL() << "Failed to call service " << name + "/local/clear_costmap_around_robot";
        }
        
        auto response = future.get();
        if (!response->success) {
            FAIL() << "Service " << name + "/local/clear_costmap_around_robot" << " failed";
        }
    }

    if (static_map) {
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        request->request = std_msgs::msg::Empty();
        
        if (!clear_global_client->wait_for_service(std::chrono::seconds(5))) {
            FAIL() << "Service " << name + "/global/clear_entire_costmap" << " not available";
        }
        
        auto future = clear_global_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            FAIL() << "Failed to call service " << name + "/global/clear_entire_costmap";
        }
        
        auto response = future.get();
        if (!response->response.success) {
            FAIL() << "Service " << name + "/global/clear_entire_costmap" << " failed";
        }
    }
}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0)
{
    costmap_2d::Costmap2DROS global(name + "/global", *transformer);
    costmap_2d::Costmap2DROS  local(name + "/local" , *transformer);
    std::shared_ptr<costmap_2d::ObstacleLayer> olayer;
        
    std::vector<std::shared_ptr<costmap_2d::Layer> >* plugins = global.getLayeredCostmap()->getPlugins();
    for (std::vector<std::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        std::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
        if(plugin->getName().find("obstacles")!=std::string::npos){
            olayer = std::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            addObservation(&(*olayer), 5.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
            addObservation(&(*olayer), 0.0, 5.0, MAX_Z/2, 0, 0, MAX_Z/2);
        }
    }
    
    global.updateMap();
    local.updateMap();
    olayer->clearStaticObservations(true, true);

    testClearBehavior("clear", distance, obstacles, static_map, &global, &local);
    
    global.updateMap();
    local.updateMap();
    
    printMap(*global.getCostmap());
    ASSERT_EQ(countValues(*global.getCostmap(), LETHAL_OBSTACLE), global_lethal);
    ASSERT_EQ(countValues( *local.getCostmap(), LETHAL_OBSTACLE),  local_lethal);
    
}

TEST(ClearTester, basicTest){
  testCountLethal("base", 3.0, true, false, 20);
}

TEST(ClearTester, bigRadiusTest){
  testCountLethal("base", 20.0, true, false, 22);
}

TEST(ClearTester, clearNoLayersTest){
  testCountLethal("base", 20.0, false, false, 22);
}

TEST(ClearTester, clearBothTest){
  testCountLethal("base", 3.0, true, true, 0);
}

TEST(ClearTester, clearBothTest2){
  testCountLethal("base", 12.0, true, true, 6);
}


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  
  node = std::make_shared<rclcpp::Node>("clear_tests");
  transformer = new tf2_ros::Buffer(node->get_clock());
  tfl = new tf2_ros::TransformListener(*transformer);
  
  auto result = RUN_ALL_TESTS();
  
  rclcpp::shutdown();
  delete tfl;
  delete transformer;
  
  return result;
}
```