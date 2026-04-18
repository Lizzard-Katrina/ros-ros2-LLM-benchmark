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
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <clear_costmap_recovery/srv/clear_costmap.hpp>

#include <costmap_2d/testing_helper.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

std::shared_ptr<tf2_ros::Buffer> transformer;
std::shared_ptr<tf2_ros::TransformListener> tfl;

using costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name, 
                       double distance, 
                       bool obstacles, 
                       bool static_map,
                       costmap_2d::Costmap2DROS* global_costmap, 
                       costmap_2d::Costmap2DROS* local_costmap) {
    auto node = rclcpp::Node::make_shared("clear_costmap_client");
    auto client = node->create_client<clear_costmap_recovery::srv::ClearCostmap>("clear_costmap");

    auto request = std::make_shared<clear_costmap_recovery::srv::ClearCostmap::Request>();
    request->distance = distance;
    request->obstacles = obstacles;
    request->static_map = static_map;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Clear costmap request succeeded");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service clear_costmap");
    }
}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0) {
    costmap_2d::Costmap2DROS global(name + "/global", *transformer);
    costmap_2d::Costmap2DROS local(name + "/local", *transformer);
    std::shared_ptr<costmap_2d::ObstacleLayer> olayer;
        
    std::vector<std::shared_ptr<costmap_2d::Layer>>* plugins = global.getLayeredCostmap()->getPlugins();
    for (auto pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        std::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
        if (plugin->getName().find("obstacles") != std::string::npos) {
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
    ASSERT_EQ(countValues(*local.getCostmap(), LETHAL_OBSTACLE), local_lethal);
}

TEST(ClearTester, basicTest) {
    testCountLethal("base", 3.0, true, false, 20);
}

TEST(ClearTester, bigRadiusTest) {
    testCountLethal("base", 20.0, true, false, 22);
}

TEST(ClearTester, clearNoLayersTest) {
    testCountLethal("base", 20.0, false, false, 22);
}

TEST(ClearTester, clearBothTest) {
    testCountLethal("base", 3.0, true, true, 0);
}

TEST(ClearTester, clearBothTest2) {
    testCountLethal("base", 12.0, true, true, 6);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    transformer = std::make_shared<tf2_ros::Buffer>(rclcpp::Duration(10));
    tfl = std::make_shared<tf2_ros::TransformListener>(*transformer);
    return RUN_ALL_TESTS();
}
```