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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/testing_helper.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nav2_msgs/srv/clear_costmaps.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

rclcpp::Node::SharedPtr g_node;
std::shared_ptr<tf2_ros::Buffer> transformer;
std::shared_ptr<tf2_ros::TransformListener> tfl;

using nav2_costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name, 
                       double distance, 
                       bool obstacles, 
                       bool static_map,
                       nav2_costmap_2d::Costmap2DROS* global_costmap, 
                       nav2_costmap_2d::Costmap2DROS* local_costmap){
    auto client = g_node->create_client<nav2_msgs::srv::ClearCostmaps>(
        global_costmap->get_name() + "/clear_costmaps");
    
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        ASSERT_TRUE(false) << "Service not available";
        return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearCostmaps::Request>();
    request->base_frame = "base_link";
    
    auto future = client->async_send_request(request);
    
    rclcpp::spin_some(*global_costmap);
    rclcpp::spin_some(*local_costmap);
    
    auto status = rclcpp::spin_until_future_complete(g_node, future);
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0)
{
    nav2_costmap_2d::Costmap2DROS global(name + "/global", g_node, transformer);
    nav2_costmap_2d::Costmap2DROS  local(name + "/local" , g_node, transformer);
    std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer;
        
    std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>* plugins = global.getLayeredCostmap()->getPlugins();
    for (std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        std::shared_ptr<nav2_costmap_2d::Layer> plugin = *pluginp;
        if(plugin->getName().find("obstacles")!=std::string::npos){
            olayer = std::static_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
            nav2_costmap_2d::addObservation(&(*olayer), 5.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
            nav2_costmap_2d::addObservation(&(*olayer), 0.0, 5.0, MAX_Z/2, 0, 0, MAX_Z/2);
        }
    }
    
    global.updateMap();
    local.updateMap();
    olayer->clearStaticObservations(true, true);

    testClearBehavior("clear", distance, obstacles, static_map, &global, &local);
    
    global.updateMap();
    local.updateMap();
    
    nav2_costmap_2d::printMap(*global.getCostmap());
    ASSERT_EQ(nav2_costmap_2d::countValues(*global.getCostmap(), LETHAL_OBSTACLE), global_lethal);
    ASSERT_EQ(nav2_costmap_2d::countValues( *local.getCostmap(), LETHAL_OBSTACLE),  local_lethal);
    
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
  g_node = std::make_shared<rclcpp::Node>("clear_tests");
  transformer = std::make_shared<tf2_ros::Buffer>(g_node->get_clock());
  tfl = std::make_shared<tf2_ros::TransformListener>(*transformer);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}