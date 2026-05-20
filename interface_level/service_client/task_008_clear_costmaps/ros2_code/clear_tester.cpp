#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/testing_helper.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

std::shared_ptr<tf2_ros::Buffer> transformer;
std::shared_ptr<tf2_ros::TransformListener> tfl;
std::shared_ptr<rclcpp::Node> test_node;

using nav2_costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name, 
                       double distance, 
                       bool obstacles, 
                       bool static_map,
                       nav2_costmap_2d::Costmap2DROS* global_costmap, 
                       nav2_costmap_2d::Costmap2DROS* local_costmap){
    auto client = test_node->create_client<nav2_msgs::srv::ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap");
    
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        ADD_FAILURE() << "Clear costmap service not available";
        return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(test_node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        SUCCEED() << "Cleared costmap successfully";
    } else {
        ADD_FAILURE() << "Failed to call clear costmap service";
    }
}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0)
{
    nav2_costmap_2d::Costmap2DROS global(name + "/global", "", true);
    nav2_costmap_2d::Costmap2DROS local(name + "/local", "", true);
    std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer;
        
    std::vector<std::shared_ptr<nav2_costmap_2d::Layer> >* plugins = global.getLayeredCostmap()->getPlugins();
    for (auto pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        std::shared_ptr<nav2_costmap_2d::Layer> plugin = *pluginp;
        if(plugin->getName().find("obstacles")!=std::string::npos){
            olayer = std::static_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
            addObservation(&(*olayer), 5.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
            addObservation(&(*olayer), 0.0, 5.0, MAX_Z/2, 0, 0, MAX_Z/2);
        }
    }
    
    global.updateMap();
    local.updateMap();
    if (olayer) {
        olayer->clearStaticObservations(true, true);
    }

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
  test_node = std::make_shared<rclcpp::Node>("clear_tests_node");
  transformer = std::make_shared<tf2_ros::Buffer>(test_node->get_clock());
  tfl = std::make_shared<tf2_ros::TransformListener>(*transformer);
  
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}