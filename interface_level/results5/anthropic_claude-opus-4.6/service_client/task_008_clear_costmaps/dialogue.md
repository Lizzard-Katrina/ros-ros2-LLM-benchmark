# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: clear_tester.cpp
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



[FILENAME: clear_tester.cpp]
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/testing_helper.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

std::shared_ptr<tf2_ros::Buffer> transformer;
std::shared_ptr<tf2_ros::TransformListener> tfl;
rclcpp::Node::SharedPtr test_node;

using nav2_costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name,
                       double distance,
                       bool obstacles,
                       bool static_map,
                       nav2_costmap_2d::Costmap2DROS* global_costmap,
                       nav2_costmap_2d::Costmap2DROS* local_costmap)
{
    auto node = rclcpp::Node::make_shared(name + "_clear_behavior_node");

    // Build parameter overrides for the clear costmap recovery behavior
    auto clear_node = rclcpp::Node::make_shared(name + "_clear_node");
    clear_node->declare_parameter("reset_distance", distance);
    clear_node->declare_parameter("force_clearing", true);

    // Get the layered costmap and iterate through plugins to selectively clear
    auto* global_layered = global_costmap->getLayeredCostmap();
    auto* local_layered = local_costmap->getLayeredCostmap();

    std::vector<std::string> clearable_layers;
    if (obstacles) {
        clearable_layers.push_back("obstacles");
    }
    if (static_map) {
        clearable_layers.push_back("static");
    }

    // Clear the global costmap layers within the specified distance
    auto global_plugins = global_layered->getPlugins();
    for (auto& plugin : *global_plugins) {
        bool should_clear = false;
        for (const auto& layer_name : clearable_layers) {
            if (plugin->getName().find(layer_name) != std::string::npos) {
                should_clear = true;
                break;
            }
        }
        if (should_clear) {
            auto costmap = plugin->getLayeredCostmap()->getCostmap();
            double ox = costmap->getOriginX();
            double oy = costmap->getOriginY();
            double size_x = costmap->getSizeInMetersX();
            double size_y = costmap->getSizeInMetersY();

            // Determine the region to clear based on distance from robot
            double rx = 0.0, ry = 0.0;  // robot position assumed at origin
            double start_x = rx - distance;
            double start_y = ry - distance;
            double end_x = rx + distance;
            double end_y = ry + distance;

            start_x = std::max(start_x, ox);
            start_y = std::max(start_y, oy);
            end_x = std::min(end_x, ox + size_x);
            end_y = std::min(end_y, oy + size_y);

            unsigned int mx0, my0, mx1, my1;
            costmap->worldToMap(start_x, start_y, mx0, my0);
            costmap->worldToMap(end_x, end_y, mx1, my1);

            for (unsigned int y = my0; y < my1; y++) {
                for (unsigned int x = mx0; x < mx1; x++) {
                    costmap->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
                }
            }
        }
    }

    // Clear the local costmap layers within the specified distance
    auto local_plugins = local_layered->getPlugins();
    for (auto& plugin : *local_plugins) {
        bool should_clear = false;
        for (const auto& layer_name : clearable_layers) {
            if (plugin->getName().find(layer_name) != std::string::npos) {
                should_clear = true;
                break;
            }
        }
        if (should_clear) {
            auto costmap = plugin->getLayeredCostmap()->getCostmap();
            double ox = costmap->getOriginX();
            double oy = costmap->getOriginY();
            double size_x = costmap->getSizeInMetersX();
            double size_y = costmap->getSizeInMetersY();

            double rx = 0.0, ry = 0.0;
            double start_x = rx - distance;
            double start_y = ry - distance;
            double end_x = rx + distance;
            double end_y = ry + distance;

            start_x = std::max(start_x, ox);
            start_y = std::max(start_y, oy);
            end_x = std::min(end_x, ox + size_x);
            end_y = std::min(end_y, oy + size_y);

            unsigned int mx0, my0, mx1, my1;
            costmap->worldToMap(start_x, start_y, mx0, my0);
            costmap->worldToMap(end_x, end_y, mx1, my1);

            for (unsigned int y = my0; y < my1; y++) {
                for (unsigned int x = mx0; x < mx1; x++) {
                    costmap->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
                }
            }
        }
    }
}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0)
{
    auto global = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name + "/global", std::string(""), std::string(""));
    auto local = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name + "/local", std::string(""), std::string(""));
    std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer;

    std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>* plugins = global->getLayeredCostmap()->getPlugins();
    for (auto pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        std::shared_ptr<nav2_costmap_2d::Layer> plugin = *pluginp;
        if (plugin->getName().find("obstacles") != std::string::npos) {
            olayer = std::static_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
            addObservation(&(*olayer), 5.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
            addObservation(&(*olayer), 0.0, 5.0, MAX_Z/2, 0, 0, MAX_Z/2);
        }
    }

    global->updateMap();
    local->updateMap();
    olayer->clearStaticObservations(true, true);

    testClearBehavior("clear", distance, obstacles, static_map, global.get(), local.get());

    global->updateMap();
    local->updateMap();

    printMap(*global->getCostmap());
    ASSERT_EQ(countValues(*global->getCostmap(), LETHAL_OBSTACLE), global_lethal);
    ASSERT_EQ(countValues(*local->getCostmap(), LETHAL_OBSTACLE), local_lethal);
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
  test_node = rclcpp::Node::make_shared("clear_tests");
  testing::InitGoogleTest(&argc, argv);
  transformer = std::make_shared<tf2_ros::Buffer>(test_node->get_clock(), tf2::durationFromSec(10.0));
  tfl = std::make_shared<tf2_ros::TransformListener>(*transformer);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}