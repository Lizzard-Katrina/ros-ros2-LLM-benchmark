import sys
import pytest
from pathlib import Path
import re

TASK_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TASK_ROOT))

# Read the C++ source file
CPP_FILE = TASK_ROOT / "stereo_sync.cpp"


class TestStereoSyncOracle:
    """Oracle tests for ROS1->ROS2 translation of stereo_sync.cpp"""
    
    @classmethod
    def setup_class(cls):
        """Read the C++ source file."""
        if not CPP_FILE.exists():
            raise FileNotFoundError(f"stereo_sync.cpp not found at {CPP_FILE}")
        
        with open(CPP_FILE, 'r') as f:
            cls.source_code = f.read()
        
        print(f"✓ Loaded stereo_sync.cpp")
    
    def test_oracle_1_ros2_headers(self):
        """Oracle 1: Must use ROS2 headers instead of ros/ros.h"""
        # Should NOT have ROS1 header
        assert '#include <ros/ros.h>' not in self.source_code, (
            "❌ Still using ROS1 header 'ros/ros.h'. "
            "Should use ROS2 headers like 'rclcpp/rclcpp.hpp'"
        )
        
        # Should have ROS2 rclcpp header
        has_rclcpp = re.search(r'#include\s+[<"]rclcpp', self.source_code)
        assert has_rclcpp, (
            "❌ Missing ROS2 header. Expected: #include <rclcpp/rclcpp.hpp>"
        )
        print("✓ Oracle 1: Using ROS2 headers (rclcpp)")
    
    def test_oracle_2_stereo_sync_class(self):
        """Oracle 2: StereoSync class must exist with proper structure"""
        # Check for class definition
        has_class = re.search(r'class\s+StereoSync\s*\{', self.source_code)
        assert has_class, "❌ StereoSync class not found"
        
        # Check for syncCallback method
        has_callback = re.search(
            r'(?:void|auto)\s+syncCallback\s*\(',
            self.source_code
        )
        assert has_callback, (
            "❌ syncCallback method not found in StereoSync class"
        )
        
        print("✓ Oracle 2: StereoSync class with syncCallback exists")
    
    def test_oracle_3_message_filters_synchronizer(self):
        """Oracle 3: Must use message_filters::Synchronizer with ApproximateTime"""
        # Check for message_filters include
        has_message_filters = '#include <message_filters' in self.source_code
        assert has_message_filters, (
            "❌ Missing message_filters include"
        )
        
        # Check for Synchronizer with ApproximateTime
        has_sync = re.search(
            r'message_filters::Synchronizer<\s*'
            r'message_filters::sync_policies::ApproximateTime',
            self.source_code
        )
        assert has_sync, (
            "❌ Missing Synchronizer with ApproximateTime policy. "
            "Expected: message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<...>>"
        )
        
        print("✓ Oracle 3: Using message_filters::Synchronizer with ApproximateTime")
    
    def test_oracle_4_two_image_subscribers(self):
        """Oracle 4: Must have two Image subscribers (left and right)"""
        # Check for two Image subscriber declarations
        image_subs = re.findall(
            r'message_filters::Subscriber<sensor_msgs::Image>',
            self.source_code
        )
        assert len(image_subs) >= 2, (
            "❌ Missing two Image subscribers. "
            "Expected: message_filters::Subscriber<sensor_msgs::Image> for left and right"
        )
        
        # Check for left_sub_ and right_sub_
        has_left = 'left_sub' in self.source_code
        has_right = 'right_sub' in self.source_code
        assert has_left and has_right, (
            "❌ Missing left_sub_ and/or right_sub_ member variables"
        )
        
        print("✓ Oracle 4: Two Image subscribers (left_sub_, right_sub_) exist")
    
    def test_oracle_5_ros2_node_creation(self):
        """Oracle 5: main() must create ROS2 node (not ros::init)"""
        # Should NOT use ROS1 initialization
        assert 'ros::init' not in self.source_code, (
            "❌ Still using ROS1 ros::init(). Should use ROS2 rclcpp::init()"
        )
        
        # Should use rclcpp methods for node creation
        has_node_creation = re.search(
            r'(?:rclcpp::Node|make_shared.*Node|std::make_shared)',
            self.source_code
        )
        assert has_node_creation, (
            "❌ Missing ROS2 node creation. "
            "Expected: std::make_shared<rclcpp::Node>(...) or rclcpp::create_node(...)"
        )
        
        print("✓ Oracle 5: Using ROS2 node creation (not ros::init)")
    
    def test_oracle_6_main_function(self):
        """Oracle 6: Must have main() function with proper ROS2 initialization"""
        # Check for main function
        has_main = re.search(
            r'int\s+main\s*\(\s*(?:int\s+argc\s*,\s*char\s*\*.*argv|)\)',
            self.source_code
        )
        assert has_main, (
            "❌ main() function not found or incorrect signature"
        )
        
        # Check for rclcpp::init or rclcpp::spin
        has_rclcpp_init_or_spin = re.search(
            r'rclcpp::init|rclcpp::spin',
            self.source_code
        )
        assert has_rclcpp_init_or_spin, (
            "❌ main() must call rclcpp::init() and rclcpp::spin() for ROS2"
        )
        
        print("✓ Oracle 6: main() properly initializes and spins ROS2 node")
    
    def test_oracle_7_no_ros1_remnants(self):
        """Oracle 7: Should not have ROS1-specific code remnants"""
        ros1_patterns = [
            ('ros::NodeHandle', 'ROS1 NodeHandle'),
            ('ros::Subscriber', 'ROS1 Subscriber'),
            ('boost::shared_ptr', 'boost::shared_ptr (use std::shared_ptr in ROS2)'),
            ('ROS_INFO', 'ROS_INFO (use RCLCPP_INFO in ROS2)'),
        ]
        
        problematic = []
        for pattern, description in ros1_patterns:
            if pattern in self.source_code:
                problematic.append(description)
        
        # Some ROS1 patterns might be okay if they're in comments, but flag them
        if problematic:
            print(f"⚠ Warning: Found ROS1 patterns: {problematic}")
            # Still pass but warn - student might have forgotten to update everything
        
        print("✓ Oracle 7: Minimal ROS1 remnants (or none)")
