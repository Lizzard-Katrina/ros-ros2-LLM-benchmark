"""
Runtime test for task_006_exploration_mapping.
Validates that the translated mapping.launch and costmap_common_params.yaml
files are correct ROS 2 configurations by:
1. Parsing the launch XML and verifying node attributes
2. Parsing the YAML and verifying costmap parameters
3. Running a minimal rclpy node that loads and validates the parameters
"""
import os
import re
import time
import subprocess
import signal
import pytest
import yaml
from pathlib import Path

# Locate the package files - they could be in the source directory or installed
# Try source directory first (where colcon build was run from)
def find_package_files():
    """Find the mapping.launch and costmap_common_params.yaml files."""
    # Check current directory and parent directories
    candidates = [
        Path(__file__).resolve().parent,
        Path(__file__).resolve().parent / "task_006_exploration_mapping",
    ]
    
    # Also check installed share directory
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "task_006_exploration_mapping"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            prefix = Path(result.stdout.strip())
            candidates.append(prefix / "share" / "task_006_exploration_mapping")
    except Exception:
        pass

    launch_file = None
    yaml_file = None

    for d in candidates:
        if d.exists():
            lf = d / "mapping.launch"
            yf = d / "costmap_common_params.yaml"
            if lf.exists() and launch_file is None:
                launch_file = lf
            if yf.exists() and yaml_file is None:
                yaml_file = yf

    return launch_file, yaml_file


LAUNCH_FILE, YAML_FILE = find_package_files()


def test_launch_file_exists():
    """Verify the mapping.launch file exists and is accessible."""
    assert LAUNCH_FILE is not None and LAUNCH_FILE.exists(), \
        f"mapping.launch not found. Searched multiple locations."


def test_yaml_file_exists():
    """Verify the costmap_common_params.yaml file exists and is accessible."""
    assert YAML_FILE is not None and YAML_FILE.exists(), \
        f"costmap_common_params.yaml not found. Searched multiple locations."


def test_launch_file_slam_toolbox_node():
    """Verify the launch file contains a properly configured slam_toolbox node."""
    content = LAUNCH_FILE.read_text()
    
    # Must use exec= (ROS 2 syntax)
    assert 'exec=' in content, "ROS 2 launch must use 'exec=' attribute."
    
    # Must reference slam_toolbox
    assert 'slam_toolbox' in content, "slam_toolbox package must be referenced."
    
    # Must reference async_slam_toolbox
    assert 'async_slam_toolbox' in content, "async_slam_toolbox_node must be the executable."
    
    # Must NOT have ROS 1 gmapping residue
    assert 'pkg="gmapping"' not in content, "ROS 1 gmapping residue found."
    assert 'type="slam_gmapping"' not in content, "ROS 1 slam_gmapping residue found."


def test_launch_file_rviz2():
    """Verify rviz is migrated to rviz2."""
    content = LAUNCH_FILE.read_text()
    assert 'rviz2' in content, "rviz2 must be present in the launch file."
    assert 'pkg="rviz"' not in content, "ROS 1 rviz package residue found."


def test_yaml_costmap_params_structure():
    """Verify the costmap YAML has correct structure and values."""
    content = YAML_FILE.read_text()
    data = yaml.safe_load(content)
    
    # Check footprint exists and contains 0.175
    assert 'footprint' in data, "footprint key missing from costmap params."
    footprint_str = str(data['footprint'])
    assert '0.175' in footprint_str, "Footprint coordinate 0.175 missing."


def test_yaml_observation_sources():
    """Verify observation sources are correctly configured."""
    content = YAML_FILE.read_text()
    
    # Check topic
    assert re.search(r"topic:\s+/?scan", content), \
        "Sensor topic '/scan' missing or incorrectly translated."
    
    # Check marking and clearing
    assert "marking: true" in content.lower(), "marking: true missing."
    assert "clearing: true" in content.lower(), "clearing: true missing."


def test_yaml_laser_source():
    """Verify laser observation source is defined."""
    content = YAML_FILE.read_text()
    assert 'laser' in content, "laser observation source missing."
    assert 'LaserScan' in content, "LaserScan data_type missing."


def test_rclpy_node_loads_yaml_params():
    """
    Actually start a rclpy node, load the YAML costmap params,
    and verify the parameters are correctly structured.
    """
    import rclpy
    from rclpy.node import Node
    
    rclpy.init()
    node = None
    try:
        node = Node('test_costmap_param_loader')
        
        # Load the YAML file
        yaml_content = YAML_FILE.read_text()
        data = yaml.safe_load(yaml_content)
        
        # Declare and set parameters from the YAML on our test node
        # This verifies the YAML is valid and parseable
        
        # Set footprint as a string parameter
        footprint = data.get('footprint', [])
        node.declare_parameter('footprint', str(footprint))
        fp_val = node.get_parameter('footprint').get_parameter_value().string_value
        assert '0.175' in fp_val, "Footprint parameter does not contain 0.175"
        
        # Set obstacle layer params
        obstacle_layer = data.get('obstacle_layer', {})
        ros_params = obstacle_layer.get('ros__parameters', {})
        
        # Verify laser config exists
        laser_config = ros_params.get('laser', {})
        assert laser_config.get('topic') in ['/scan', 'scan'], \
            f"Laser topic is '{laser_config.get('topic')}', expected '/scan' or 'scan'"
        assert laser_config.get('marking') is True, "marking must be true"
        assert laser_config.get('clearing') is True, "clearing must be true"
        assert laser_config.get('data_type') == 'LaserScan', "data_type must be LaserScan"
        
        # Declare these as node parameters to verify they work in ROS 2
        node.declare_parameter('obstacle_layer.laser.topic', laser_config['topic'])
        node.declare_parameter('obstacle_layer.laser.marking', laser_config['marking'])
        node.declare_parameter('obstacle_layer.laser.clearing', laser_config['clearing'])
        
        topic_val = node.get_parameter('obstacle_layer.laser.topic').get_parameter_value().string_value
        marking_val = node.get_parameter('obstacle_layer.laser.marking').get_parameter_value().bool_value
        clearing_val = node.get_parameter('obstacle_layer.laser.clearing').get_parameter_value().bool_value
        
        assert 'scan' in topic_val, f"Topic parameter value '{topic_val}' doesn't contain 'scan'"
        assert marking_val is True, "marking parameter must be True"
        assert clearing_val is True, "clearing parameter must be True"
        
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_xml_well_formed():
    """Verify the launch XML is well-formed."""
    import xml.etree.ElementTree as ET
    
    content = LAUNCH_FILE.read_text()
    try:
        tree = ET.fromstring(content)
    except ET.ParseError as e:
        pytest.fail(f"mapping.launch is not well-formed XML: {e}")
    
    # Find all node elements
    nodes = tree.findall('.//node')
    assert len(nodes) >= 1, "At least one <node> element expected in launch file."
    
    # Verify each node has 'exec' attribute (ROS 2 style)
    for n in nodes:
        assert 'exec' in n.attrib, f"Node '{n.attrib.get('name', '?')}' missing 'exec' attribute."
        assert 'pkg' in n.attrib, f"Node '{n.attrib.get('name', '?')}' missing 'pkg' attribute."
    
    # Find the slam_toolbox node specifically
    slam_nodes = [n for n in nodes if 'slam_toolbox' in n.attrib.get('pkg', '')]
    assert len(slam_nodes) >= 1, "No slam_toolbox node found in launch file."
    
    slam_node = slam_nodes[0]
    assert 'async_slam_toolbox' in slam_node.attrib.get('exec', ''), \
        "slam_toolbox node must use async_slam_toolbox_node executable."