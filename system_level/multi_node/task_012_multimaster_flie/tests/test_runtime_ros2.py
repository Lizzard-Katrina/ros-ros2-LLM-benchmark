"""
Runtime tests for task_012_multimaster_flie.

Tests the MasterMonitor and SyncThread logic including:
- XML-RPC succeed helper
- Master state population
- Loop prevention
- Batched multicall usage
- Remote URI preservation
"""
import pytest
import subprocess
import time
import sys
import os
import threading
import re
from pathlib import Path
from unittest.mock import MagicMock, patch, call
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client as xmlrpcclient

# Ensure the package is importable
sys.path.insert(0, str(Path(__file__).resolve().parent))

from task_012_multimaster_flie.master_monitor import MasterMonitor, _succeed, MasterInfo
from task_012_multimaster_flie.sync_thread import SyncThread


class TestSucceedHelper:
    """Test the _succeed XML-RPC response validator."""

    def test_succeed_with_valid_response(self):
        result = _succeed((1, 'ok', ['/topic1', '/topic2']))
        assert result == ['/topic1', '/topic2']

    def test_succeed_with_error_response(self):
        with pytest.raises(Exception, match="remote call failed"):
            _succeed((0, 'error occurred', None))

    def test_succeed_with_negative_code(self):
        with pytest.raises(Exception, match="remote call failed"):
            _succeed((-1, 'not found', None))


class TestMasterMonitorUpdateState:
    """Test MasterMonitor.updateState() logic."""

    def _create_mock_master_server(self, port):
        """Create a mock XML-RPC server that mimics a ROS Master."""
        server = SimpleXMLRPCServer(('127.0.0.1', port), logRequests=False, allow_none=True)

        def getUri(caller_id):
            return (1, 'ok', 'http://127.0.0.1:%d/' % port)

        def getTopicTypes(caller_id):
            return (1, 'ok', [
                ['/chatter', 'std_msgs/String'],
                ['/numbers', 'std_msgs/Int32'],
                ['/rosout', 'rosgraph_msgs/Log'],
            ])

        def getSystemState(caller_id):
            publishers = [
                ['/chatter', ['/talker_node']],
                ['/rosout', ['/talker_node', '/listener_node']],
            ]
            subscribers = [
                ['/chatter', ['/listener_node']],
                ['/rosout', ['/rosout_node']],
            ]
            services = [
                ['/talker_node/get_loggers', ['/talker_node']],
            ]
            return (1, 'ok', [publishers, subscribers, services])

        server.register_function(getUri, 'getUri')
        server.register_function(getTopicTypes, 'getTopicTypes')
        server.register_function(getSystemState, 'getSystemState')
        return server

    def test_monitor_populates_master_info(self):
        """Test that updateState correctly populates MasterInfo from XML-RPC data."""
        port = 11399
        server = self._create_mock_master_server(port)
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()

        try:
            time.sleep(0.3)
            monitor = MasterMonitor(
                masteruri='http://127.0.0.1:%d/' % port,
                ros_node_name='/test_monitor',
                mastername='test_master',
                rpcport=0,
                do_retry=False
            )

            state = monitor.updateState()

            assert state is not None, "updateState should return a MasterInfo object"
            assert isinstance(state, MasterInfo)

            # Check that topics were populated (including from getTopicTypes)
            assert '/chatter' in state._topic_types
            assert state._topic_types['/chatter'] == 'std_msgs/String'
            assert '/numbers' in state._topic_types
            assert state._topic_types['/numbers'] == 'std_msgs/Int32'

            # Check that nodes were created
            assert '/talker_node' in state.nodes
            assert '/listener_node' in state.nodes

            # Check publishers were recorded
            pub_topics = [t for t, n, nu in state._publishers]
            assert '/chatter' in pub_topics
            assert '/rosout' in pub_topics

            # Check subscribers were recorded
            sub_topics = [t for t, n, nu in state._subscribers]
            assert '/chatter' in sub_topics

        finally:
            server.shutdown()

    def test_monitor_uses_succeed_for_validation(self):
        """Test that _succeed is used for XML-RPC response validation."""
        port = 11398
        server = self._create_mock_master_server(port)
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()

        try:
            time.sleep(0.3)
            monitor = MasterMonitor(
                masteruri='http://127.0.0.1:%d/' % port,
                ros_node_name='/test_monitor2',
                mastername='test_master2',
                rpcport=0,
                do_retry=False
            )

            # Patch _succeed to track calls
            original_succeed = monitor._succeed
            call_count = [0]

            def tracking_succeed(args):
                call_count[0] += 1
                return original_succeed(args)

            monitor._succeed = tracking_succeed
            state = monitor.updateState()

            # _succeed should have been called at least twice (getTopicTypes + getSystemState)
            assert call_count[0] >= 2, (
                "self._succeed should be called for each XML-RPC response. "
                "Got %d calls" % call_count[0]
            )
        finally:
            server.shutdown()


class TestSyncThreadLoopPrevention:
    """Test SyncThread loop prevention and filtering."""

    def test_loop_prevention_skips_local_node(self):
        """Test that _do_ignore_ntp returns True for the local node name."""
        sync = SyncThread(
            name='remote_master',
            uri='http://remote:11311',
            discoverer_name='/remote_discovery',
            monitoruri='http://remote:11611',
            timestamp=time.time(),
            ros_node_name='/my_sync_node',
            masteruri_local='http://localhost:11311'
        )

        # The local node name should be ignored (loop prevention)
        assert sync._do_ignore_ntp('/my_sync_node', '/some_topic', 'std_msgs/String') is True
        # A different node should NOT be ignored (assuming not in filter list)
        assert sync._do_ignore_ntp('/remote_talker', '/custom_topic', 'std_msgs/String') is False

    def test_loop_prevention_in_subscriber(self):
        """Test loop prevention for subscribers."""
        sync = SyncThread(
            name='remote_master',
            uri='http://remote:11311',
            discoverer_name='/remote_discovery',
            monitoruri='http://remote:11611',
            timestamp=time.time(),
            ros_node_name='/my_sync_node',
            masteruri_local='http://localhost:11311'
        )

        assert sync._do_ignore_nts('/my_sync_node', '/topic', 'std_msgs/String') is True
        assert sync._do_ignore_nts('/other_node', '/custom_topic', 'std_msgs/String') is False

    def test_loop_prevention_in_service(self):
        """Test loop prevention for services."""
        sync = SyncThread(
            name='remote_master',
            uri='http://remote:11311',
            discoverer_name='/remote_discovery',
            monitoruri='http://remote:11611',
            timestamp=time.time(),
            ros_node_name='/my_sync_node',
            masteruri_local='http://localhost:11311'
        )

        assert sync._do_ignore_ns('/my_sync_node', '/some_service') is True
        assert sync._do_ignore_ns('/other_node', '/custom_service') is False


class TestSyncThreadApplyRemoteState:
    """Test SyncThread._apply_remote_state with a mock local master."""

    def _create_mock_local_master(self, port):
        """Create a mock local ROS Master XML-RPC server."""
        server = SimpleXMLRPCServer(('127.0.0.1', port), logRequests=False, allow_none=True)
        server.registrations = []

        def registerPublisher(caller_id, topic, topic_type, caller_api):
            server.registrations.append(('pub', caller_id, topic, topic_type, caller_api))
            return (1, 'ok', [])

        def registerSubscriber(caller_id, topic, topic_type, caller_api):
            server.registrations.append(('sub', caller_id, topic, topic_type, caller_api))
            return (1, 'ok', [])

        def registerService(caller_id, service, service_api, caller_api):
            server.registrations.append(('srv', caller_id, service, service_api, caller_api))
            return (1, 'ok', 0)

        def unregisterPublisher(caller_id, topic, caller_api):
            server.registrations.append(('upub', caller_id, topic, caller_api))
            return (1, 'ok', 0)

        def unregisterSubscriber(caller_id, topic, caller_api):
            server.registrations.append(('usub', caller_id, topic, caller_api))
            return (1, 'ok', 0)

        def unregisterService(caller_id, service, service_api):
            server.registrations.append(('usrv', caller_id, service, service_api))
            return (1, 'ok', 0)

        server.register_function(registerPublisher, 'registerPublisher')
        server.register_function(registerSubscriber, 'registerSubscriber')
        server.register_function(registerService, 'registerService')
        server.register_function(unregisterPublisher, 'unregisterPublisher')
        server.register_function(unregisterSubscriber, 'unregisterSubscriber')
        server.register_function(unregisterService, 'unregisterService')
        return server

    def test_apply_remote_state_registers_publishers(self):
        """Test that _apply_remote_state registers remote publishers on local master."""
        port = 11397
        server = self._create_mock_local_master(port)
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()

        try:
            time.sleep(0.3)
            sync = SyncThread(
                name='remote_master',
                uri='http://remote:11311',
                discoverer_name='/remote_discovery',
                monitoruri='http://remote:11611',
                timestamp=time.time(),
                ros_node_name='/my_sync_node',
                masteruri_local='http://127.0.0.1:%d/' % port
            )

            # Construct a remote state with publishers
            remote_state = (
                str(time.time()),  # timestamp
                str(time.time()),  # timestamp_local
                'http://remote:11311',  # masteruri
                'remote_master',  # mastername
                # publishers: list of (topic, node, nodeuri, topictype)
                [
                    ('/remote_topic', '/remote_talker', 'http://remote:12345', 'std_msgs/String'),
                ],
                # subscribers
                [],
                # services
                [],
                # topic_types
                [('/remote_topic', 'std_msgs/String')],
                # nodes
                [],
                # service_providers
                [],
            )

            sync._apply_remote_state(remote_state)
            time.sleep(0.5)

            # Check that registerPublisher was called
            pub_regs = [r for r in server.registrations if r[0] == 'pub']
            assert len(pub_regs) >= 1, (
                "Expected at least one registerPublisher call, got %d. Registrations: %s" % (
                    len(pub_regs), server.registrations))

            # Verify the remote URI is preserved
            reg = pub_regs[0]
            assert reg[1] == '/remote_talker', "caller_id should be the remote node name"
            assert reg[2] == '/remote_topic', "topic should be /remote_topic"
            assert reg[3] == 'std_msgs/String', "type should be std_msgs/String"
            assert 'remote' in reg[4], "caller_api should contain the remote URI"

        finally:
            server.shutdown()

    def test_apply_remote_state_skips_local_node(self):
        """Test that _apply_remote_state skips publishers from the local sync node."""
        port = 11396
        server = self._create_mock_local_master(port)
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()

        try:
            time.sleep(0.3)
            sync = SyncThread(
                name='remote_master',
                uri='http://remote:11311',
                discoverer_name='/remote_discovery',
                monitoruri='http://remote:11611',
                timestamp=time.time(),
                ros_node_name='/my_sync_node',
                masteruri_local='http://127.0.0.1:%d/' % port
            )

            # Remote state where the publisher is our own sync node (should be skipped)
            remote_state = (
                str(time.time()),
                str(time.time()),
                'http://remote:11311',
                'remote_master',
                [
                    ('/synced_topic', '/my_sync_node', 'http://remote:12345', 'std_msgs/String'),
                ],
                [],
                [],
                [('/synced_topic', 'std_msgs/String')],
                [],
                [],
            )

            sync._apply_remote_state(remote_state)
            time.sleep(0.5)

            # No registrations should have been made (loop prevention)
            pub_regs = [r for r in server.registrations if r[0] == 'pub']
            assert len(pub_regs) == 0, (
                "Should not register publishers from local sync node (loop prevention). "
                "Got %d registrations" % len(pub_regs))

        finally:
            server.shutdown()


class TestStaticOracleCompatibility:
    """Verify that the source files pass the static oracle regex checks."""

    def _get_monitor_content(self):
        path = Path(__file__).resolve().parent / "task_012_multimaster_flie" / "master_monitor.py"
        with open(path, 'r') as f:
            return f.read()

    def _get_sync_content(self):
        path = Path(__file__).resolve().parent / "task_012_multimaster_flie" / "sync_thread.py"
        with open(path, 'r') as f:
            return f.read()

    def test_monitor_calls_master_api(self):
        content = self._get_monitor_content()
        assert re.search(r'\.getTopicTypes\s*\(', content)
        assert re.search(r'\.getSystemState\s*\(', content)

    def test_monitor_uses_succeed_helper(self):
        content = self._get_monitor_content()
        assert re.search(r'self\._succeed\s*\(', content)

    def test_monitor_populates_master_info(self):
        content = self._get_monitor_content()
        assert re.search(r'for\s+\w+,\s+\w+\s+in\s+publishers:', content)

    def test_sync_uses_multicall(self):
        content = self._get_sync_content()
        assert re.search(r'own_master_multi\s*\(', content)

    def test_sync_loop_prevention(self):
        content = self._get_sync_content()
        assert re.search(r'if\s+[\w\.]+\s*==\s*(?:rospy\.get_name\(\)|self\.ros_node_name)', content)

    def test_sync_filter_application(self):
        content = self._get_sync_content()
        assert re.search(r'\.is_ignored_(?:publisher|subscriber|service)', content)

    def test_sync_preserves_remote_uri(self):
        content = self._get_sync_content()
        assert re.search(r'registerPublisher\s*\(.*,\s*[\w_]*uri', content)

    def test_absence_of_hardcoded_names(self):
        content_monitor = self._get_monitor_content()
        content_sync = self._get_sync_content()
        hardcoded = re.search(r'[\'"]/(?:master_discovery|master_sync)[\'"]', content_monitor + content_sync)
        assert not hardcoded


class TestROS2NodeLaunch:
    """Test that the ROS2 node can be launched."""

    def test_node_launches_and_publishes(self):
        """Test that master_monitor_node starts and publishes heartbeat."""
        import rclpy
        from std_msgs.msg import String

        # Use a unique ROS domain to avoid interference
        env = os.environ.copy()

        rclpy.init()
        proc = None
        test_node = None
        try:
            # Launch the node as a subprocess
            proc = subprocess.Popen(
                [sys.executable, '-c',
                 'from task_012_multimaster_flie.master_monitor_node import main; main()'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
            )

            # Give the node time to start up
            time.sleep(2.0)

            # Create a test subscriber
            test_node = rclpy.create_node('test_heartbeat_listener')
            received_msgs = []

            def callback(msg):
                received_msgs.append(msg.data)

            sub = test_node.create_subscription(
                String, 'master_monitor/heartbeat', callback, 10)

            # Spin for a few seconds to receive messages
            end_time = time.time() + 10.0
            while time.time() < end_time and len(received_msgs) < 2:
                rclpy.spin_once(test_node, timeout_sec=0.5)

            assert len(received_msgs) >= 1, (
                "Expected to receive at least 1 heartbeat message, got %d" % len(received_msgs))
            assert received_msgs[0].startswith('alive_'), (
                "Heartbeat message should start with 'alive_', got: %s" % received_msgs[0])

        finally:
            if test_node is not None:
                test_node.destroy_node()
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2)
            rclpy.shutdown()