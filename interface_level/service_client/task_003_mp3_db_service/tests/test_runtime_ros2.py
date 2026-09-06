"""
Runtime test for task_003_mp3_db_service.
Launches the server node, then the client node, and verifies that the client
successfully queries the server and prints the expected album/title information.
"""

import subprocess
import sys
import time
import os
import signal
import pytest

import rclpy
from rclpy.node import Node


@pytest.fixture(scope="module", autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mp3_client_queries_server():
    """
    Start the server, then run the client, capture its stdout,
    and verify it printed the expected albums and titles.
    """
    env = os.environ.copy()

    # Start the server as a subprocess
    server_proc = subprocess.Popen(
        [sys.executable, "-u", "-c",
         "from mp3_server import main; main()"],
        cwd=os.path.dirname(os.path.abspath(__file__)),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    client_proc = None
    try:
        # Give the server a moment to start
        time.sleep(2.0)

        # Run the client and capture output
        client_proc = subprocess.Popen(
            [sys.executable, "-u", "-c",
             "from mp3_controller import main; main()"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        stdout, stderr = client_proc.communicate(timeout=15.0)
        output = stdout.decode('utf-8')

        # Verify the client printed album information
        assert '- Albums:' in output, (
            f"Expected '- Albums:' in client output. Got:\n{output}\nStderr:\n{stderr.decode()}"
        )

        # Check that at least one album name appears
        assert 'Abbey Road' in output or 'Thriller' in output or 'Back in Black' in output, (
            f"Expected at least one album name in output. Got:\n{output}"
        )

        # Check that titles appear
        assert 'Titles:' in output, (
            f"Expected 'Titles:' in client output. Got:\n{output}"
        )

        # Check specific titles
        has_title = False
        known_titles = [
            'Come Together', 'Something', 'Here Comes The Sun',
            'Thriller', 'Beat It',
            'Hells Bells', 'Back in Black', 'You Shook Me All Night Long',
        ]
        for title in known_titles:
            if title in output:
                has_title = True
                break
        assert has_title, (
            f"Expected at least one known title in output. Got:\n{output}"
        )

    finally:
        # Clean up
        if client_proc and client_proc.poll() is None:
            client_proc.terminate()
            try:
                client_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                client_proc.kill()

        server_proc.terminate()
        try:
            server_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            server_proc.kill()


def test_mp3_client_service_interaction_via_rclpy():
    """
    Use rclpy directly to create a mock server, then import and run the
    client node, verifying it makes the expected service calls.
    """
    from task_003_mp3_db_service.srv import MP3InventoryService
    import threading

    # Track calls made to the service
    calls_received = []

    test_db = {
        'TestAlbum1': ['Song1', 'Song2'],
        'TestAlbum2': ['Song3'],
    }

    node = rclpy.create_node('test_mock_server')

    def handle_request(request, response):
        calls_received.append((request.request_string, request.album))
        if request.request_string == 'album_list':
            response.response_string = 'ok'
            response.list_strings = list(test_db.keys())
        elif request.request_string == 'title_list':
            album = request.album
            if album in test_db:
                response.response_string = 'ok'
                response.list_strings = test_db[album]
            else:
                response.response_string = 'not_found'
                response.list_strings = []
        else:
            response.response_string = 'unknown'
            response.list_strings = []
        return response

    srv = node.create_service(
        MP3InventoryService,
        'mp3_inventory_interaction',
        handle_request
    )

    # Spin the mock server in a background thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        time.sleep(1.0)

        # Now create a client node (import from the actual translated file)
        from mp3_controller import Mp3InventoryClient

        # We need a separate context or we reuse the existing one
        # Actually, spin_until_future_complete inside Mp3InventoryClient will
        # conflict with the spin in the thread. So we run the client in a subprocess.
        client_proc = subprocess.Popen(
            [sys.executable, "-u", "-c",
             "from mp3_controller import main; main()"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            env=os.environ.copy(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        stdout, stderr = client_proc.communicate(timeout=15.0)
        output = stdout.decode('utf-8')

        # Verify the mock server received calls
        assert len(calls_received) >= 1, (
            f"Expected at least 1 service call, got {len(calls_received)}"
        )

        # First call should be album_list
        assert calls_received[0][0] == 'album_list', (
            f"First call should be 'album_list', got '{calls_received[0][0]}'"
        )

        # Should have title_list calls for each album
        title_calls = [c for c in calls_received if c[0] == 'title_list']
        assert len(title_calls) == 2, (
            f"Expected 2 title_list calls, got {len(title_calls)}"
        )

        # Verify output contains test albums
        assert 'TestAlbum1' in output, f"Expected 'TestAlbum1' in output. Got:\n{output}"
        assert 'TestAlbum2' in output, f"Expected 'TestAlbum2' in output. Got:\n{output}"
        assert 'Song1' in output, f"Expected 'Song1' in output. Got:\n{output}"

    finally:
        node.destroy_node()