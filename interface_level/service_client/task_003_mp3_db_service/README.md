# Task 003 – MP3 Database Query Service (Custom ROS Service)

### Goal
Translate a ROS1 custom service server and multiple client nodes into ROS2.  
This task evaluates handling of custom service definitions, internal database logic, and multi-client interactions.

### Structure
- ros1_code/
    - server_node.py (incomplete)
    - client_query1.py (incomplete)
    - client_query2.py (incomplete)
    - srv/QueryMP3.srv (complete)
    - launch/mp3_server.launch
- docker/
- expected_outcome/
- tests/
- metadata.json

### Expected Outcome (Behavior-Level)
- Server loads an internal MP3 database.
- Client 1 queries by artist.
- Client 2 queries by track title.
- ROS2 equivalents run without errors.

### Notes
This task is more complex than task_001 and task_002 because it requires:
- Multiple client programs
- Internal server state
- Custom service type extending beyond simple request–response pairs
