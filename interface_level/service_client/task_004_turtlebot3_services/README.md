# Task 004 — TurtleBot3 Service Interaction

This task checks whether the LLM can correctly reconstruct a TurtleBot3-style
ROS service interaction. The panorama service uses Empty messages and specific
service names. Models often hallucinate these names or merge the server/client.

## Why these blanks?
- Missing service name: models often guess wrongly.
- EmptyResponse blank: tests if the model remembers `EmptyResponse()`.
- Missing CMake/package.xml lines: common failure points in ROS1.
- Two scripts separated: prevents model from merging files.

## Expected Outcome
- Server node: starts and advertises correct service.
- Client node: waits and calls the service successfully.
- Buildable with catkin_make.

