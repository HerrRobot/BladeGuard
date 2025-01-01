# ROS TOPICS

This document provides an overview of the ROS topics used by the robot.

### Distance Sensors
- Name - /distance_sensors
- Message type - std_msgs/msg/String
- Format - leftDistance;rightDistance;ceilingDistance;rootDistance
- Use - used to publish the data from the distance sensors, from the Raspberry Pi to the UI

### Logger
- Name - /log_status
- Message type - std_msgs/msg/String
- Format - hh:mm:ss; [message content]
- Use - used to publish status messages from the Raspberry Pi to the UI, to be displayed in the logger