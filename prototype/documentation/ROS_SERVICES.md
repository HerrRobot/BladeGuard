# ROS TOPICS

This document provides an overview of the ROS services used by the robot.

### Logger
- Name - /logger_service
- Message type - service_interfaces/srv/LogMessage
    - Request:
        - string timestamp - timestamp at which the message was generated
        - string message - content of the message to be logged
    - Response: N/A
- Use:
    - Client - on the Raspberry Pi, call the service to have it display the message
    - Server - in the UI, display the received message in the logger