# ROS SERVICES

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

### Manual Action
- Name - /manual_action_service
- Message type - service_interfaces/srv/ManualAction
    - Request:
        - string action_name - name of the action to be performed
        - int64 request_id - ID of the request, increasing order, starting at zero
    - Response:
        - string action_name - name of the action which was performed
        - int 64 response_id - ID of the response, equal to the request_id
        - bool success - indicates whether the action was performed successfully or not
- Use:
    - Client - in the UI, call the service to make the robot perform a manual action (sand, clean...)
    - Server - on the Raspberry Pi, propagates the request further and responds with success/failure