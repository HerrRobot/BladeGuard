const raspberryPiIPAddress = '192.168.0.2';
const rosbridgeServerPort = '9090';

const localhostIPAddress = '127.0.0.1';
const localhostPort = '9090';

// use for testing, when the rosbridge server is on the local machine
const localhostRosbridgeURL = `ws://${localhostIPAddress}:${localhostPort}`
// use in production/testing, when the rosbridge server is on the Raspberry Pi
const raspberryPiRosBridgeURL = `ws://${raspberryPiIPAddress}:${rosbridgeServerPort}`

// Create ros object to communicate over your Rosbridge connection
const ros = new ROSLIB.Ros({ url: localhostRosbridgeURL });

// handle successful connection
ros.on("connection", () => {
    console.log(`Successfully connected to rosbridge server at ${raspberryPiRosBridgeURL}`);
});

// handle error in connection
ros.on("error", (error) => {
    console.log(`errored out rosbridge connection at ${raspberryPiRosBridgeURL}: (${error})`);
    alert(`errored out rosbridge connection at ${raspberryPiRosBridgeURL}: (${error})`);
});

// handle closing connection
ros.on("close", () => {
    alert(`Closed rosbridge server connection at ${raspberryPiRosBridgeURL}`);
});

// topic for reading values of the four distance sensors
const distance_sensors_topic = new ROSLIB.Topic({
    ros,
    name: "/distance_sensors",
    messageType: "std_msgs/String"
})

// after receiving a message from topic /distance_sensors, parse the data and display the sensor values
distance_sensors_topic.subscribe((message) => {
    const { left, right, ceiling, root } = parseSensorDataFromROSTopic(message.data);

    displaySensorValues(left, right, ceiling, root);
});

// topic for receiving messages for the logger
const log_status_topic = new ROSLIB.Topic({
    ros,
    name: '/log_status',
    messageType: 'std_msgs/String'
})

// after receiving a message from topic /log_status, display the message in the logger
log_status_topic.subscribe((message) => {
    addEntryToLog(message.data);
})

/**
 * Create logger service.
 * FOR THE CUSTOM LOG_MESSAGE TO WORK, ONE MUST RUN `source BladeGuard/prototype/ros/motor_controls_ws/install/setup.bash` before 
 * running the rosbridge server
 */
const log_message_server = new ROSLIB.Service({
    ros,
    name: '/logger_service',
    serviceType: 'service_interfaces/LogMessage'
})

// advertise the service (switch from client to server)
// when the service is called, display the message in the logger
log_message_server.advertise((request, response) => {
    addEntryToLog(`${request.timestamp}; ${request.message}`);
    updateActionStatus(request.message);

    return true // needs to be a boolean, otherwise rosbridge complains
})

/**
 * Client for sending a manual anction command to the robot
 */
const manual_action_client = new ROSLIB.Service({
    ros,
    name: '/manual_action_service',
    serviceType: 'service_interfaces/ManualAction'
})

const manual_driving_topic = new ROSLIB.Topic({
    ros,
    name: '/manual_driving',
    messageType: 'std_msgs/String'
});

manual_driving_topic.advertise();

/**
 * Published the given message to the /manual_driving topic
 * @param {String} message 
 */
function publishToManualDrivingTopic(message) {
    const rosMessage = {data: message}
    manual_driving_topic.publish(rosMessage);
}

/**
 * Parse the input string from the /distance_sensors topic and return an object containing the individual sensor values as strings,
 * rounded to two decimal places.
 * @param {String} sensorDataString string in the form 'leftDistance;rightDistance;ceilingDistance;rootDistance'
 */
function parseSensorDataFromROSTopic(sensorDataString) {
    const split = sensorDataString.split(';');

    return {
        left: parseFloat(split[0]).toFixed(2),
        right: parseFloat(split[1]).toFixed(2),
        ceiling: parseFloat(split[2]).toFixed(2),
        root: parseFloat(split[3]).toFixed(2)
    }
}

/**
 * Given the name of a manual action, call the manual action service with this action
 * @param {*} action_name name of the action to perform
 */
function sendActionToRobot(action_name, request_id) {
    console.log(action_name, request_id)
    manual_action_client.callService({ action_name, request_id }, handleManualActionCompletion, (error) => {
        console.log(error);
    });
}