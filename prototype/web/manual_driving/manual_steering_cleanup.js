let allowedToSend = true;

const forwardSpeedMax = 128;
const forwardSpeedIncrement = 8;
const steeringSpeedMax = 32;
const steeringSpeedIncrement = 4;

let currentRightWheelSpeed = {
    forward: 0,
    sideways: 0
}

let currentLeftWheelSpeed = {
    forward: 0,
    sideways: 0
}

const forwardAccelerationIntervalDuration = 750; // miliseconds
const sidewaysAccelerationIntervalDuration = 500; // miliseconds

const activeKeys = new Set();

let brakingMode = "N";

let forwardInterval;
let sidewaysInterval;
let steeringNegationInterval;

if (document.getElementById("BRAKE").checked) {
    brakingMode = "BR";
}

document.addEventListener('keydown', (e) => {
    if (activeKeys.has(e.key)) {
        return;
    }

    activeKeys.add(e.key);

    if (e.key === 'w') {
        if (activeKeys.has('s')) {
            return;
        }

        accelerateForwards(forwardSpeedIncrement, forwardAccelerationIntervalDuration);
    } else if (e.key === 'a') {
        if (activeKeys.has('d')) {
            return;
        }

        if (steeringNegationInterval){
            clearInterval(steeringNegationInterval);
        }

        accelerateSideways(-steeringSpeedIncrement, sidewaysAccelerationIntervalDuration);
    } else if (e.key === 's') {
        if (activeKeys.has('w')) {
            return;
        }

        accelerateForwards(-forwardSpeedIncrement, forwardAccelerationIntervalDuration);
    } else if (e.key === 'd') {
        if (activeKeys.has('a')) {
            return;
        }

        if (steeringNegationInterval) {
            clearInterval(steeringNegationInterval);
        }

        accelerateSideways(steeringSpeedIncrement, sidewaysAccelerationIntervalDuration);
    } else {
        return;
    }
})

document.addEventListener('keyup', (e) => {
    activeKeys.delete(e.key);

    if (e.key === 'w' || e.key === 's') {
        clearInterval(forwardInterval);

        currentLeftWheelSpeed.forward = 0;
        currentRightWheelSpeed.forward = 0;

        sendMessageToArduino();
    } else if (e.key === 'a') {
        clearInterval(sidewaysInterval);

        graduallyNegateSteering(steeringSpeedIncrement, sidewaysAccelerationIntervalDuration);
    } else if (e.key === 'd') {
        clearInterval(sidewaysInterval);

        graduallyNegateSteering(-steeringSpeedIncrement, sidewaysAccelerationIntervalDuration);
    } else {
        return;
    }
})

/**
 * Stops all wheel motors using the arduino and prevents further wheel commands from being sent.
 */
function emergencyStop() {
    allowedToSend = false;

    const stopLeftMotorsMessage = "W_L;N;0;";
    const stopRightMotorsMessage = "W_R;N;0;";

    // publish both to Arduino

    sendSerialToArduino(stopLeftMotorsMessage);
    sendSerialToArduino(stopRightMotorsMessage);

    document.getElementById("emergency_heading").setAttribute("style", "display: block;")
}

/**
 * Calculates the parameters for the Arduino message. The parameters are the side of the wheels (Left and Right), direction/braking mode, and speed
 * @returns the parameters for both the left and right wheels
 */
function calculateArduinoMessageParameters() {
    let resultParametersLeft = {
        side: 'L',
        direction: '',
        speed: 0
    }

    let resultParametersRight = {
        side: 'R',
        direction: '',
        speed: 0
    }

    let result = {
        resultParametersLeft,
        resultParametersRight
    }

    let currentLeftWheelSpeedTotal = currentLeftWheelSpeed.forward + currentLeftWheelSpeed.sideways;
    let currentRightWheelSpeedTotal = currentRightWheelSpeed.forward + currentRightWheelSpeed.sideways;

    if (currentLeftWheelSpeedTotal > 255) {
        currentLeftWheelSpeedTotal = 255;
    } else if (currentLeftWheelSpeedTotal < -255) {
        currentLeftWheelSpeedTotal = -255
    }

    if (currentRightWheelSpeedTotal > 255) {
        currentRightWheelSpeedTotal = 255;
    } else if (currentRightWheelSpeedTotal < -255) {
        currentRightWheelSpeedTotal = -255
    }

    if (currentLeftWheelSpeedTotal < 0) {
        resultParametersLeft.direction = 'B';
        resultParametersLeft.speed = -currentLeftWheelSpeedTotal;
    } else if (currentLeftWheelSpeedTotal > 0) {
        resultParametersLeft.direction = 'F';
        resultParametersLeft.speed = currentLeftWheelSpeedTotal;
    } else {
        resultParametersLeft.direction = brakingMode;
        resultParametersLeft.speed = 0;
    }

    if (currentRightWheelSpeedTotal < 0) {
        resultParametersRight.direction = 'B';
        resultParametersRight.speed = -currentRightWheelSpeedTotal;
    } else if (currentRightWheelSpeedTotal > 0) {
        resultParametersRight.direction = 'F';
        resultParametersRight.speed = currentRightWheelSpeedTotal;
    } else {
        resultParametersRight.direction = brakingMode;
        resultParametersRight.speed = 0;
    }

    return result;
}

/**
 * Builds the command to be sent to the Arduino, as specified by the communicatrion protocol
 * @param {*} messageParameters parameters for the messages for the left and right wheels
 * @returns the command strings to send to the Arduino
 */
function buildArduinoMessages(messageParameters) {
    let commandLeft = `W_${messageParameters.resultParametersLeft.side};${messageParameters.resultParametersLeft.direction};${messageParameters.resultParametersLeft.speed};`;
    let commandRight = `W_${messageParameters.resultParametersRight.side};${messageParameters.resultParametersRight.direction};${messageParameters.resultParametersRight.speed};`;

    return {
        commandLeft,
        commandRight
    }
}

/**
 * Sets the braking mode to the specified value
 * @param {*} val New braking mode
 */
function setBrakeMode(val) {
    brakingMode = val;
}

/**
 * Emergency stop if user presses space or Enter key
 */
document.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') {
        emergencyStop();
    }
})

/**
 * Starts forward acceleration/deceleration interval
 * @param {*} speedIncrement speed increment of all the wheels
 * @param {*} intervalPeriod period of the interval
 */
function accelerateForwards(speedIncrement, intervalPeriod) {
    currentLeftWheelSpeed.forward += speedIncrement;
    currentRightWheelSpeed.forward += speedIncrement;

    sendMessageToArduino();

    forwardInterval = setInterval(() => {
        if (Math.abs(currentLeftWheelSpeed.forward) < forwardSpeedMax) {
            currentLeftWheelSpeed.forward += speedIncrement;
            currentRightWheelSpeed.forward += speedIncrement;

            sendMessageToArduino();      
        } else {
            clearInterval(forwardInterval);
        }
    }, intervalPeriod)
}

/**
 * Starts sideways acceleration/deceleration interval
 * @param {*} leftSpeedIncrement speed increment of the left wheels. The increment of the right wheels is -1 * leftSpeedIncrement
 * @param {*} intervalPeriod period of the interval
 */
function accelerateSideways(leftSpeedIncrement, intervalPeriod) {
    currentLeftWheelSpeed.sideways += leftSpeedIncrement;
    currentRightWheelSpeed.sideways -= leftSpeedIncrement;

    sendMessageToArduino();

    sidewaysInterval = setInterval(() => {
        if (Math.abs(currentLeftWheelSpeed.sideways) < steeringSpeedMax) {
            currentLeftWheelSpeed.sideways += leftSpeedIncrement;
            currentRightWheelSpeed.sideways -= leftSpeedIncrement;

            sendMessageToArduino();
        } else {
            clearInterval(sidewaysInterval);
        }
    }, intervalPeriod);
}

/**
 * Gradually negates the effects of the latest steering command
 * @param {*} leftSpeedIncrement speed increment of the left wheels. The increment of the right wheels is -1 * leftSpeedIncrement
 * @param {*} intervalPeriod period of the interval
 */
function graduallyNegateSteering(leftSpeedIncrement, intervalPeriod) {
    currentLeftWheelSpeed.sideways += leftSpeedIncrement;
    currentRightWheelSpeed.sideways -= leftSpeedIncrement;

    sendMessageToArduino();

    steeringNegationInterval = setInterval(() => {
        if (Math.abs(currentLeftWheelSpeed.sideways) > 0) {
            currentLeftWheelSpeed.sideways += leftSpeedIncrement;
            currentRightWheelSpeed.sideways -= leftSpeedIncrement;

            sendMessageToArduino();
        } else {
            clearInterval(sidewaysInterval);
        }
    }, intervalPeriod);
}

/**
 * Calculates the appropriate commands for left and right wheel speeds and directions and sends them to the Arduino via serial communication
 */
function sendMessageToArduino() {
    const arduinoMessageParameters = calculateArduinoMessageParameters();
    const arduinoMessages = buildArduinoMessages(arduinoMessageParameters);
    sendSerialToArduino(arduinoMessages.commandLeft);
    sendSerialToArduino(arduinoMessages.commandRight);
    console.log(arduinoMessages)
}