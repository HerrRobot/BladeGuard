let allowedToSend = true;

const forwardSpeedMax = 128;
const forwardSpeedIncrement = 8;
const steeringSpeed = 32;

let currentRightWheelSpeed = {
    forward: 0,
    sideways: 0
}

let currentLeftWheelSpeed = {
    forward: 0,
    sideways: 0
}

const accelerationIntervalDuration = 750; // miliseconds

const activeKeys = new Set();

let brakingMode = "N";

let interval;

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

        currentLeftWheelSpeed.forward += forwardSpeedIncrement;
        currentRightWheelSpeed.forward += forwardSpeedIncrement;

        interval = setInterval(() => {
            if (currentLeftWheelSpeed.forward < forwardSpeedMax) {
                currentLeftWheelSpeed.forward += forwardSpeedIncrement;
                currentRightWheelSpeed.forward += forwardSpeedIncrement;

                calculateArduinoMessage();
            } else {
                clearInterval(interval);
            }

            // if (currentRightWheelSpeed.forward < forwardSpeedMax) {
            //     currentRightWheelSpeed.forward += forwardSpeedIncrement;
            // }
        }, accelerationIntervalDuration)
    } else if (e.key === 'a') {
        if (activeKeys.has('d')) {
            return;
        }

        currentLeftWheelSpeed.sideways = -steeringSpeed;
        currentRightWheelSpeed.sideways = steeringSpeed;
    } else if (e.key === 's') {
        if (activeKeys.has('w')) {
            return;
        }

        currentLeftWheelSpeed.forward -= forwardSpeedIncrement;
        currentRightWheelSpeed.forward -= forwardSpeedIncrement;

        interval = setInterval(() => {
            if (currentLeftWheelSpeed.forward > -forwardSpeedMax) {
                currentLeftWheelSpeed.forward -= forwardSpeedIncrement;
                currentRightWheelSpeed.forward -= forwardSpeedIncrement;

                calculateArduinoMessage();
            } else {
                clearInterval(interval);
            }

            // if (currentRightWheelSpeed.forward < forwardSpeedMax) {
            //     currentRightWheelSpeed.forward += forwardSpeedIncrement;
            // }
        }, accelerationIntervalDuration)
    } else if (e.key === 'd') {
        if (activeKeys.has('a')) {
            return;
        }

        currentLeftWheelSpeed.sideways = steeringSpeed;
        currentRightWheelSpeed.sideways = -steeringSpeed;
    } else {
        return;
    }

    calculateArduinoMessage();
})

document.addEventListener('keyup', (e) => {
    activeKeys.delete(e.key);

    if (e.key === 'w' || e.key === 's') {
        clearInterval(interval);

        currentLeftWheelSpeed.forward = 0;
        currentRightWheelSpeed.forward = 0;

        clearInterval(interval);
    } else if (e.key === 'a' || e.key === 'd') {
        currentLeftWheelSpeed.sideways = 0;
        currentRightWheelSpeed.sideways = 0;
    } else {
        return;
    }

    calculateArduinoMessage();
})

function emergencyStop() {
    allowedToSend = false;

    const stopLeftMotorsMessage = "W_L;N;0;";
    const stopRightMotorsMessage = "W_R;N;0;";

    // publish both to Arduino

    sendSerialToArduino(stopLeftMotorsMessage);
    sendSerialToArduino(stopRightMotorsMessage);

    document.getElementById("emergency_heading").setAttribute("style", "display: block;")
}

function calculateArduinoMessage() {
    let currentLeftWheelSpeedTotal = currentLeftWheelSpeed.forward + currentLeftWheelSpeed.sideways;
    let currentRightWheelSpeedTotal = currentRightWheelSpeed.forward + currentRightWheelSpeed.sideways;

    // console.log(currentRightWheelSpeedTotal, currentRightWheelSpeed.forward, currentRightWheelSpeed.sideways)

    if (currentLeftWheelSpeedTotal < 0) {
        sendSpeedToArduino('L', 'B', -currentLeftWheelSpeedTotal);
    } else if (currentLeftWheelSpeedTotal > 0) {
        sendSpeedToArduino('L', 'F', currentLeftWheelSpeedTotal);
    } else {
        sendSpeedToArduino('L', brakingMode, currentLeftWheelSpeedTotal);
    }

    if (currentRightWheelSpeedTotal < 0) {
        sendSpeedToArduino('R', 'B', -currentRightWheelSpeedTotal);
    } else if (currentRightWheelSpeedTotal > 0) {
        sendSpeedToArduino('R', 'F', currentRightWheelSpeedTotal);
    } else {
        sendSpeedToArduino('R', brakingMode, currentRightWheelSpeedTotal);
    }
}

function sendSpeedToArduino(side, direction, speed) {
    if (!allowedToSend) {
        return;
    }

    if (speed > 255) {
        speed = 255;
    }

    if (speed < -255) {
        speed = -255;
    }

    let command = `W_${side};${direction};${speed};`;

    // SEND command OVER SERIAL
    sendSerialToArduino(command);
    // console.log(command)
}

function setBrakeMode(val) {
    brakingMode = val;
}

document.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') {
        emergencyStop();
    }
})