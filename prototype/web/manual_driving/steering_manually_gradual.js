let allowedToSend = true;

const forwardSpeedMax = 128;
const forwardSpeedIncrement = 32;
const steeringSpeed = 32;

let currentRightWheelSpeed = 0;
let currentLeftWheelSpeed = 0;

let upadatesPerformed = 0;

const accelerationIntervalDuration = 1000; // miliseconds

const activeKeys = new Set();

let brakingMode = "BRAKE";

let interval;

if (document.getElementById("NEUTRAL").checked) {
    brakingMode = "NEUTRAL";
}

document.addEventListener('keydown', (e) => {
    if (activeKeys.has(e.key)) {
        return;
    }

    activeKeys.add(e.key);

    if (e.key === 'w') {
        currentLeftWheelSpeed += forwardSpeedIncrement;
        currentRightWheelSpeed += forwardSpeedIncrement;

        upadatesPerformed = 1;
    } else if (e.key === 'a') {
        currentLeftWheelSpeed -= steeringSpeed;
        currentRightWheelSpeed += steeringSpeed;
    } else if (e.key === 's') {
        currentLeftWheelSpeed -= forwardSpeedIncrement;
        currentRightWheelSpeed -= forwardSpeedIncrement;

        upadatesPerformed = 1;
    } else if (e.key === 'd') {
        currentLeftWheelSpeed += steeringSpeed;
        currentRightWheelSpeed -= steeringSpeed;
    } else {
        return;
    }

    calculateArduinoMessage();

    if (e.key === 'w' || e.key === 's') {
        let remainingUpdates = forwardSpeedMax / forwardSpeedIncrement - 1;

        interval = setInterval(() => {
            if (remainingUpdates <= 0 || !activeKeys.has(e.key)) {
                clearInterval(interval);
            } else {
                if (e.key === 'w') {
                    currentLeftWheelSpeed += forwardSpeedIncrement;
                    currentRightWheelSpeed += forwardSpeedIncrement;
                } else if (e.key === 's') {
                    currentLeftWheelSpeed -= forwardSpeedIncrement;
                    currentRightWheelSpeed -= forwardSpeedIncrement;
                }

                remainingUpdates -= 1;
                upadatesPerformed += 1;

                calculateArduinoMessage();
            }
        }, accelerationIntervalDuration)
    }
})

document.addEventListener('keyup', (e) => {
    activeKeys.delete(e.key);

    if (e.key === 'w') {
        currentLeftWheelSpeed -= upadatesPerformed * forwardSpeedIncrement;
        currentRightWheelSpeed -= upadatesPerformed * forwardSpeedIncrement;

        upadatesPerformed = 0;
        clearInterval(interval);
    } else if (e.key === 'a') {
        currentLeftWheelSpeed += steeringSpeed;
        currentRightWheelSpeed -= steeringSpeed;
    } else if (e.key === 's') {
        currentLeftWheelSpeed += upadatesPerformed * forwardSpeedIncrement;
        currentRightWheelSpeed += upadatesPerformed * forwardSpeedIncrement;

        upadatesPerformed = 0;
        clearInterval(interval);
    } else if (e.key === 'd') {
        currentLeftWheelSpeed -= steeringSpeed;
        currentRightWheelSpeed += steeringSpeed;
    } else {
        return;
    }

    calculateArduinoMessage();
})

function emergencyStop() {
    allowedToSend = false;

    const stopLeftMotorsMessage = "WHEEL_LEFT;NEUTRAL;0;";
    const stopRightMotorsMessage = "WHEEL_RIGHT;NEUTRAL;0;";

    // publish both to Arduino

    sendSerialToArduino(stopLeftMotorsMessage);
    sendSerialToArduino(stopRightMotorsMessage);

    document.getElementById("emergency_heading").setAttribute("style", "display: block;")
}

function calculateArduinoMessage() {
    if (currentLeftWheelSpeed < 0) {
        sendSpeedToArduino('LEFT', 'BACKWARD', -currentLeftWheelSpeed);
    } else if (currentLeftWheelSpeed > 0) {
        sendSpeedToArduino('LEFT', 'FORWARD', currentLeftWheelSpeed);
    } else {
        sendSpeedToArduino('LEFT', brakingMode, currentLeftWheelSpeed);
    }

    if (currentRightWheelSpeed < 0) {
        sendSpeedToArduino('RIGHT', 'BACKWARD', -currentRightWheelSpeed);
    } else if (currentRightWheelSpeed > 0) {
        sendSpeedToArduino('RIGHT', 'FORWARD', currentRightWheelSpeed);
    } else {
        sendSpeedToArduino('RIGHT', brakingMode, currentRightWheelSpeed);
    }
}

function sendSpeedToArduino(side, direction, speed) {
    if (!allowedToSend) {
        return;
    }

    let command = `WHEEL_${side};${direction};${speed};`;

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