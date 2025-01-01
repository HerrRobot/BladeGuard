const rightWallSensorValueP = document.getElementById('right_wall_sensor_value');
const leftWallSensorValueP = document.getElementById('left_wall_sensor_value');
const ceilingSensorValueP = document.getElementById('ceiling_sensor_value');
const rootSensorValueP = document.getElementById('root_sensor_value');

const startupRightWallSensorValueP = document.getElementById('startup_right_wall_sensor_value');
const startupLeftWallSensorValueP = document.getElementById('startup_left_wall_sensor_value');
const startupCeilingSensorValueP = document.getElementById('startup_ceiling_sensor_value');
const startupRootSensorValueP = document.getElementById('startup_root_sensor_value');

const progressBarPercentageDiv = document.getElementById('progress_bar_inside');
const progressBarPercentageValue = document.getElementById('progress_bar_percentage_value');

let targetDistance = 18.0;

/**
 * Appends the given message to the log and scrolls down to the new message
 * @param {String} message The message to be logged
 */
function addEntryToLog(message) {
    const newLogLine = document.createElement('p');

    newLogLine.innerHTML = `> ${message}`

    logContentDiv.appendChild(newLogLine);

    logContentDiv.scrollTo({
        top: logContentDiv.scrollHeight,
        left: 0,
        behavior: "smooth",
    })
}

/**
 * Displays the sensor values given.
 * @param {Number} leftDistance distance to the left wall in cm
 * @param {Number} rightDistance distance to the right wall in cm
 * @param {Number} ceilingDistance distance to the ceiling in cm
 * @param {Number} rootDistance distance to root in m
 */
function displaySensorValues(leftDistance, rightDistance, ceilingDistance, rootDistance) {
    rightWallSensorValueP.innerHTML = `${rightDistance}cm`;
    leftWallSensorValueP.innerHTML = `${leftDistance}cm`;
    ceilingSensorValueP.innerHTML = `${ceilingDistance}cm`;
    rootSensorValueP.innerHTML = `${rootDistance}m`

    let newProgressBarPercentage = 100 * rootDistance / targetDistance;

    progressBarPercentageValue.innerHTML = newProgressBarPercentage.toFixed(1) + "%";

    if (newProgressBarPercentage < 0.5) {
        newProgressBarPercentage = 0.5;
    } else if (newProgressBarPercentage > 98.0) {
        if (newProgressBarPercentage > 98.5) {
            newProgressBarPercentage = 98.5;
        }

        progressBarPercentageDiv.style.borderRadius = '20px';
    }

    progressBarPercentageDiv.style.width = newProgressBarPercentage.toFixed(1) + "%";
}

/**
 * Displays the sensor values given, on the startup page
 * @param {Number} leftDistance distance to the left wall in cm
 * @param {Number} rightDistance distance to the right wall in cm
 * @param {Number} ceilingDistance distance to the ceiling in cm
 * @param {Number} rootDistance distance to root in m
 */
function displaySensorValuesStartupPage(leftDistance, rightDistance, ceilingDistance, rootDistance) {
    startupRightWallSensorValueP.innerHTML = `${rightDistance}cm`;
    startupLeftWallSensorValueP.innerHTML = `${leftDistance}cm`;
    startupCeilingSensorValueP.innerHTML = `${ceilingDistance}cm`;
    startupRootSensorValueP.innerHTML = `${rootDistance}m`
}