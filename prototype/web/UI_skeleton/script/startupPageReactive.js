const startupPageMainDiv = document.getElementById('startup_main_div');
const mainPageMainDiv = document.getElementById('main_div');

const autonomousRadioButton = document.getElementById('autonomous_radio_button');
const manualRadioButton = document.getElementById('manual_radio_button');
const targetInput = document.getElementById('target_depth_input');
const startupConnectionStatusLED = document.getElementById('startup_connection_status_LED');
const startupCOnnectionStatusStrong = document.getElementById('startup_connection_status_strong');

/**
 * Handless the pressing of the START button:
 * 
 * - Tests the rosbridge connection
 * - Checks the target distance
 * - handles autonomous VS manual operation mode
 * - changes the HTML page displayed
 * - adds event listeners for manual driving
 * @returns nothing
 */
function handleStartRobotButtonPress(){
    if (!isConnectedToRosbridge) {
        alert("Make sure the robot is connected to your laptop and the rosbridge server is running.")
        return;
    }

    const targetInputValue = targetInput.value;
    
    if (!checkTargetInputValue(targetInputValue)) {
        alert("Please enter a valid target depth between 1 and 33 meters.");
        return;
    }

    targetDistance = parseFloat(targetInputValue);

    if (autonomousRadioButton.checked) {
        isAutonomousOperation = true;

        manualControlToggleCheckbox.checked = false;
    } else {
        manualControlToggleCheckbox.checked = true;
        handleManualControlToggle();
    }

    startupPageMainDiv.style.display = 'none';
    mainPageMainDiv.style.display = 'flex';

    addManualDrivingEventListeners();
}

/**
 * Handles the radio buttons for selecting the operation mode
 * @param {String} mode 'autonomous' or 'manual' operation
 */
function selectOperationMode(mode) {
    if (mode === "autonomous") {
        autonomousRadioButton.checked = true;
        manualRadioButton.checked = false;
    } else if (mode === "manual") {
        autonomousRadioButton.checked = false;
        manualRadioButton.checked = true;
    }
}

/**
 * Validation of the entered target distance
 * @param {String} value value of the input field
 * @returns boolean indicating whether the input is valid
 */
function checkTargetInputValue(value) {
    if (!value || value === "") {
        return false;
    }

    try {
        const parsedValue = parseFloat(value);

        if (parsedValue < 1 || parsedValue > 33) {
            return false;
        }

        return true;
    } catch (e) {
        return false;
    }
}

/**
 * Handles the connection status display
 * @param {Boolean} isConnected 
 */
function setConnectionIndicatorOnStartup(isConnected) {
    if (isConnected) {
        startupConnectionStatusLED.style.backgroundColor = 'green';
        startupCOnnectionStatusStrong.innerHTML = 'Connected';
    } else {
        startupConnectionStatusLED.style.backgroundColor = 'red';
        startupCOnnectionStatusStrong.innerHTML = 'Disconnected';
    }
}