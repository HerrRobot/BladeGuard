const drivingKeyPressedColour = '#9cbaff';
const drivingKeyReleasedColour = '#BDD1FF';
const darkBlue = "#304166";
const red = "#EE4932";

const opacityEnabled = 1;
const opacityDisabled = 0.5;

const drivingKeys = new Set(['w', 'a', 's', 'd']);

const WASDKeysContainer = document.getElementById('wasd_container');
const controlPanelActionsContainer = document.getElementById('control_panel_actions');
const controlPanelActions = document.getElementsByClassName('ctrl_pan_action');
const logContentDiv = document.getElementById('log_content_div');
const sensorValuesDiv = document.getElementById('sensor_values');
const manualControlToggleCheckbox = document.getElementById('manual_control_toggle_checkbox');
const logEnableButton = document.getElementById('log_enable_button');
const sensorsEnableButton = document.getElementById('sensors_enable_button');

manualControlToggleCheckbox.checked = false;

let isLogVisible = true;
let isSensorsVisible = true;

/**
 * Make WASD keys darker when pressed
 */
document.addEventListener('keydown', (e) => {
    const lowercaseKey = e.key.toLocaleLowerCase();

    if (drivingKeys.has(lowercaseKey)) {
        document.getElementById(`${lowercaseKey}_key_wrapper`).style.backgroundColor = drivingKeyPressedColour;
    }
})

/**
 * Restore WASD keys to normal colour when released
 */
document.addEventListener('keyup', (e) => {
    const lowercaseKey = e.key.toLocaleLowerCase();

    if (drivingKeys.has(lowercaseKey)) {
        document.getElementById(`${lowercaseKey}_key_wrapper`).style.backgroundColor = drivingKeyReleasedColour;
    }
})

/**
 * Handles the pressing of the manual control toggle switch
 */
function handleManualControlToggle() {
    toggleWASDKeysVisibility();
    toggleManualActionsVisualCues()
}

/**
 * Handles the pressing of the log enable button
 */
function handleLogButtonPress() {
    toggleLogVisibility();
    toggleLogSensorButtonVisualCues('log');

    isLogVisible = !isLogVisible;
}

function hanldeSensorsButtonPress() {
    toggleSensorsVisibility();
    toggleLogSensorButtonVisualCues('sensors');

    isSensorsVisible = !isSensorsVisible;
}

/**
 * Changes the visibility of the WASD keys from 'hidden' to 'visible' and vice versa
 */
function toggleWASDKeysVisibility() {
    if (manualControlToggleCheckbox.checked) {
        WASDKeysContainer.style.visibility = 'visible';
    } else {
        WASDKeysContainer.style.visibility = 'hidden';
    }
}

/**
 * Toggles the visual cues for the manual actions. 
 * 
 * When disabled, the opacity is lowered and the cursor is crossed out. 
 * When enabled, the opacity is increased and the cursor becomes a pointer.
 */
function toggleManualActionsVisualCues() {
    if (manualControlToggleCheckbox.checked) {
        controlPanelActionsContainer.style.opacity = opacityEnabled;

        for (let i = 0; i < controlPanelActions.length; i++) {
            controlPanelActions[i].classList.add('ctrl_pan_action_hover');
            controlPanelActions[i].classList.remove('ctrl_pan_action_disable');
        }
    } else {
        controlPanelActionsContainer.style.opacity = opacityDisabled

        for (let i = 0; i < controlPanelActions.length; i++) {
            controlPanelActions[i].classList.remove('ctrl_pan_action_hover');
            controlPanelActions[i].classList.add('ctrl_pan_action_disable');
        }
    }
}

/**
 * Changes the visibility of the logger. 
 * 
 * Visibility is either 'visible' or 'hidden'.
 */
function toggleLogVisibility() {
    if (isLogVisible) {
        logContentDiv.style.visibility = 'hidden';
    } else {
        logContentDiv.style.visibility = 'visible';
    }
}

/**
 * Toggles the visual cues for the log/sensors enable/disable buttons.
 * 
 * If the log/sensors are visible, the button text is '///' and is coloured red.
 * Otherwise, the text is '\\\', coloured blue.
 * @param {*} button 
 * @returns 
 */
function toggleLogSensorButtonVisualCues(button) {
    let selectedButton;
    let selectedButtonVisibility;

    if (button === 'log') {
        selectedButton = logEnableButton;
        selectedButtonVisibility = isLogVisible;
    } else if (button === 'sensors') {
        selectedButton = sensorsEnableButton;
        selectedButtonVisibility = isSensorsVisible;
    } else {
        return;
    }

    if (selectedButtonVisibility) {
        selectedButton.innerHTML = "\\\\\\";
        selectedButton.style.color = darkBlue;
    } else {
        selectedButton.innerHTML = "///";
        selectedButton.style.color = red;
    }
}

/**
 * Changes the visibility of the distance sensors data. 
 * 
 * Visibility is either 'visible' or 'hidden'.
 */
function toggleSensorsVisibility() {
    if (isSensorsVisible) {
        sensorValuesDiv.style.visibility = 'hidden';
    } else {
        sensorValuesDiv.style.visibility = 'visible';
    }
}