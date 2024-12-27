const drivingKeyPressedColour = '#9cbaff';
const drivingKeyReleasedColour = '#BDD1FF';
const darkBlue = "#304166";
const red = "#EE4932";
const performedActionGreen = "#005500";
const actionBeingPerformedColour = '#111111';
const actionFailedRed = '#550000';

const opacityEnabled = 1;
const opacityDisabled = 0.5;

const drivingKeys = new Set(['w', 'a', 's', 'd']);
const performedActions = new Set();
const actionsInProgress = new Set();

const WASDKeysContainer = document.getElementById('wasd_container');
const controlPanelActionsContainer = document.getElementById('control_panel_actions');
const controlPanelActions = document.getElementsByClassName('ctrl_pan_action');
const logContentDiv = document.getElementById('log_content_div');
const sensorValuesDiv = document.getElementById('sensor_values');
const manualControlToggleCheckbox = document.getElementById('manual_control_toggle_checkbox');
const logEnableButton = document.getElementById('log_enable_button');
const sensorsEnableButton = document.getElementById('sensors_enable_button');
const manualControlToggleCheckboxWrapper = document.getElementById('manual_control_toggle_checkbox_wrapper');
const manualControlToggleCheckboxSpan = document.getElementById('manual_control_toggle_checkbox_span');
const sandAction = document.getElementById('sand_action');
const cleanAction = document.getElementById('clean_action');
const glueBoxAction = document.getElementById('glue_box_action');
const glueStampAction = document.getElementById('glue_stamp_action');
const action5Action = document.getElementById('action_5_action');
const action6Action = document.getElementById('action_6_action');

manualControlToggleCheckbox.checked = false;

let isLogVisible = true;
let isSensorsVisible = true;

let isAutonomousOperation = true;

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
    if (isAutonomousOperation) {
        isAutonomousOperation = !isAutonomousOperation;

        toggleWASDKeysVisibility();
        toggleManualActionsVisualCues();
        disableManualControlToggleCheckbox();
    }
}

/**
 * Handles the pressing of the log enable button
 */
function handleLogButtonPress() {
    toggleLogVisibility();
    toggleLogSensorButtonVisualCues('log');

    isLogVisible = !isLogVisible;
}

/**
 * Handles the pressing of the sensor enable button
 */
function hanldeSensorsButtonPress() {
    toggleSensorsVisibility();
    toggleLogSensorButtonVisualCues('sensors');

    isSensorsVisible = !isSensorsVisible;
}

/**
 * Handles the pressing of a manual action button.
 * 
 * If the robot is in autonomous operation mode, the function returns imediatelly.
 * The action is not performed if another action is in progress, or if the action has already been performed.
 * Actions in progress are coloured gray, while finished actions are green.
 * @param {*} actionName name of the action to perform
 * @returns nothing
 */
function handleActionPress(actionName) {
    if (isAutonomousOperation) {
        return;
    }

    if (actionsInProgress.size > 0) {
        alert("Action in progress");
        return;
    }

    // disables performing the same action twice
    if (performedActions.has(actionName)) {
        // remove this line
        alert("already pressed this");
        return;
    }

    actionsInProgress.add(actionName);

    const actionElement = selectActionElement(actionName);

    if (!actionElement) {
        return;
    }

    actionElement.style.backgroundColor = actionBeingPerformedColour;
    actionElement.style.color = 'white';
    actionElement.style.opacity = 0.5;

    sendActionToRobot(actionName);
}

/**
 * Returns the appropriate HTML element representing the action with the provided name.
 * @param {*} actionName name of the action
 * @returns HTML div element holding the manual action
 */
function selectActionElement(actionName) {
    switch (actionName) {
        case "SAND":
            return sandAction;
        case "CLEAN":
            return cleanAction;
        case "GLUE_BOX":
            return glueBoxAction;
        case "GLUE_STAMP":
            return glueStampAction;
        case "ACTION_5":
            return action5Action;
        case "ACTION_6":
            return action6Action;
        default:
            return null;
    }
}

/**
 * Handles the response of the manual action service.
 * 
 * If successful, the action is removed from actions in progress and added to completed actions.
 * If failed, the operator is alerted. The action is coloured red and can be retried.
 * @param {{success: Boolean, action_name: String}} response Response from the server. Contains the name of the action performed and the success status.
 * @returns 
 */
function handleManualActionCompletion(response) {
    console.log(response);

    const {success, action_name} = response;

    const actionElement = selectActionElement(action_name);

    if (!success) {
        actionElement.style.backgroundColor = actionFailedRed;
        actionsInProgress.delete(action_name)

        alert(`${action_name} action failed. Press it again to retry.`);
        return;
    }

    actionsInProgress.delete(action_name)

    if (!actionElement) {
        return;
    }

    performedActions.add(action_name);

    actionElement.style.backgroundColor = performedActionGreen;
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
 * Make the manual control checkbox unclickable and add visual hints in the form of cursor and opacity
 */
function disableManualControlToggleCheckbox() {
    manualControlToggleCheckbox.disabled = true;
    manualControlToggleCheckboxWrapper.style.cursor = 'not-allowed';
    manualControlToggleCheckboxWrapper.style.opacity = 0.5;

    manualControlToggleCheckboxSpan.style.cursor = 'not-allowed';
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