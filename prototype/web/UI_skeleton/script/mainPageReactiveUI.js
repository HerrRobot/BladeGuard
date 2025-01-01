const drivingKeyPressedColour = '#9cbaff';
const drivingKeyReleasedColour = '#BDD1FF';
const darkBlue = "#304166";
const red = "#EE4932";
const performedActionGreen = "#005500";
const actionBeingPerformedColour = '#111111';
const actionFailedRed = '#550000';

const opacityEnabled = 1;
const opacityDisabled = 0.5;

const actionDuration = 5000; //ms
const actionTimeoutTolerance = 3;

const drivingKeys = new Set(['w', 'a', 's', 'd']);
const performedActions = new Set();
const actionsInProgress = new Set();
const actionNames = new Set(['SAND', 'CLEAN', 'GLUE_BOX', 'GLUE_STAMP', 'ACTION_5', 'ACTION_6']);

const actionResponseMap = new Map();

let actionPgrogressBarInterval;
let actionTimeoutObject;

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
const sandActionProgressBar = document.getElementById('sand_action_progress_bar_body');
const cleanActionProgressBar = document.getElementById('clean_action_progress_bar_body');
const glueBoxActionProgressBar = document.getElementById('glue_box_action_progress_bar_body');
const glueStampActionProgressBar = document.getElementById('glue_stamp_action_progress_bar_body');
const action5ActionProgressBar = document.getElementById('action_5_action_progress_bar_body');
const action6ActionProgressBar = document.getElementById('action_6_action_progress_bar_body');

manualControlToggleCheckbox.checked = false;

let isLogVisible = true;
let isSensorsVisible = true;

let isAutonomousOperation = true;

setup();

/**
 * Initialise the map of action names to their next response ID
 */
function setup() {
    actionNames.forEach((elem) => {
        actionResponseMap.set(elem, 0);
    })
}

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
        clearInterval(actionPgrogressBarInterval);
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
 * @param {String} actionName name of the action to perform
 * @returns nothing
 */
function handleActionPress(actionName) {
    if (isAutonomousOperation) {
        return;
    }

    if (actionsInProgress.size > 0) {
        alert("An action is currently in progress. You must wait for it to complete or time out.");
        return;
    }

    // alert the user if the action has been performed already
    if (performedActions.has(actionName)) {
        if(!confirm(`${actionName.replaceAll("_", " ")} action has already been completed. Are you sure you want to re-do it?`)){
            return;
        }

        performedActions.delete(actionName);
    }

    actionsInProgress.add(actionName);

    setActionStartedVisualCues(actionName);

    sendActionToRobot(actionName, actionResponseMap.get(actionName));

    setActionTimeout(actionName);

    startActionProgressBar(actionName);
}

/**
 * Set the visual cues for an action which was started
 * @param {String} actionName name of the action which was started
 * @returns 
 */
function setActionStartedVisualCues(actionName) {
    const actionElement = selectActionElement(actionName);

    if (!actionElement) {
        return;
    }

    actionElement.style.backgroundColor = actionBeingPerformedColour;
    actionElement.style.color = 'white';
    actionElement.style.opacity = 0.5;
}

/**
 * Starts a timeout for a manual action. If the time runs out before getting a response from the robot, the action is assumed to have failed.
 * @param {String} actionName the name of the action 
 * @returns nothing
 */
function setActionTimeout(actionName) {
    const { actionDuration } = selectActionDurationAndPgrogressBarElement(actionName);

    if (actionDuration < 0) {
        return;
    }

    actionTimeoutObject = setTimeout(() => {
        actionResponseMap.set(actionName, actionResponseMap.get(actionName) + 1);

        const actionElement = selectActionElement(actionName);

        actionsInProgress.delete(actionName);
        clearActionProgressBar(actionName);
        actionElement.style.backgroundColor = actionFailedRed;

        alert(`The robot took too long to perform the ${actionName.replaceAll("_", " ")} action. It probably failed. Press it again to retry.`);
    }, actionDuration * actionTimeoutTolerance);
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
 * Animates a progress bar for the action being currently performed.
 * The progress bar is only an _estimate_ of the actual progress, as it is purely based on the elapsed time since the start of the action,
 * and the time the action is believed to take.
 * @param {String} actionName The name of the action being performed
 * @returns nothing
 */
function startActionProgressBar(actionName) {
    const { actionDuration, progressBarBody } = selectActionDurationAndPgrogressBarElement(actionName);

    if (!progressBarBody) {
        return;
    }

    progressBarBody.parentElement.style.visibility = 'visible';

    let actionProgress = 0;

    actionPgrogressBarInterval = setInterval(() => {
        if (actionProgress <= 100) {
            progressBarBody.style.width = `${actionProgress}%`;

            actionProgress += 1;
        } else if (actionProgress <= 105) {
            actionProgress += 1;
        } else {
            actionProgress = 90;
        }
    }, actionDuration / 100);
}

/**
 * Clears the action progress bar after the action has been completed or failed, and resets it to 0%.
 * @param {String} actionName Name of the action with the progress bar.
 * @returns nothing
 */
function clearActionProgressBar(actionName) {
    clearInterval(actionPgrogressBarInterval);

    const { progressBarBody } = selectActionDurationAndPgrogressBarElement(actionName);

    if (!progressBarBody) {
        return;
    }

    progressBarBody.parentElement.style.visibility = 'hidden';
    progressBarBody.style.width = '0%';
}

/**
 * Returns the duration and progress bar HTML element of the specified action.
 * @param {String} actionName Name of the action
 * @returns {{actionDuration: Number, progressBarBody: HTMLElement | null}} Duration and progress bar element of the action
 */
function selectActionDurationAndPgrogressBarElement(actionName){
    switch (actionName) {
        case "SAND":
            return {
                actionDuration,
                progressBarBody: sandActionProgressBar
            }
        case "CLEAN":
            return {
                actionDuration,
                progressBarBody: cleanActionProgressBar
            }
        case "GLUE_BOX":
            return {
                actionDuration,
                progressBarBody: glueBoxActionProgressBar
            }
        case "GLUE_STAMP":
            return {
                actionDuration,
                progressBarBody: glueStampActionProgressBar
            }
        case "ACTION_5":
            return {
                actionDuration,
                progressBarBody: action5ActionProgressBar
            }
        case "ACTION_6":
            return {
                actionDuration,
                progressBarBody: action6ActionProgressBar
            }
        default:
            return {
                actionDuration: -1,
                progressBarBody: null
            }
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

    const {success, action_name, response_id} = response;

    const currentResponseId = actionResponseMap.get(action_name);

    if(currentResponseId !== response_id) {
        return;
    }

    clearTimeout(actionTimeoutObject);

    const actionElement = selectActionElement(action_name);

    actionsInProgress.delete(action_name);

    clearActionProgressBar(action_name);

    if (!success) {
        actionElement.style.backgroundColor = actionFailedRed;

        alert(`${action_name.replaceAll("_", " ")} action failed. Press it again to retry.`);
        return;
    }

    if (!actionElement) {
        return;
    }

    performedActions.add(action_name);

    actionElement.style.backgroundColor = performedActionGreen;

    clearActionProgressBar(action_name);

    actionResponseMap.set(action_name, actionResponseMap.get(action_name) + 1)
}

/**
 * Update the status of an action based on incoming logger messages
 * @param {String} message incomming message from the logger
 * @returns nothing
 */
function updateActionStatus(message) {
    if (!isAutonomousOperation) {
        return;
    }

    const { actionName, status } = selectActionNameAndStatusFromLogMessage(message);

    if (!actionName || !status) {
        return;
    }

    if (status === "STARTED") {
        setActionStartedVisualCues(actionName)
        startActionProgressBar(actionName);
    } else if (status === "COMPLETED") {
        performedActions.add(actionName);
        const element = selectActionElement(actionName);
        console.log(element);
        element.style.backgroundColor = performedActionGreen;
        clearActionProgressBar(actionName);
    } else if (status === "FAILED") {
        selectActionElement(actionName).style.backgroundColor.actionFailedRed;
        clearActionProgressBar(actionName);
    }
}

/**
 * Get the name of the action and its completion status (STARTED, COMPLETED, FAILED) based on a logger message
 * @param {String} message logger message
 * @returns {{actionName: String, status: String}} name of the action and its status
 */
function selectActionNameAndStatusFromLogMessage(message) {
    switch (message) {
        case "SENSOR - STARTED SANDING":
            return {
                actionName: "SAND",
                status: "STARTED"
            }
        
        case "SENSOR - FINISHED SANDING":
            return {
                actionName: "SAND",
                status: "COMPLETED"
            }

        case "SENSOR - STARTED CLEANING":
            return {
                actionName: "CLEAN",
                status: "STARTED"
            }

        case "SENSOR - FINISHED CLEANING":
            return {
                actionName: "CLEAN",
                status: "COMPLETED"
            }

        case "SENSOR - STARTED GLUING":
            return {
                actionName: "GLUE_BOX",
                status: "STARTED"
            }

        case "SENSOR - FINISHED GLUING":
            return {
                actionName: "GLUE_BOX",
                status: "COMPLETED"
            }

        case "CABLE - DISPENSING GLUE":
            return {
                actionName: "GLUE_STAMP",
                status: "STARTED"
            }

        case "CABLE - PRESSING DOWN":
            return {
                actionName: "GLUE_STAMP",
                status: "COMPLETED"
            }
    
        default:
            return {
                actionName: null,
                status: null
            };
    }
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