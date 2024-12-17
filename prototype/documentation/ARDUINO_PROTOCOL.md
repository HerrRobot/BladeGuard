# Communicating with the Arduino

This documents explains how to control the motors using the Arduino, by sending it messages via serial connection.

## Gripper

### Usage
Send these messages over the serial connection to trigger functions:
- **GRIPPER_LOAD** - Opens the gripper completely, waits 3 seconds, and closes the gripper to clamp the sensor box.
- **GRIPPER_RELEASE** - Opens the gripper completely.
- **GRIPPER_PLACE** - Lowers the gripper. Waits 1.5 seconds and releases the sensor box. Waits another 2 seconds and raises the gripper.

### Motors
- gripperServoHorisontal [**Pin 9**] - Controlls the fingers of the gripper to grip or release the sensor box.
- gripperServoVertical [**Pin 10**] - Lowers and raises the sensor box.

### Parameters

#### Motor positions:
- gripperServoHorisontalMin [**23 degrees**] [**starting position**] - Position at which the gripper fingers are completely apart.
- gripperServoHorisontalMin [**2 degrees**] - Position at which the box is secured by the gripper.
- gripperServoVerticalMin [**0 degrees**] [**starting position**] - Position at which the gripper is fully raised.
- gripperServoVerticalMax [**180 degrees**] - Position at which the gripper is fully lowered.

#### Timing (delays):
- gripperClosingDelay [**3000 ms**] - When loading the sensor box, wait this long before closing the gripper.
- gripperOpeningDelay [**1500 ms**] - When installing the sensor box, lower the gripper and wait this long before opening the fingers.
- gripperLiftingDelay [**2000 ms**] - When installing the sensor box, open the fingers and wait this long to raise the gripper.

#### State:
These state variables (all initially **false**) are used to prevent timed events from firing when they are not supposed to:
- canCloseGripper
- canOpenGripper
- canLiftGripper

## Clamp Dispenser

### Usage

Send these messages over the serial connection to trigger functions:

- **DISPENSE_CLAMP** - Dispenser goes into the downwards position, dropping a clamp. It presses on it for 5 seconds, and comes back into the upwards position to load another clamp.

### Motors
- clampDispenserServo [**pin 11**] - loads and dispenses the cable clamps.

### Parameters

#### Motor positions
Note that the servo we have only turns 170 degrees.

- gripperServoHorisontalMin [**0 degrees**] [**starting position**] - Position at which the dispenser loads a new clamp (top position).
- gripperServoHorisontalMin [**180 degrees**] - Position at which the clamp is dropped onto the ground (bottom position).

#### Timing (delays)

- clampPressingDuration [**5000 ms**] - When dispensing a clamp, press down on it for this long before loading a new one.

#### State:

These state variables (all initially false) are used to prevent timed events from firing when they are not supposed to:
- canReloadClamp

## Wheels (DC Motors)

### Usage
The messages used to control the DC motors have the form: `SIDE;MODE;SPEED;`.

- **SIDE** - Specifies which two wheels are being controlled (two left or two right). 
    - **W_L** - left wheels
    - **W_R** - right wheels
- **MODE** - Specifies the mode in which the wheels will turn (as per the INA1 and INA2 pins):
    - **F** - forwards
    - **B** - backwards
    - **N** - neutral (the robot comes to a gradual stop when given a speed of zero)
    - **BR** - break (stop the motors imediatelly)
- **SPEED** - Specifies the speed of the motor
    - Integer in range [0, 255]

### Motors
There are four DC motors. Each motor requires 3 pins: IN1, IN1, and PWM. An example for the front left wheel:
```arduino
const int wheelFrontLeftPin1 = 12;
const int wheelFrontLeftPin2 = 13;
const int wheelFrontLeftPinPWM = 3;
```

### Parameters
N/A