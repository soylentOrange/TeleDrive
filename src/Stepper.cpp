// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <FunctionalInterrupt.h>
#include <Preferences.h>
#include <thingy.h>

#include <functional>

#define TAG "Stepper"

void Stepper::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;
  _srHome.setWaiting();
  _srDiag.setWaiting();
  _srHoming.setWaiting();

  // Set up IRQ for homing switch
  pinMode(TMC_DIAG, INPUT);
  attachInterrupt(TMC_DIAG, [&] { _isrDiag(); }, RISING);
  // create and run a task for for getting diagnostic events from TMC2209
  _diagIRQTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { _diagIRQCallback(); }, _scheduler, false, NULL, NULL, true);
  _diagIRQTask->enable();
  _diagIRQTask->waitFor(&_srDiag);
  _movementDirection = MotorDirection::STANDSTILL;

  // Set up IRQ for homing button
  pinMode(TMC_HOME, INPUT);
  attachInterrupt(TMC_HOME, [&] { _isrHome(); }, FALLING);
  // create and run a task for for getting home button presses
  _homingIRQTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { _homingIRQCallback(); }, _scheduler, false, NULL, NULL, true);
  _homingIRQTask->enable();
  _homingIRQTask->waitFor(&_srHome);

  // register listener to website
  LOGD(TAG, "register event handler to website");
  webSite.listenWebEvent([&](JsonDocument doc) { _webEventCallback(doc); });

  // handle persistent options (auto homing...)
  LOGD(TAG, "Get persistent options from preferences...");
  Preferences preferences;
  preferences.begin("tdrive", true);
  _destination_speed = preferences.getInt("speed", 30);
  _destination_acceleration = preferences.getInt("acc", 300);
  _autoHome = preferences.getBool("ahome", false);
  preferences.end();

  // Set up a task for initializing the motor
  _initializationState = InitializationState::UNITITIALIZED;
  _driverComState = DriverComState::UNKNOWN;
  _motorState = MotorState::UNKNOWN;
  Task* initTMC2209Task = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initTMC2209(); }, _scheduler, false, NULL, NULL, true);
  initTMC2209Task->enable();
  initTMC2209Task->waitFor(webSite.getStatusRequest());
}

void Stepper::end() {
  _motorEventCallback = nullptr;
  _srHome.setWaiting();
  _srDiag.setWaiting();
  _srHoming.setWaiting();
  _initializationState = InitializationState::UNITITIALIZED;

  // end the diag-task
  detachInterrupt(TMC_DIAG);
  if (_diagIRQTask != nullptr) {
    _diagIRQTask->disable();
    _diagIRQTask = nullptr;
  }

  // end the homingIRQ-task
  detachInterrupt(TMC_HOME);
  if (_homingIRQTask != nullptr) {
    _homingIRQTask->disable();
    _homingIRQTask = nullptr;
  }

  // end the check-task
  if (_checkTMC2209Task != nullptr) {
    _checkTMC2209Task->disable();
    _checkTMC2209Task = nullptr;
  }

  // software-disable the driver
  _stepper_driver.disable();

  // possibly stop an ongoing movement
  _stepper->forceStop();
  _movementDirection = MotorDirection::STANDSTILL;
}

// callback when homing button was hit
void Stepper::_homingIRQCallback() {
  // Stop the current movement, when moving towards home
  if (_movementDirection == MotorDirection::BACKWARDS) {
    // ALWAYS stop and always remember that we hit the home button
    // adding a safety margin 0f 0.5mm
    _stepper->forceStopAndNewPosition(-STEPS_PER_MM / 2);
    _homed = true;
    _destination_position = 0;
    _movementDirection = MotorDirection::STANDSTILL;
    _stepper->moveTo(0);

    // we're initializing right now
    if (_initializationState == InitializationState::GRADIENT_HOMING) {
      _initializationState = InitializationState::GRADIENT_HOME;
      LOGI(TAG, "Hit Home while initializing");
    } else { // Initialization is probably done already
      // Yeah, homing is done!
      if (_motorState == MotorState::HOMING) {
        LOGI(TAG, "Hit Home while homing");
        _motorState = MotorState::IDLE;
        _movementDirection = MotorDirection::STANDSTILL;
        led.setMode(LED::LEDMode::IDLE);

        // send websock event
        if (_motorEventCallback != nullptr) {
          JsonDocument jsonMsg;
          jsonMsg["type"] = "motor_state";
          jsonMsg["state"] = MotorState_string_map[MotorState::HOMED].c_str();
          jsonMsg["move_state"]["position"] = 0;
          jsonMsg["move_state"]["speed"] = 0;
          jsonMsg.shrinkToFit();
          _motorEventCallback(jsonMsg);
        }
      } else if (_motorState == MotorState::DRIVING) {
        LOGW(TAG, "Hit Home while driving");
        // bring the motor to halt now
        _srStandstill.signalComplete();
      }
    }
  }

  // Wait for the next event...
  _srHome.setWaiting();
  _homingIRQTask->waitFor(&_srHome);
}

void Stepper::_diagIRQCallback() {
  // find what happened
  if (!_stepper_driver.isCommunicating()) {
    _driverComState = DriverComState::ERROR;
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);
    LOGW(TAG, "Loss of motor power");

    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg["error"] = DriverError_string_map[DriverError::POWER].c_str();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else {
    // Get global status of TMC2209
    TMC2209::GlobalStatus globalStatus = _stepper_driver.getGlobalStatus();
    if (globalStatus.uv_cp) {
      LOGW(TAG, "Charge pump under-voltage");
    } else if (globalStatus.drv_err) {
      // Some error has occurred...
      TMC2209::Status status = _stepper_driver.getStatus();
      if (status.low_side_short_a) {
        LOGW(TAG, "low_side_short_a");
      } else if (status.low_side_short_b) {
        LOGW(TAG, "low_side_short_b");
      } else if (status.open_load_a) {
        LOGW(TAG, "open_load_a");
      } else if (status.open_load_b) {
        LOGW(TAG, "open_load_b");
      } else if (status.short_to_ground_a) {
        LOGW(TAG, "short_to_ground_a");
      } else if (status.short_to_ground_b) {
        LOGW(TAG, "short_to_ground_b");
      } else if (status.over_temperature_warning) {
        LOGW(TAG, "over_temperature_warning");
      } else if (status.over_temperature_shutdown) {
        LOGW(TAG, "over_temperature_shutdown");
      }
    } else if (digitalRead(TMC_EN)) {
      LOGW(TAG, "Motor is hardware-disabled");
    }
  }
  // TODO(me): handle diagnostics

  // Wait for the next event...
  _srDiag.setWaiting();
  _diagIRQTask->waitFor(&_srDiag);
}

void Stepper::setAutoHome(bool autoHome) {
  LOGI(TAG, "AutoHoming: %s", autoHome ? "On" : "Off");
  // save if value differs from known
  if (_autoHome != autoHome) {
    _autoHome = autoHome;
    Preferences preferences;
    preferences.begin("tdrive", false);
    preferences.putBool("ahome", _autoHome);
    preferences.end();
  }
}

// re-Initialization
void Stepper::_reInitTMC2209() {
  LOGI(TAG, "Running TMC2209 re-initialization routine...");
  // 16 µSteps & 1.8°/per step --> 3200 (200*16) µSteps per rev --> with 8mm pitch --> 400 µSteps per mm
  _stepper_driver.setMicrostepsPerStep(USTEPS_PER_STEP);

  // configure TMC
  _stepper_driver.useExternalSenseResistors();
  // calculated for: [E Series Nema 17 Stepper 2A 55Ncm 1.8°](https://www.omc-stepperonline.com/e-series-nema-17-bipolar-55ncm-77-88oz-in-2a-42x48mm-4-wires-w-1m-cable-connector-17he19-2004s)
  // using the [TMC2209 Calculator](https://www.analog.com/media/en/engineering-tools/design-tools/tmc2209_calculations.xlsx)
  _stepper_driver.setRMSCurrent(1414, 0.11);

  // activate StealthChop
  _stepper_driver.setStealthChopDurationThreshold(STEALTHCHOP_THRSH);
  _stepper_driver.enableStealthChop();

  // deactivate CoolStep
  _stepper_driver.setCoolStepDurationThreshold(STEALTHCHOP_THRSH + 1);
  _stepper_driver.disableCoolStep();

  // set power saving standstill mode
  _stepper_driver.setStandstillMode(TMC2209::StandstillMode::BRAKING);

  // don't use stall guard
  _stepper_driver.setStallGuardThreshold(0);

  // set the known values for gradient and offset
  _stepper_driver.setPwmGradient(_pwmGradient);
  _stepper_driver.setPwmOffset(_pwmOffset);
  _stepper_driver.enableAutomaticCurrentScaling();
  _stepper_driver.enableAutomaticGradientAdaptation();

  // software-enable TMC2209
  _stepper_driver.enable();
  // Hardware-disable the motor
  digitalWrite(TMC_EN, HIGH);
  _stepper->setAutoEnable(true);

  // Final check of driver
  if (_stepper_driver.isSetupAndCommunicating()) {
    LOGI(TAG, "Stepper driver is setup and communicating!");
    _initializationState = InitializationState::OK;
    _driverComState = DriverComState::OK;
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);

    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else {
    LOGE(TAG, "Stepper driver setup failed!");
    _stepper->setAutoEnable(false);
    _initializationState = InitializationState::UNITITIALIZED;
    _driverComState = DriverComState::ERROR;
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);

    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

// final part of initialization
void Stepper::_initTMC2209Finished() {
  LOGD(TAG, "Final pwmAutoScale: %d", _stepper_driver.getPwmScaleAuto());
  // Hardware-disable the motor
  digitalWrite(TMC_EN, HIGH);
  // Software-disable TMC2209
  _stepper_driver.disable();

  // activate StealthChop
  _stepper_driver.setStealthChopDurationThreshold(STEALTHCHOP_THRSH);
  _stepper_driver.enableStealthChop();

  // deactivate CoolStep
  _stepper_driver.setCoolStepDurationThreshold(STEALTHCHOP_THRSH + 1);
  _stepper_driver.disableCoolStep();

  // set power saving standstill mode
  _stepper_driver.setStandstillMode(TMC2209::StandstillMode::BRAKING);

  // software-enable TMC2209
  _stepper_driver.enable();
  _stepper->setAutoEnable(true);

  // Final check of driver
  if (_stepper_driver.isSetupAndCommunicating()) {
    LOGI(TAG, "Stepper driver is setup and communicating!");
    _initializationState = InitializationState::OK;
    _driverComState = DriverComState::OK;
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);
  } else {
    LOGE(TAG, "Stepper driver setup failed!");
    _stepper->setAutoEnable(false);
    _initializationState = InitializationState::UNITITIALIZED;
    _driverComState = DriverComState::ERROR;
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);

    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }

    // bail out, the check-Task might be able to recover from this mess!
    return;
  }

  // possibly do power-on homing
  if (!_homed && _autoHome) {
    do_homing();
  } else {
    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg["move_state"]["position"] = 0;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

// gradient calibration callback
void Stepper::_checkTMC2209Gradient() {
  int16_t pwmAutoScale = _stepper_driver.getPwmScaleAuto();
  LOGD(TAG, "Check pwmAutoScale: %d", pwmAutoScale);
  if (abs(pwmAutoScale) < 10) {
    _initTMC2209Finished();
    // Remember values to skip initialization on power loss
    _pwmGradient = _stepper_driver.getPwmGradientAuto();
    _pwmOffset = _stepper_driver.getPwmOffsetAuto();
  } else {
    // Try again in half a second
    Task* optimizeGradientTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initTMC2209Gradient(); }, _scheduler, false, NULL, NULL, true);
    optimizeGradientTask->enableDelayed(500);
  }
}

// gradient calibration callback
void Stepper::_initTMC2209Gradient(bool startAdaptation) {
  // set movement speed to 250 rpm
  _stepper->setSpeedInMilliHz(HOMING_SPEED);

  // set acceleration
  _stepper->setAcceleration(HOMING_ACCELERATION);

  // we're at the homing button, move 3 mm away from home
  if ((_initializationState == InitializationState::GRADIENT_HOME) || !digitalRead(TMC_HOME)) {
    _movementDirection = MotorDirection::FORWARDS;
    _initializationState = InitializationState::GRADIENT_DEHOMING;
    _stepper->move(3 * STEPS_PER_MM);
    // Wait for 500 ms and check result
    Task* optimizeGradientDeHomingTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _checkTMC2209Gradient(); }, _scheduler, false, NULL, NULL, true);
    optimizeGradientDeHomingTask->enableDelayed(500);
  } else { // just move 2 mm towards home
    _movementDirection = MotorDirection::BACKWARDS;
    _initializationState = InitializationState::GRADIENT_HOMING;
    _stepper->move(-2 * STEPS_PER_MM);
    // Wait for 500 ms and check result
    Task* optimizeGradientHomingTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _checkTMC2209Gradient(); }, _scheduler, false, NULL, NULL, true);
    optimizeGradientHomingTask->enableDelayed(500);
  }

  // start adaptation if requested (not running yet)
  if (startAdaptation) {
    LOGD(TAG, "Starting pwmAutoScale: %d", _stepper_driver.getPwmScaleAuto());
    _stepper_driver.enableAutomaticGradientAdaptation();
  }
}

// Initial part of initialization
void Stepper::_initTMC2209() {
  // possibly delay initialization if network isn't connected to WiFi
  // like programming...
  if (eventHandler.getStatusRequest()->pending()) {
    LOGI(TAG, "Delay TMC2209 setup");
    Task* initDelayedTMC2209Task = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initTMC2209(); }, _scheduler, false, NULL, NULL, true);
    initDelayedTMC2209Task->enable();
    initDelayedTMC2209Task->waitFor(eventHandler.getStatusRequest());
    return;
  }

  // Reflect state
  _driverComState = DriverComState::UNKNOWN;
  _motorState = MotorState::UNINITIALIZED;
  _initializationState = InitializationState::UNITITIALIZED;
  led.setMode(LED::LEDMode::INITIALIZING);

  // Start communication with driver
  _stepper_driver.setup(Serial1, 115200, TMC2209::SerialAddress::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);

  // Check if the driver is responding, otherwise the power might have failed
  if (!_stepper_driver.isCommunicating()) {
    LOGW(TAG, "Driver is not communicating, delay initialization");
    _driverComState = DriverComState::ERROR;
    led.setMode(LED::LEDMode::ERROR);

    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }

    // delay initialization if driver is not communicating, yet
    Task* initDelayedStartupTMC2209Task = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initTMC2209(); }, _scheduler, false, NULL, NULL, true);
    initDelayedStartupTMC2209Task->enableDelayed(1000);
    return;
  }

  // Set up a task for continuously monitoring the driver
  if (_checkTMC2209Task == nullptr) {
    LOGD(TAG, "starting _checkTMC2209Task");
    _checkTMC2209Task = new Task(1000, TASK_FOREVER, [&] { _checkTMC2209(); }, _scheduler, false, NULL, NULL, true);
    _checkTMC2209Task->enableDelayed(1000);
  }

  LOGI(TAG, "Running TMC2209 initialization routine%s", _driverComState == DriverComState::UNKNOWN ? "..." : " again!");
  // 16 µSteps & 1.8°/per step --> 3200 (200*16) µSteps per rev --> with 8mm pitch --> 400 µSteps per mm
  _stepper_driver.setMicrostepsPerStep(USTEPS_PER_STEP);

  // configure TMC
  _stepper_driver.useExternalSenseResistors();
  // calculated for: [E Series Nema 17 Stepper 2A 55Ncm 1.8°](https://www.omc-stepperonline.com/e-series-nema-17-bipolar-55ncm-77-88oz-in-2a-42x48mm-4-wires-w-1m-cable-connector-17he19-2004s)
  // using the [TMC2209 Calculator](https://www.analog.com/media/en/engineering-tools/design-tools/tmc2209_calculations.xlsx)
  _stepper_driver.setRMSCurrent(1414, 0.11);
  // set standstill mode to use IHOLD for calibration
  _stepper_driver.setStandstillMode(TMC2209::StandstillMode::NORMAL);
  _stepper_driver.enableInverseMotorDirection();

  // enable StealthChop for initialization
  _stepper_driver.setStealthChopDurationThreshold(0);
  _stepper_driver.enableStealthChop();

  // don't use coolStep
  _stepper_driver.disableCoolStep();

  // don't use stall guard
  _stepper_driver.setStallGuardThreshold(0);

  // do automatic offset calibration
  // 1. enable motor driver and (blockingly) do one step
  digitalWrite(TMC_EN, LOW);
  _stepper_driver.enable();
  _stepper->setAutoEnable(false);
  _stepper->backwardStep(true);
  // 2. do standstill calibration (should take ~130ms)
  _stepper_driver.enableAutomaticCurrentScaling();
  // wait (non-blockingly) for 250ms and continue initialization (for gradient) in a new task
  Task* optimizeGradientTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initTMC2209Gradient(true); }, _scheduler, false, NULL, NULL, true);
  optimizeGradientTask->enableDelayed(250);
}

void Stepper::_checkTMC2209() {
  if (_stepper_driver.isSetupAndCommunicating()) {
    if (_driverComState != DriverComState::OK) {
      LOGD(TAG, "Stepper driver is setup and communicating, now!");
      _driverComState = DriverComState::OK;
      _motorState = MotorState::IDLE;
      led.setMode(LED::LEDMode::IDLE);

      // execute callback (from website)
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = getMotorState_as_string().c_str();
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (_stepper_driver.isCommunicatingButNotSetup()) {
    // check if motor is running (fastAccelStepper)
    if (_stepper->getCurrentSpeedInMilliHz() != 0) {
      _stepper->forceStop();
    }
    if (_driverComState != DriverComState::UNINITIALIZED) {
      LOGW(TAG, "Stepper driver is communicating but not setup, now!");
      _driverComState = DriverComState::UNINITIALIZED;
      _motorState = MotorState::UNINITIALIZED;
      led.setMode(LED::LEDMode::INITIALIZING);

      // execute callback (from website)
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = getMotorState_as_string().c_str();
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
    // Set up a task for (re-)initializing the driver
    if (_initializationState == InitializationState::OK) {
      Task* reInitTMC2209Task = new Task(100, TASK_ONCE, [&] { _reInitTMC2209(); }, _scheduler, false, NULL, NULL, true);
      reInitTMC2209Task->enable();
    } else {
      Task* initTMC2209Task = new Task(100, TASK_ONCE, [&] { _initTMC2209(); }, _scheduler, false, NULL, NULL, true);
      initTMC2209Task->enable();
    }
  } else {
    // check if motor is running (fastAccelStepper)
    if (_stepper->getCurrentSpeedInMilliHz() != 0) {
      _stepper->forceStop();
    }
    if (_driverComState != DriverComState::ERROR) {
      LOGE(TAG, "Stepper driver is not communicating, now!");
      _driverComState = DriverComState::ERROR;
      _motorState = MotorState::ERROR;
      led.setMode(LED::LEDMode::ERROR);

      // execute callback (from website)
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = getMotorState_as_string().c_str();
        jsonMsg["error"] = DriverError_string_map[DriverError::UNKNOWN].c_str();
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  }
}

void Stepper::_webEventCallback(JsonDocument doc) {
  LOGD(TAG, "Received Command: %s from client: %d", doc["type"].as<const char*>(), doc["origin"].as<int32_t>());

  // Move command
  if (strcmp(doc["type"].as<const char*>(), "move") == 0) {
    LOGD(TAG, "Motor shall move to %d mm at %d mm/s with %d mm/ss", doc["position"].as<int32_t>(), doc["speed"].as<int32_t>(), doc["acceleration"].as<int32_t>());

    // Can we start/update a movement?
    if ((_motorState == MotorState::DRIVING) || (_motorState == MotorState::IDLE)) {
      if (_destination_position == doc["position"].as<int32_t>() && _destination_speed == doc["speed"].as<int32_t>()) {
        LOGD(TAG, "Motor movement parameters are identical to current move!");
        // send websock event
        if (_motorEventCallback != nullptr) {
          JsonDocument jsonMsg;
          jsonMsg["type"] = "motor_state";
          jsonMsg["state"] = MotorState_string_map[MotorState::ARRIVED].c_str();
          jsonMsg.shrinkToFit();
          _motorEventCallback(jsonMsg);
        }
        return;
      }

      if (doc["speed"].as<int32_t>() == 0) {
        LOGD(TAG, "Motor speed is 0!");
        // send websock event
        if (_motorEventCallback != nullptr) {
          JsonDocument jsonMsg;
          jsonMsg["type"] = "motor_state";
          jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
          jsonMsg["warning"] = "Speed unplausible!";
          jsonMsg.shrinkToFit();
          _motorEventCallback(jsonMsg);
        }
        return;
      }
    } else {
      LOGW(TAG, "Motor movement not allowed!");
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Movement not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
      return;
    }

    start_move(doc["position"].as<int32_t>(), doc["speed"].as<int32_t>(), doc["acceleration"].as<int32_t>(), doc["origin"].as<int32_t>());
  } else if (strcmp(doc["type"].as<const char*>(), "stop") == 0) { // Stop command
    LOGD(TAG, "Motor shall be stopped");

    // Can we stop a movement?
    if ((_motorState == MotorState::DRIVING) || (_motorState == MotorState::HOMING)) {
      halt_move();
    } else {
      LOGW(TAG, "Stopping not allowed!");
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Stopping not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (strcmp(doc["type"].as<const char*>(), "home") == 0) { // Homing command
    LOGD(TAG, "Motor shall go/find home");

    // Can we start the homing procedure?
    if (_motorState == MotorState::IDLE) {
      do_homing();
    } else {
      LOGW(TAG, "Homing not allowed!");
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Homing not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (strcmp(doc["type"].as<const char*>(), "update_config") == 0) { // Config command
    LOGD(TAG, "Update config");

    // Update config
    stepper.setAutoHome(doc["autoHome"].as<bool>());

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "config";
      jsonMsg["autoHome"] = stepper.getAutoHome();
      jsonMsg["origin"] = doc["origin"].as<int32_t>();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else {
    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
      jsonMsg["warning"] = "Unknown command received!";
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

void Stepper::start_move(int32_t position, int32_t speed, int32_t acceleration, int32_t clientID) {
  LOGD(TAG, "Motor will move!");

  // save speed and/or acceleration if values differ from known
  if (_destination_speed != speed || _destination_acceleration != acceleration) {
    Preferences preferences;
    preferences.begin("tdrive", false);
    if (_destination_speed != speed) {
      preferences.putInt("speed", speed);
    }
    if (_destination_acceleration != acceleration) {
      preferences.putInt("acc", acceleration);
    }
    preferences.end();
  }

  _destination_position = position;
  _destination_speed = speed;
  _destination_acceleration = acceleration;

  // in which direction is the upcoming movement?
  if (_destination_position * STEPS_PER_MM > _stepper->getCurrentPosition()) {
    _movementDirection = MotorDirection::FORWARDS;
  } else {
    _movementDirection = MotorDirection::BACKWARDS;
  }

  // configure FastAccelStepper
  if (_stepper->setAcceleration(_destination_acceleration * STEPS_PER_MM)) {
    LOGE(TAG, "Error setting acceleration!");
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);
    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
      jsonMsg["error"] = "Motor won't move";
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else if (_stepper->setSpeedInMilliHz(_destination_speed * STEPS_PER_MM * 1000)) {
    LOGE(TAG, "Error setting speed!");
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);
    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
      jsonMsg["error"] = "Motor won't move";
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else if (_stepper->moveTo(_destination_position * STEPS_PER_MM)) {
    LOGE(TAG, "Error setting speed!");
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);
    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
      jsonMsg["error"] = "Motor won't move";
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else {
    // Update state and create monitoring tasks
    if (_motorState != MotorState::DRIVING) {
      _motorState = MotorState::DRIVING;
      led.setMode(LED::LEDMode::DRIVING);

      // update position and speed regularly
      _checkMovementTask = new Task(MOVEMENT_UPDATE_MS, TASK_FOREVER, [&] { _checkMovementCallback(); }, _scheduler, false, NULL, NULL, true);
      _checkMovementTask->enableDelayed(MOVEMENT_UPDATE_MS);

      _srStandstill.setWaiting();
      Task* checkStandstillTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _checkStandstillCallback(); }, _scheduler, false, NULL, NULL, true);
      checkStandstillTask->enable();
      checkStandstillTask->waitFor(&_srStandstill);
    }

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["origin"] = clientID;
      jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
      jsonMsg["destination"]["position"] = _destination_position;
      jsonMsg["destination"]["speed"] = _destination_speed;
      jsonMsg["destination"]["acceleration"] = _destination_acceleration;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

void Stepper::halt_move() {
  LOGD(TAG, "Motor will stop!");

  // bring the motor to halt now
  _stepper->setAcceleration(1600 * STEPS_PER_MM);
  _stepper->applySpeedAcceleration();
  _stepper->stopMove();
  _movementDirection = MotorDirection::STANDSTILL;

  // Forcefully stop driving operation
  if (_motorState == MotorState::DRIVING) {
    LOGD(TAG, "Driving Cancelled!");
    _srStandstill.signalComplete();
  } else {
    LOGD(TAG, "Movement Cancelled!");
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);
    _destination_position = _stepper->getCurrentPosition() / STEPS_PER_MM;

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::STOPPED].c_str();
      jsonMsg["move_state"]["position"] = _destination_position;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

void Stepper::do_homing() {
  LOGD(TAG, "Motor will go/find home!");

  // Check if we are already at home position
  if (!digitalRead(TMC_HOME)) {
    LOGI(TAG, "Homing not required - already there!");
    // Set position to 0 anyways...
    _movementDirection = MotorDirection::STANDSTILL;
    _stepper->setCurrentPosition(-STEPS_PER_MM / 2);
    _stepper->setAcceleration(HOMING_ACCELERATION);
    _stepper->setSpeedInMilliHz(HOMING_SPEED);
    _stepper->moveTo(0);
    _homed = true;
    _destination_position = 0;
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::HOMED].c_str();
      jsonMsg["move_state"]["position"] = 0;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else {
    _motorState = MotorState::HOMING;
    led.setMode(LED::LEDMode::HOMING);

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
      jsonMsg["move_state"]["position"] = 0;
      jsonMsg["move_state"]["speed"] = 33;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }

    LOGI(TAG, "Start Regular Homing");
    // move at 250 rpm toward the homing button
    _movementDirection = MotorDirection::BACKWARDS;
    _stepper->setAcceleration(HOMING_ACCELERATION);
    _stepper->setSpeedInMilliHz(HOMING_SPEED);
    _stepper->runBackward();
  }
}

std::string Stepper::getHomingState_as_string() {
  if (_homed) {
    return std::string("OK");
  } else if (_motorState == MotorState::HOMING) {
    return std::string("HOMING");
  } else {
    return std::string("UNHOMED");
  }
}

void Stepper::_checkMovementCallback() {
  // Get current position
  int32_t position = _stepper->getCurrentPosition() / STEPS_PER_MM;

  // send websock event
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "move_state";
    jsonMsg["position"] = position;
    jsonMsg["speed"] = _stepper->getCurrentSpeedInMilliHz() / STEPS_PER_MM / 1000;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }

  // if current position equals the destination we're done
  if (position == _destination_position) {
    LOGD(TAG, "Movement Done!");
    _srStandstill.signalComplete();
  }
}

void Stepper::_checkStandstillCallback() {
  if (_checkMovementTask != nullptr) {
    _checkMovementTask->disable();
    _checkMovementTask = nullptr;
  }

  // Handle case of premature stopping
  _destination_position = _stepper->getCurrentPosition() / STEPS_PER_MM;

  // send websock event
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "motor_state";
    jsonMsg["state"] = MotorState_string_map[MotorState::STOPPED].c_str();
    jsonMsg["move_state"]["position"] = _destination_position;
    jsonMsg["move_state"]["speed"] = 0;
    jsonMsg["destination"]["position"] = _destination_position;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }

  _movementDirection = MotorDirection::STANDSTILL;
  _motorState = MotorState::IDLE;
  led.setMode(LED::LEDMode::IDLE);
}
