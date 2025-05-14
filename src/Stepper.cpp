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
  _isHoming = false;

  // Set up IRQ for homing switch
  pinMode(TMC_DIAG, INPUT);
  attachInterrupt(TMC_DIAG, [&] { _isrDiag(); }, RISING);
  // create and run a task for for getting diagnostic events from TMC2209
  _diagIRQTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { _diagIRQCallback(); }, _scheduler, false, NULL, NULL, true);
  _diagIRQTask->enable();
  _diagIRQTask->waitFor(&_srDiag);

  // Set up IRQ for homing button
  pinMode(TMC_HOME, INPUT_PULLUP);
  attachInterrupt(TMC_HOME, [&] { _isrHome(); }, FALLING);
  // create and run a task for for getting home button presses
  _homingIRQTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { _homingIRQCallback(); }, _scheduler, false, NULL, NULL, true);
  _homingIRQTask->enable();
  _homingIRQTask->waitFor(&_srHome);

  // Register the hardware enable pin
  _stepper_driver.setHardwareEnablePin(TMC_EN);

  // register listener to website
  LOGD(TAG, "register event handler to website");
  webSite.listenWebEvent([&](JsonDocument doc) { _webEventCallback(doc); });

  // TODO(me): handle persistent options (auto homing...)
  LOGD(TAG, "Get persistent options from preferences...");
  Preferences preferences;
  preferences.begin("tdrive", true);
  _destination_speed = preferences.getInt("speed", 30);
  _destination_acceleration = preferences.getInt("acc", 300);
  preferences.end();

  // Set up a task for initializing the motor
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
  _isHoming = false;

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

  // stop the driver
  _stepper_driver.disable();
  // detach enable pin
  pinMode(TMC_EN, INPUT);
}

void Stepper::_homingIRQCallback() {
  // Stop the current movement
  if (_movementDirection == DECREASING) {
    halt_move();
    LOGI(TAG, "Homing button pressed");

    // Set position to 0 anyways...
    _stepper->setCurrentPosition(0);

    // Yeah, homing is done!
    if (_motorState == MotorState::HOMING) {
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
    }
  }

  // Wait for the next event...
  _srHome.setWaiting();
  _homingIRQTask->waitFor(&_srHome);
}

void Stepper::_diagIRQCallback() {
  LOGI(TAG, "Diag event occurred");

  // find, what happened
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
  } // else if ()

  // Wait for the next event...
  _srDiag.setWaiting();
  _diagIRQTask->waitFor(&_srDiag);
}

void Stepper::_initTMC2209() {
  LOGI(TAG, "Running TMC2209 setup%s", _driverComState == DriverComState::UNKNOWN ? "..." : " again!");
  _stepper_driver.setup(Serial1, 115200, TMC2209::SerialAddress::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  // 16 µSteps & 1.8°/per step --> 3200 (200*16) µSteps per rev --> with 8mm pitch --> 400 µSteps per mm
  _stepper_driver.setMicrostepsPerStep(USTEPS_PER_STEP);

  // configure TMC
  _stepper_driver.useExternalSenseResistors();
  // calculated for: [E Series Nema 17 Stepper 2A 55Ncm 1.8°](https://www.omc-stepperonline.com/e-series-nema-17-bipolar-55ncm-77-88oz-in-2a-42x48mm-4-wires-w-1m-cable-connector-17he19-2004s)
  // using the [TMC2209 Calculator](https://www.analog.com/media/en/engineering-tools/design-tools/tmc2209_calculations.xlsx)
  _stepper_driver.setRMSCurrent(1414, 0.11);
  _stepper_driver.setStandstillMode(TMC2209::StandstillMode::BRAKING);
  _stepper_driver.enableAutomaticCurrentScaling();
  _stepper_driver.enableAutomaticGradientAdaptation();
  _stepper_driver.enableInverseMotorDirection();

  // don't use StealthChop
  _stepper_driver.disableStealthChop();

  // don't use coolStep
  _stepper_driver.disableCoolStep();

  // We're probably fine by now
  _driverComState = DriverComState::OK;
  _motorState = MotorState::IDLE;
  led.setMode(LED::LEDMode::IDLE);
  LOGI(TAG, "Stepper driver is probably setup and communicating!");

  // execute callback (from website)
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "motor_state";
    jsonMsg["state"] = getMotorState_as_string().c_str();
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }

  // TODO(me)
  // if (!_homed) {
  //   // do power-on homing
  //   // Start the homing task
  //   // Start the homing ended task
  //   //_srHoming.setWaiting();
  // } else {
  // execute callback (from website)

  // }

  // Set up a task for continuously monitoring the driver
  if (_checkTMC2209Task == nullptr) {
    LOGD(TAG, "starting _checkTMC2209Task");
    _checkTMC2209Task = new Task(1000, TASK_FOREVER, [&] { _checkTMC2209(); }, _scheduler, false, NULL, NULL, true);
    _checkTMC2209Task->enable();
  }
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
    // TODO(me): check if motor is running (fastAccelStepper)
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
    // Set up a task for initializing the motor
    Task* reInitTMC2209Task = new Task(100, TASK_ONCE, [&] { _initTMC2209(); }, _scheduler, false, NULL, NULL, true);
    reInitTMC2209Task->enable();
  } else {
    // TODO(me): check if motor is running (fastAccelStepper)
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

  // Which kind of message was received?
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
        return;
      }
    } else {
      LOGW(TAG, "Motor movement not allowed!");
      return;
    }

    start_move(doc["position"].as<int32_t>(), doc["speed"].as<int32_t>(), doc["acceleration"].as<int32_t>(), doc["origin"].as<int32_t>());
  } else if (strcmp(doc["type"].as<const char*>(), "stop") == 0) {
    LOGD(TAG, "Motor shall be stopped");

    // Can we stop a movement?
    if ((_motorState == MotorState::DRIVING) || (_motorState == MotorState::HOMING)) {
      halt_move();
    } else {
      LOGW(TAG, "Stopping not allowed!");
    }
  } else if (strcmp(doc["type"].as<const char*>(), "home") == 0) {
    LOGD(TAG, "Motor shall go/find home");

    // Can we start the homing procedure?
    if (_motorState == MotorState::IDLE) {
      do_homing();
    } else {
      LOGW(TAG, "Homing not allowed!");
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
    _movementDirection = INCREASING;
  } else {
    _movementDirection = DECREASING;
  }

  // enable Motor and configure FastAccelStepper
  _stepper_driver.enable();
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
    _motorState = MotorState::DRIVING;
    led.setMode(LED::LEDMode::DRIVING);

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

    // set up a new task for monitoring position
    if (_motorState == MotorState::DRIVING) {
      if (_checkMovementTask != nullptr) {
        _checkMovementTask->disable();
      }

      // update position and speed regularly
      _checkMovementTask = new Task(MOVEMENT_UPDATE_MS, TASK_FOREVER, [&] { _checkMovementCallback(); }, _scheduler, false, NULL, NULL, true);
      _checkMovementTask->enable();

      _srStandstill.setWaiting();
      Task* checkStandstillTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _checkStandstillCallback(); }, _scheduler, false, NULL, NULL, true);
      checkStandstillTask->enable();
      checkStandstillTask->waitFor(&_srStandstill);
    }
  }
}

void Stepper::halt_move() {
  LOGD(TAG, "Motor will stop!");

  // stop by all means possible
  _stepper_driver.moveAtVelocity(0);
  _stepper->stopMove();

  // To be sure, disable the driver
  if (_motorState != MotorState::DRIVING) {
    _stepper_driver.disable();
  }

  // Forcefully stop driving operation
  if (_motorState == MotorState::DRIVING) {
    LOGD(TAG, "Movement Cancelled!");
    _srStandstill.signalComplete();
  }
}

void Stepper::do_homing() {
  LOGD(TAG, "Motor will go/find home!");

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

  // Check if we are already at home position
  if (!digitalRead(TMC_HOME)) {
    LOGD(TAG, "Homing not required - already there!");
    // Set position to 0 anyways...
    _stepper->setCurrentPosition(0);
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
    LOGD(TAG, "Start Regular Homing");
    _movementDirection = DECREASING;
    // move at 250 rpm (using the step generator from TMC2209) toward the homing button
    _stepper_driver.enable();
    _stepper_driver.moveAtVelocity(HOMING_SPEED);
  }
}

void Stepper::_checkMovementCallback() {
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
  _stepper_driver.disable();
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

  _motorState = MotorState::IDLE;
  led.setMode(LED::LEDMode::IDLE);
}
