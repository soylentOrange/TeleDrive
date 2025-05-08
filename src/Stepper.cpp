// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <FunctionalInterrupt.h>
#include <thingy.h>

#include <functional>

// using InterruptFn = std::function<void(void)>;

#define TAG "STEPPER"

void Stepper::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;
  _srHome.setWaiting();
  _srDiag.setWaiting();
  _srStepper.setWaiting();

  // Set up IRQ for homing button
  pinMode(TMC_DIAG, INPUT);
  attachInterrupt(TMC_DIAG, [&] { _isrDiag(); }, RISING);
  // create and run a task for for getting diagnostic events from TMC2209
  _diagIRQTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { 
    LOGI(TAG, "Diag event occurred");
    _srDiag.setWaiting(); 
    _diagIRQTask->waitFor(&_srDiag); }, _scheduler, false, NULL, NULL, true);
  _diagIRQTask->enable();
  _diagIRQTask->waitFor(&_srDiag);

  // Set up IRQ for homing button
  pinMode(TMC_HOME, INPUT_PULLUP);
  attachInterrupt(TMC_HOME, [&] { _isrHome(); }, FALLING);
  // create and run a task for for getting home button presses
  _homingIRQTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { 
    LOGI(TAG, "Homing button pressed");
    _srHome.setWaiting(); 
    _homingIRQTask->waitFor(&_srHome); }, _scheduler, false, NULL, NULL, true);
  _homingIRQTask->enable();
  _homingIRQTask->waitFor(&_srHome);
}

void Stepper::end() {
  // _spooldataCallback = nullptr;
  _srHome.setWaiting();
  _srDiag.setWaiting();
  _srStepper.setWaiting();

  // end the diag task
  detachInterrupt(TMC_DIAG);
  if (_diagIRQTask != nullptr) {
    _diagIRQTask->disable();
    _diagIRQTask = nullptr;
  }

  // end the homing task
  detachInterrupt(TMC_HOME);
  if (_homingIRQTask != nullptr) {
    _homingIRQTask->disable();
    _homingIRQTask = nullptr;
  }
}
