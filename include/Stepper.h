// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <TMC2209.h>
#include <TaskSchedulerDeclarations.h>

class Stepper {
  public:
    Stepper() {
      _srHome.setWaiting();
      _srDiag.setWaiting();
      _srStepper.setWaiting();
    }
    void begin(Scheduler* scheduler);
    void end();

  private:
    Scheduler* _scheduler = nullptr;
    TMC2209 stepper_driver;
    StatusRequest _srHome;
    void IRAM_ATTR _isrHome() {
      if (_srHome.pending())
        _srHome.signalComplete();
    }
    StatusRequest _srDiag;
    void IRAM_ATTR _isrDiag() {
      if (_srDiag.pending())
        _srDiag.signalComplete();
    }
    StatusRequest _srStepper;
    Task* _homingIRQTask = nullptr;
    Task* _diagIRQTask = nullptr;
};
