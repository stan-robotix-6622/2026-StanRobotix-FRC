// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "commands/PivotIntake.h"
#include "subsystems/SubIntake.h"

class FullIntake {
 public:
  static frc2::CommandPtr FullIntakeCommand(SubIntake* iIntake, SubPivotIntake* iPivot, PivotIntake::StatePivotIntake iTargetState);
};
