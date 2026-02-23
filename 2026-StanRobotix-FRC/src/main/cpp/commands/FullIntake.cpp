// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FullIntake.h"
#include <cmath>

FullIntake::FullIntake(SubIntake* iIntake, SubPivotIntake* iPivot, PivotIntake::StatePivotIntake itargetState) {
  // Use addRequirements() here to declare subsystem dependencies.
  if (itargetState == PivotIntake::StatePivotIntake::kDown) {
    AddCommands(
      PivotIntake(iPivot, PivotIntake::StatePivotIntake::kDown),
      Intake(iIntake)
    );
  }
  else if (itargetState == PivotIntake::StatePivotIntake::kUp) {
    iIntake->Idle();
    AddCommands(
      PivotIntake(iPivot, PivotIntake::StatePivotIntake::kUp)
    );
  };
}
