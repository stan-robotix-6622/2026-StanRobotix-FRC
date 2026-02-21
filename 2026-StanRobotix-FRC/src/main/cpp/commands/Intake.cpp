// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake.h"

Intake::Intake(SubIntake* iIntake) {
  mIntake = iIntake;
  AddRequirements(mIntake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Intake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() {
  mIntake->SetSpeed(IntakeConstants::kSpeed);
}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {
  mIntake->Stop();
}

// Returns true when the command should end.
bool Intake::IsFinished() {
  return false;
}
