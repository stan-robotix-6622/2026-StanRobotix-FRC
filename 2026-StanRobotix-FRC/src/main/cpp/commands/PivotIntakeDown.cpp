// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntakeDown.h"

PivotIntakeDown::PivotIntakeDown(SubIntake * iIntake, SubPivotIntake * iPivotIntake) {
  mIntake = iIntake;
  mPivotIntake = iPivotIntake;
  mPIDController = new frc::PIDController {PivotConstants::kP, PivotConstants::kI, PivotConstants::kD, 20_ms};

  AddRequirements(mIntake);
  AddRequirements(mPivotIntake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PivotIntakeDown::Initialize() {
  mPIDController->Reset();
  mPIDController->SetSetpoint(PivotConstants::setpointDown);
}

// Called repeatedly when this Command is scheduled to run
void PivotIntakeDown::Execute() {
  mIntake->SetVoltage(1); // il faudra definir un nombre
  mPivotIntake->SetVoltage(mPIDController->Calculate(mPivotIntake->GetAngle()) + (0 * cos(mPivotIntake->GetAngle()))); // il faudra definir un nombre
}

// Called once the command ends or is interrupted.
void PivotIntakeDown::End(bool interrupted) {
  mPivotIntake->Stop();
  mIntake->Stop();
  std::cout << "Intake Pivot Down Fini" << std::endl;
}

// Returns true when the command should end.
bool PivotIntakeDown::IsFinished() {
  return false;
}
