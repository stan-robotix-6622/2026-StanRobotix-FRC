// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntakeUp.h"


PivotIntakeUp::PivotIntakeUp(SubIntake * iIntake, SubPivotIntake * iPivotIntake) {
  mIntake = iIntake;
  mPivotIntake = iPivotIntake;
  mPIDController = new frc::PIDController {PivotConstants::kP, PivotConstants::kI, PivotConstants::kD, 20_ms};

  AddRequirements(mIntake);
  AddRequirements(mPivotIntake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PivotIntakeUp::Initialize() {
  mPIDController->Reset();
  mPIDController->SetSetpoint(PivotConstants::setpointUp);
}

// Called repeatedly when this Command is scheduled to run
void PivotIntakeUp::Execute() {
  mPivotIntake->SetVoltage(mPIDController->Calculate(mPivotIntake->GetAngle()) + (0 * cos(mPivotIntake->GetAngle()))); // il faudra definir un nombre
}

// Called once the command ends or is interrupted.
void PivotIntakeUp::End(bool interrupted) {
  mPivotIntake->Stop();
  mIntake->Stop();
  std::cout << "Intake Pivot Up Fini" << std::endl;
}

// Returns true when the command should end.
bool PivotIntakeUp::IsFinished() {
  return false;
}
