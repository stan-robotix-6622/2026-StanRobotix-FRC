// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntake.h"


PivotIntake::PivotIntake(SubIntake * iIntake, SubPivotIntake * iPivotIntake, StatePivotIntake iTarget) {
  mPivotIntake = iPivotIntake;
  mPIDController = new frc::PIDController {PivotConstants::kP, PivotConstants::kI, PivotConstants::kD, 20_ms};
  mState = iTarget;

  frc::SmartDashboard::PutNumber("Arm P", PivotConstants::kP);
  frc::SmartDashboard::PutNumber("Arm I", PivotConstants::kI);
  frc::SmartDashboard::PutNumber("Arm D", PivotConstants::kD);

  AddRequirements(mPivotIntake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PivotIntake::Initialize() {
  mPIDController->Reset();
  switch (mState){
    case kUp:
      mPIDController->SetSetpoint(PivotConstants::setpointUp);
      std::cout << "Pivot du Intake Up";
      break;

    case kDown:
      mPIDController->SetSetpoint(PivotConstants::setpointDown);
      std::cout << "Pivot du Intake Down";
      break;
  }
}

// Called repeatedly when this Command is scheduled to run
void PivotIntake::Execute() {
  double wP = frc::SmartDashboard::GetNumber("Arm P", PivotConstants::kP);
  double wI = frc::SmartDashboard::GetNumber("Arm I", PivotConstants::kI);
  double wD = frc::SmartDashboard::GetNumber("Arm D", PivotConstants::kD);
  mPIDController->SetPID(wP, wI, wD);
  double wVoltage = mPIDController->Calculate(mPivotIntake->GetAngle());
  std::cout << wVoltage << std::endl;
  mPivotIntake->SetVoltage(wVoltage + (PivotConstants::kG * cos(mPivotIntake->GetAngle())));
}

// Called once the command ends or is interrupted.
void PivotIntake::End(bool interrupted) {
  std::cout << "Intake Pivot Up Fini" << std::endl;
}

// Returns true when the command should end.
bool PivotIntake::IsFinished() {
  return false;
}