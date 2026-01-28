// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbUp.h"

ClimbUp::ClimbUp(SubClimb * iSubClimb) {
  // Use addRequirements() here to declare subsystem dependencies.
  mSubClimb = iSubClimb;
  AddRequirements(mSubClimb);
  mPIDController = new frc::PIDController(ClimbConstants::kp, ClimbConstants::ki, ClimbConstants::kd);
}

// Called when the command is initially scheduled.
void ClimbUp::Initialize() {
  mPIDController->SetSetpoint(mSubClimb->GetPosition() + ClimbConstants::kPoseUp);
}

// Called repeatedly when this Command is scheduled to run
void ClimbUp::Execute() {
  mSubClimb->SetSpeed(mPIDController->Calculate(mSubClimb->GetPosition()) * ClimbConstants::kSpeedMultiplier);
}

// Called once the command ends or is interrupted.
void ClimbUp::End(bool interrupted) {
  mSubClimb->StopMotor();
}

// Returns true when the command should end.
bool ClimbUp::IsFinished() {
  return mPIDController->AtSetpoint();
}
