// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"

Climb::Climb(SubClimb * iSubClimb, ClimbDirection iDirection) {
  // Use addRequirements() here to declare subsystem dependencies.
  mSubClimb = iSubClimb;
  AddRequirements(mSubClimb);
  mPIDController = new frc::PIDController(ClimbConstants::kP, ClimbConstants::kI, ClimbConstants::kD);
  mDirection = iDirection;
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  switch (mDirection) {
    case Up:
      mPIDController->SetSetpoint(mSubClimb->GetPosition() + ClimbConstants::kPoseUp);
      break;
    case Down:
      mPIDController->SetSetpoint(mSubClimb->GetPosition() - ClimbConstants::kPoseUp);
      break;
    default:
      break;
  }
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  mSubClimb->SetSpeed(mPIDController->Calculate(mSubClimb->GetPosition()) * ClimbConstants::kSpeedMultiplier);
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  mSubClimb->StopMotors();
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return mPIDController->AtSetpoint();
}
