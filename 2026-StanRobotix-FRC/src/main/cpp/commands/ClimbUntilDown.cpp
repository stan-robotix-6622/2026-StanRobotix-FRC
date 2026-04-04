// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbUntilDown.h"
#include "Constants.h"

ClimbUntilDown::ClimbUntilDown(SubClimb * iSubClimb) {
  // Use addRequirements() here to declare subsystem dependencies.
  mSubClimb = iSubClimb;
  AddRequirements(mSubClimb);
}

// Called when the command is initially scheduled.
void ClimbUntilDown::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimbUntilDown::Execute() {
  mSubClimb->SetSpeed(ClimbConstants::kConstantSpeed);
  mCounter++;
}

// Called once the command ends or is interrupted.
void ClimbUntilDown::End(bool interrupted) {
  mSubClimb->StopMotor();
  mSubClimb->SetDownPosition(mSubClimb->GetPosition());
  mSubClimb->SetDefaultCommand(mSubClimb->GetClimbCommand(SubClimb::Direction::Down));
}

// Returns true when the command should end.
bool ClimbUntilDown::IsFinished() {
  return mCounter > 10 && mSubClimb->GetCurrentFiltered() > ClimbConstants::kMaxCurrentFiltered;
}
