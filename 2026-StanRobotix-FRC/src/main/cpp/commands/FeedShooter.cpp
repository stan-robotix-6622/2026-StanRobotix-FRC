// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file ks the root directory of this project.

#include "commands/FeedShooter.h"

FeedShooter::FeedShooter(SubFeeder * iSubFeeder) {
  mSubFeeder = iSubFeeder;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(mSubFeeder);
}

// Called when the command is initially scheduled.
void FeedShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FeedShooter::Execute() {
  mSubFeeder->setVoltage(FeederConstants::kDesiredVoltage);
}

// Called once the command ends or is interrupted.
void FeedShooter::End(bool interrupted) 
{
  mSubFeeder->setVoltage(0_V);
}

// Returns true when the command should end.
bool FeedShooter::IsFinished() {
  return false;
}
