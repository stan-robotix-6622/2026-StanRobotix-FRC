// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FeedShooter.h"

FeedShooter::FeedShooter(subFeeder * i_subFeeder) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_subFeeder);
}

// Called when the command is initially scheduled.
void FeedShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FeedShooter::Execute() {
  m_subFeeder-> setVoltage(subFeederConstants::kVoltage);
}

// Called once the command ends or is interrupted.
void FeedShooter::End(bool interrupted) 
{
  m_subFeeder->setVoltage(0_V);
}

// Returns true when the command should end.
bool FeedShooter::IsFinished() {
  return false;
}
