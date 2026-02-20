// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Index.h"

Index::Index(SubIndexer * iSubIndexer) {
  mSubIndexer = iSubIndexer;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(mSubIndexer);
}

// Called when the command is initially scheduled.
void Index::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Index::Execute() 
{
  mSubIndexer->setVoltage(IndexerConstants::kDesiredVoltage);
}

// Called once the command ends or is interrupted.
void Index::End(bool interrupted) 
{
  mSubIndexer->setVoltage(0_V);
}

// Returns true when the command should end.
bool Index::IsFinished() {
  return false;
}
