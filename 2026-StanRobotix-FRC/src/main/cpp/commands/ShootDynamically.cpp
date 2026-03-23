// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootDynamically.h"

#include <frc/geometry/Transform2d.h>

#include "Constants.h"
#include "ShooterLookupTable.h"

ShootDynamically::ShootDynamically(SubShooter* iShooter, SubDrivetrain* iDrivetrain) {
  mShooter = iShooter;
  mDrivetrain = iDrivetrain;
  AddRequirements({mShooter, mDrivetrain});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShootDynamically::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootDynamically::Execute()
{
  units::second_t wCurrentTime = frc::Timer::GetFPGATimestamp();
  units::meter_t wDistanceToTarget = mDrivetrain->getTranslationToHub().Norm();
  frc::ChassisSpeeds wRobotMovement = mDrivetrain->getFieldRelativeSpeeds();
  frc::Transform2d wTargetMovement = frc::Transform2d(wRobotMovement.vx * (mLastTime - wCurrentTime),
                                                      wRobotMovement.vy * (mLastTime - wCurrentTime),
                                                      wRobotMovement.omega * (mLastTime - wCurrentTime));
}

// Called once the command ends or is interrupted.
void ShootDynamically::End(bool interrupted) {}

// Returns true when the command should end.
bool ShootDynamically::IsFinished() {
  return false;
}
