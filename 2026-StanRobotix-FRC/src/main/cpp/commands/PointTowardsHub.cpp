// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PointTowardsHub.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

PointTowardsHub::PointTowardsHub(SubDrivetrain* iDrivetrain) {
  mDrivetrain = iDrivetrain;
  AddRequirements(mDrivetrain);

	mRotationPIDController = new frc::PIDController{DrivetrainConstants::PIDs::kRotationP, DrivetrainConstants::PIDs::kRotationI, DrivetrainConstants::PIDs::kRotationD};
	mRotationPIDController->EnableContinuousInput(0, std::numbers::pi * 2);
	frc::SmartDashboard::PutData("commands/point towards hub/rotation PID", mRotationPIDController);
}

// Called when the command is initially scheduled.
void PointTowardsHub::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PointTowardsHub::Execute() {
  mRotationPIDController->SetSetpoint(mDrivetrain->getTranslationToHub().Angle().Radians().value());
  mDrivetrain->driveFieldRelative(0,
                                  0,
                                  mRotationPIDController->Calculate(mDrivetrain->getPose().Rotation().Radians().value()),
                                  0.5);
}

// Called once the command ends or is interrupted.
void PointTowardsHub::End(bool interrupted) {}

// Returns true when the command should end.
bool PointTowardsHub::IsFinished() {
  return mDrivetrain->isTowardsHub();
}
