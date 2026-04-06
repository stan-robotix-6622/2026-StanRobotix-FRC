// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootInPlace.h"

#include "Constants.h"

ShootInPlace::ShootInPlace(SubShooter* iShooter, SubDrivetrain* iDrivetrain) {
	mShooter = iShooter;
	mDrivetrain = iDrivetrain;
	AddRequirements({mShooter, mDrivetrain});

	mShooterPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
	mRotationPIDController = new frc::PIDController{DrivetrainConstants::PIDs::kRotationP, DrivetrainConstants::PIDs::kRotationI, DrivetrainConstants::PIDs::kRotationD};
	mRotationPIDController->EnableContinuousInput(0, std::numbers::pi * 2);
	frc::SmartDashboard::PutData("commands/shoot in place/shooter PID", mShooterPIDController);
	frc::SmartDashboard::PutData("commands/shoot in place/rotation PID", mRotationPIDController);
}

// Called when the command is initially scheduled.
void ShootInPlace::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootInPlace::Execute() {
  mDistanceToTarget = mDrivetrain->getTranslationToHub().Norm();
  mShooterStatus = LookupTable::interpolate(mDistanceToTarget);

	mShooterPIDController->SetSetpoint(mShooterStatus.shooterVelocity.value());

	mPIDAdjustment = units::turns_per_second_t(mShooterPIDController->Calculate(mShooter->getVelocity().value()));
	mCurrentVelocity = mShooter->getVelocity();
	mAdjustedVelocity = mCurrentVelocity + mPIDAdjustment;

	mShooter->setDesiredVelocity(mAdjustedVelocity);

  if (mDrivetrain->isTowardsHub()) {
    mDrivetrain->modulesXFormation();
  }
  else {
    mRotationPIDController->SetSetpoint(mDrivetrain->getTranslationToHub().Angle().Radians().value());
    mDrivetrain->driveFieldRelative(0,
                                    0,
                                    mRotationPIDController->Calculate(mDrivetrain->getPose().Rotation().Radians().value()),
                                    0.5);
  }
}

// Called once the command ends or is interrupted.
void ShootInPlace::End(bool interrupted)
{
	mShooter->setDesiredVelocity(0_tps);
}

// Returns true when the command should end.
bool ShootInPlace::IsFinished() {
  return false;
}
