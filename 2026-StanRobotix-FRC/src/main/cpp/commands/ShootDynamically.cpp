// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootDynamically.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

ShootDynamically::ShootDynamically(SubShooter* iShooter, SubDrivetrain* iDrivetrain, frc2::CommandXboxController* iJoystick)
{
	mShooter = iShooter;
	mDrivetrain = iDrivetrain;
	AddRequirements({mShooter, mDrivetrain});

	mJoystick = iJoystick;

	mShooterPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
	mRotationPIDController = new frc::PIDController{DrivetrainConstants::PIDs::kRotationP, DrivetrainConstants::PIDs::kRotationI, DrivetrainConstants::PIDs::kRotationD};
	mRotationPIDController->EnableContinuousInput(0, std::numbers::pi * 2);
	frc::SmartDashboard::PutData("shooter/command/shooter PID 2", mShooterPIDController);
	frc::SmartDashboard::PutData("shooter/command/rotation PID", mRotationPIDController);
}

// Called when the command is initially scheduled.
void ShootDynamically::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootDynamically::Execute()
{
	mRobotMovement = mDrivetrain->getFieldRelativeSpeeds();
	mTargetMovement = {0_m, 0_m};
	mShooterStatus = {0_m, 0_tps, 0_s};
	mLastCalculatedShooterVelocity = 0_tps;
	mTranslationToHub = mDrivetrain->getTranslationToHub();
	for (int i = 0; i < 5; i++) {
		mLastCalculatedShooterVelocity = mShooterStatus.shooterVelocity;
		mDistanceToTarget = (mTranslationToHub + mTargetMovement).Norm();
		mShooterStatus = LookupTable::interpolate(mDistanceToTarget);
		mTargetMovement = {-mRobotMovement.vx * mShooterStatus.timeOfFlight,
											 -mRobotMovement.vy * mShooterStatus.timeOfFlight};
	}
	mShooterPIDController->SetSetpoint(mShooterStatus.shooterVelocity.value());

	mPIDAdjustment = units::turns_per_second_t(mShooterPIDController->Calculate(mShooter->getVelocity().value()));
	mCurrentVelocity = mShooter->getVelocity();
	mAdjustedVelocity = mCurrentVelocity + mPIDAdjustment;

	mShooter->setDesiredVelocity(mAdjustedVelocity);

	mRotationPIDController->SetSetpoint(mDrivetrain->getTranslationToHub().Angle().Radians().value());
	mDrivetrain->driveFieldRelative(-mJoystick->GetLeftY(),
																	-mJoystick->GetLeftX(),
																	mRotationPIDController->Calculate(mDrivetrain->getPose().Rotation().Radians().value()),
																	(0.5 + (mJoystick->GetRightTriggerAxis() / 2)));
}

// Called once the command ends or is interrupted.
void ShootDynamically::End(bool interrupted)
{
	mShooter->setDesiredVelocity(0_tps);
}

// Returns true when the command should end.
bool ShootDynamically::IsFinished()
{
	return false;
}
