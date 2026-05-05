// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootVariable.h"

#include "Constants.h"

#include "frc/smartdashboard/SmartDashboard.h"

ShootVariable::ShootVariable(SubShooter* iShooter, SubDrivetrain* iDrivetrain)
{
	mShooter = iShooter;
	mDrivetrain = iDrivetrain;
	AddRequirements(mShooter);

	mShooterPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
	frc::SmartDashboard::PutData("commands/shoot in place/shooter PID", mShooterPIDController);
}

// Called when the command is initially scheduled.
void ShootVariable::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootVariable::Execute()
{
	mDistanceToTarget = mDrivetrain->getTranslationToHub().Norm();
	mShooterStatus = LookupTable::interpolate(mDistanceToTarget);

	mShooterPIDController->SetSetpoint(mShooterStatus.shooterVelocity.value());
	mShooter->setTargetVelocity(mShooterStatus.shooterVelocity);

	mPIDAdjustment = units::turns_per_second_t(mShooterPIDController->Calculate(mShooter->getVelocity().value()));
	mCurrentVelocity = mShooter->getVelocity();
	mAdjustedVelocity = mCurrentVelocity + mPIDAdjustment;

	mShooter->setVelocity(mAdjustedVelocity);
}

// Called once the command ends or is interrupted.
void ShootVariable::End(bool interrupted)
{
	mShooter->setVelocity(0_tps);
	mShooter->setTargetVelocity(0_tps);
}

// Returns true when the command should end.
bool ShootVariable::IsFinished()
{
	return false;
}
