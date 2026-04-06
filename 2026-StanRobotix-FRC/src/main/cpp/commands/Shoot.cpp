// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

Shoot::Shoot(SubShooter* iSubShooter)
{
	mSubShooter = iSubShooter;
	AddRequirements(iSubShooter);

	mPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
	frc::SmartDashboard::PutData("commands/shoot/shooter PID", mPIDController);
}

// Called when the command is initially scheduled.
void Shoot::Initialize()
{
	mSetpointVelocity = (units::turns_per_second_t)frc::SmartDashboard::GetNumber("tunable/Shooter Setpoint", ShooterConstants::PIDConstants::setpoint.value());
	mPIDController->SetSetpoint(mSetpointVelocity.value());
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute()
{
	mPIDAdjustment = units::turns_per_second_t(mPIDController->Calculate(mSubShooter->getVelocity().value()));
	mCurrentVelocity = mSubShooter->getVelocity();
	mAdjustedVelocity = mCurrentVelocity + mPIDAdjustment;

	// frc::SmartDashboard::PutNumber("shooter/command/PID adjustment", mPIDAdjustment.value());
	// frc::SmartDashboard::PutNumber("shooter/command/current velocity", mCurrentVelocity.value());
	// frc::SmartDashboard::PutNumber("shooter/command/adjusted velocity", mAdjustedVelocity.value());

	mSubShooter->setDesiredVelocity(mAdjustedVelocity);
}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted)
{
	mSubShooter->setDesiredVelocity(0_tps);
}

// Returns true when the command should end.
bool Shoot::IsFinished()
{
	return false;
}
