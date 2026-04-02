
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

#include "Constants.h"

Shoot::Shoot(SubShooter* iSubShooter) {
  mSubShooter = iSubShooter;
  
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(iSubShooter);
  
  mPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
  frc::SmartDashboard::PutData("shooter/command/shooter PID", mPIDController);
}

// Called when the command is initially scheduled.
void Shoot::Initialize() 
{
  mSetpointVelocity = (units::turns_per_second_t)frc::SmartDashboard::GetNumber("Shooter Setpoint", ShooterConstants::PIDConstants::setpoint.value());
  mPIDController->SetSetpoint(mSetpointVelocity.value());
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() 
{
  frc::SmartDashboard::PutNumber("shooter/command/PID adjustment", mPIDAdjustment.value());
  frc::SmartDashboard::PutNumber("shooter/command/current velocity", mCurrentVelocity.value());
  frc::SmartDashboard::PutNumber("shooter/command/adjusted velocity", mAdjustedVelocity.value());
  mSubShooter->setVelocity(mSubShooter->getAdjustedVelocity());
}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) {
  mSubShooter->setVelocity(0_tps);
}

// Returns true when the command should end.
bool Shoot::IsFinished()
{
  return false;
}