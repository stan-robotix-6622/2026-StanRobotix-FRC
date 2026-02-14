
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"


Shoot::Shoot(subShooter* iSubShooter) {
  mSubShooter = iSubShooter;
  
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(iSubShooter);
  
  mPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
  frc::SmartDashboard::PutData(mPIDController);
}

// Called when the command is initially scheduled.
void Shoot::Initialize() 
{
  mPIDController->SetSetpoint(ShooterConstants::PIDConstants::setpoint.value());
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() 
{
  wDesiredVelocity = units::turns_per_second_t(mPIDController->Calculate(mSubShooter->getVelocity().value()));
  wCurrentVelocity = mSubShooter->getVelocity();

  frc::SmartDashboard::PutNumber("shooter/desired velocity", wDesiredVelocity.value());
  frc::SmartDashboard::PutNumber("shooter/current velocity", wCurrentVelocity.value());
  mSubShooter->setVelocity(wDesiredVelocity);
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