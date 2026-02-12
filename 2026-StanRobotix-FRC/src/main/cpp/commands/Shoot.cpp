
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"


Shoot::Shoot(subShooter* iSubShooter) {
  m_PIDController = new frc::PIDController{ PIDConstants::kP, PIDConstants::kI, PIDConstants::kD, 20_ms};
  mSubShooter = iSubShooter;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(iSubShooter);
}

// Called when the command is initially scheduled.
void Shoot::Initialize() 
{
  m_PIDController->SetSetpoint(PIDConstants::setpoint);
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() 
{
  units::turns_per_second_t wDesireVelocity = units::turns_per_second_t(subShooterConstants::kVitesseVoulue);
  mSubShooter->setVelocity(wDesireVelocity);
  units::turns_per_second_t wCurrentVelocity = units::turns_per_second_t(m_PIDController->Calculate(mSubShooter->getVelocity().value()));
}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) {}

// Returns true when the command should end.
bool Shoot::IsFinished()
{
  return false;
}