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
  mShooterPIDController = new frc::PIDController{ShooterConstants::PIDConstants::kP, ShooterConstants::PIDConstants::kI, ShooterConstants::PIDConstants::kD};
  frc::SmartDashboard::PutData("shooter/command/PID Controller", mShooterPIDController);
}

// Called when the command is initially scheduled.
void ShootDynamically::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootDynamically::Execute()
{
  units::meter_t wDistanceToTarget = 0_m;
  frc::ChassisSpeeds wRobotMovement = mDrivetrain->getFieldRelativeSpeeds();
  frc::Translation2d wTargetMovement = {0_m, 0_m};
  ShooterLookupTable::Status wShooterStatus = {0_m, 0_tps, 0_s};
  for (int i = 0; i < 5; i++)
  {
    wDistanceToTarget = (mDrivetrain->getTranslationToHub() + wTargetMovement).Norm();
    wShooterStatus = ShooterLookupTable::interpolate(wDistanceToTarget);
    wTargetMovement = {-wRobotMovement.vx * wShooterStatus.timeOfFlight,
                       -wRobotMovement.vy * wShooterStatus.timeOfFlight};
  }
  mShooterPIDController->SetSetpoint(wShooterStatus.shooterVelocity.value());

  mPIDAdjustment = units::turns_per_second_t(mShooterPIDController->Calculate(mShooter->getVelocity().value()));
  mCurrentVelocity = mShooter->getVelocity();
  mAdjustedVelocity = mCurrentVelocity + mPIDAdjustment;

  frc::SmartDashboard::PutNumber("shooter/command/PID adjustment", mPIDAdjustment.value());
  frc::SmartDashboard::PutNumber("shooter/command/current velocity", mCurrentVelocity.value());
  frc::SmartDashboard::PutNumber("shooter/command/adjusted velocity", mAdjustedVelocity.value());
  mShooter->setVelocity(mAdjustedVelocity);
  frc2::CommandScheduler::GetInstance().Schedule(mDrivetrain->Idle());
}

// Called once the command ends or is interrupted.
void ShootDynamically::End(bool interrupted)
{
  mShooter->setVelocity(0_tps);
}

// Returns true when the command should end.
bool ShootDynamically::IsFinished() {
  return false;
}
