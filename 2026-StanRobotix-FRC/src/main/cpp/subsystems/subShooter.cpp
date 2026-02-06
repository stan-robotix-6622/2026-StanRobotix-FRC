// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subShooter.h"



subShooter::subShooter()
{
    mPIDcontroller = new frc::PIDController{PIDConstants::kP, PIDConstants::kI, PIDConstants::kD};
    mShooterController =  new rev::spark::SparkMax{subShooterConstants::kCANid, rev::spark::SparkLowLevel::MotorType::kBrushless};
}

// This method will be called once per scheduler run
void subShooter::Periodic() {}

void subShooter::setVelocity(kV, kA)
{
    m_feedforward.Calculate(SparkMaxSim.GetVelocity(), nextVelocity);
};

