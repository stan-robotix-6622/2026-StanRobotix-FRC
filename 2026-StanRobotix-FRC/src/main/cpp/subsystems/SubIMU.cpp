// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIMU.h"

SubIMU::SubIMU()
{
    mIMU = new ctre::phoenix6::hardware::Pigeon2{IMUConstants::kCanID};
    if (frc::RobotBase::IsSimulation())
    {
        mIMUSim = new ctre::phoenix6::sim::Pigeon2SimState{*mIMU};
    }
}

// This method will be called once per scheduler run
void SubIMU::Periodic() {}

frc::Rotation2d SubIMU::getRotation2d()
{
    return mIMU->GetRotation2d();
}

void SubIMU::setAngleYaw(units::radian_t iYaw)
{
    mIMU->SetYaw(iYaw);
}

units::radian_t SubIMU::getAngleYaw()
{
    return mIMU->GetYaw().GetValue();
}

units::radians_per_second_t SubIMU::getYawRate()
{
    return mIMU->GetAngularVelocityZWorld().GetValue();
}

void SubIMU::resetAngle()
{
    mIMU->Reset();
}

void SubIMU::setSimAngleYaw(units::radian_t iAngle)
{
    mIMU->SetYaw(iAngle);
}

void SubIMU::setSimYawRate(units::radians_per_second_t iRate)
{
    mIMUSim->SetAngularVelocityZ(iRate);
}