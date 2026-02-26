// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIMU.h"

SubIMU::SubIMU()
{
    mIMU = new ctre::phoenix6::hardware::Pigeon2{CANid::kIMUPigeonID};
}

// This method will be called once per scheduler run
void SubIMU::Periodic() {}

frc::Rotation2d SubIMU::getRotation2d()
{
    return mIMU->GetRotation2d();
}

units::degree_t SubIMU::getAngleYaw()
{
    return mIMU->GetYaw().GetValue();
}

units::degrees_per_second_t SubIMU::getYawRate()
{
    return mIMU->GetAngularVelocityZWorld().GetValue();
}

void SubIMU::resetAngle()
{
    mIMU->Reset();
}

void SubIMU::setAngleYaw(units::degree_t iAngle)
{
    mIMU->SetYaw(iAngle);
}

void SubIMU::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("IMU");
    builder.AddDoubleProperty("rotation rads", [this] {return getRotation2d().Degrees().value();}, [this] (double iValue) {setAngleYaw(units::radian_t(iValue));});
    builder.AddDoubleProperty("rotation degrees", [this] {return getRotation2d().Radians().value();}, [this] (double iValue) {setAngleYaw(units::degree_t(iValue));});
    builder.AddDoubleProperty("angular velocity rad/sec", [this] {return units::radians_per_second_t(getYawRate()).value();}, nullptr);
    builder.AddDoubleProperty("angular velocity deg/sec", [this] {return getYawRate().value();}, nullptr);
}