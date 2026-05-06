// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IMU.h"

#include "Constants.h"

IMU::IMU()
{
	mIMU = new ctre::phoenix6::hardware::Pigeon2{CANid::kIMUPigeonID};
	mSimIMU = &mIMU->GetSimState();
}

void IMU::advanceSimulation(units::radians_per_second_t iYawVelocity)
{
	mSimIMU->SetAngularVelocityX(0_rad_per_s);
	mSimIMU->SetAngularVelocityY(0_rad_per_s);
	mSimIMU->AddYaw(iYawVelocity * 0.02_s);
	mSimIMU->SetAngularVelocityZ(iYawVelocity);
}

frc::Rotation2d IMU::getRotation2d()
{
	return mIMU->GetRotation2d();
}

units::degree_t IMU::getAngleYaw()
{
	return mIMU->GetYaw().GetValue();
}

units::degrees_per_second_t IMU::getYawRate()
{
	return mIMU->GetAngularVelocityZWorld().GetValue();
}

void IMU::reset()
{
	mIMU->Reset();
}

void IMU::setAngleYaw(units::degree_t iAngle)
{
	mIMU->SetYaw(iAngle);
}

void IMU::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("IMU");
	builder.AddDoubleProperty("rotation rads", [this] { return getRotation2d().Radians().value(); }, [this](double iValue) { setAngleYaw(units::radian_t(iValue)); });
	builder.AddDoubleProperty("rotation degrees", [this] { return getRotation2d().Degrees().value(); }, [this](double iValue) { setAngleYaw(units::degree_t(iValue)); });
	builder.AddDoubleProperty("angular velocity rad/sec", [this] { return units::radians_per_second_t(getYawRate()).value(); }, nullptr);
	builder.AddDoubleProperty("angular velocity deg/sec", [this] { return getYawRate().value(); }, nullptr);
}
