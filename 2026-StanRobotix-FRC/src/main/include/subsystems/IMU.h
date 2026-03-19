// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/geometry/Rotation2d.h>
#include <ctre/phoenix6/Pigeon2.hpp>

#include <units/angle.h>
#include <units/angular_velocity.h>

class IMU : public wpi::Sendable
{
 public:
  IMU();

  frc::Rotation2d getRotation2d();
  units::degree_t getAngleYaw();
  units::degrees_per_second_t getYawRate();

  void resetAngle();
  void setAngleYaw(units::degree_t iAngle);

  void InitSendable(wpi::SendableBuilder &builder) override;

 private:
  ctre::phoenix6::hardware::Pigeon2* mIMU;
};
