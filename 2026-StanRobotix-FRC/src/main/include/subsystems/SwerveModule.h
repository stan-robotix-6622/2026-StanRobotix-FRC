// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/SparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/velocity.h>
#include <units/angle.h>

#include "Configs.h"
#include "Constants.h"

class SwerveModule : public wpi::Sendable{
 public:
  // Constructeur de la classe avec un motorID pour le Driving et un pour le Turning
  SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInveryed = false, bool iTurningInverted = true);

  // Méthode qui retourne le SwerveModulePosition du module
  frc::SwerveModulePosition getModulePosition();
  // Méthode qui retourne le SwerveModuleState du module
  frc::SwerveModuleState getModuleState();

  void InitSendable(wpi::SendableBuilder& builder) override;

  // Méthode qui fait rouler le module à partir du SwerveModuleState désiré
  void setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation);

  void setPIDValues(double kP, double kI, double kD);

  void setTurningVoltage(units::volt_t iVoltage);
  void setDrivingVoltage(units::volt_t iVoltage);

  // Méthode qui met à jour le SwerveModulePosition et le SwerveModuleState du module
  void refreshModule();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax *mDrivingMotor;
  rev::spark::SparkMax *mTurningMotor;

  rev::spark::SparkClosedLoopController *mDrivingClosedLoopController; // Not used currently (please do)
  rev::spark::SparkClosedLoopController *mTurningClosedLoopController;
  frc::PIDController *mTurningPID; // TODO: Remove if ClosedLoop working

  rev::spark::SparkRelativeEncoder *mDrivingEncoder;
  rev::spark::SparkAbsoluteEncoder *mTurningAbsoluteEncoder;

  frc::Rotation2d mTurningCurrentAngle;

  frc::SwerveModuleState mOptimizedState;

  frc::SwerveModuleState mModuleState;
  frc::SwerveModulePosition mModulePosition;
};