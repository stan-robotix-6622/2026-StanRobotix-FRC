// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/sim/SparkRelativeEncoderSim.h>
#include <rev/sim/SparkAbsoluteEncoderSim.h>

#include <units/moment_of_inertia.h>
#include <units/area.h>
#include <units/mass.h>

#include "Constants.h"

class SwerveModuleSim{
 public:
  SwerveModuleSim(int iNeoMotorID, int iNeo550MotorID, bool iNeoInverted = false, bool iNeo550Inverted = true);

// Méthode qui retourne le SwerveModulePosition du module
  frc::SwerveModulePosition getModulePosition();
// Méthode qui retourne le SwerveModuleState du module
  frc::SwerveModuleState getModuleState();

// Méthode qui fait rouler le module à partir du SwerveModuleState désiré
  void setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation);

  void setPIDValues(double iP, double iI, double iD);

// Méthode qui met à jour le SwerveModulePosition et le SwerveModuleState du module
  void refreshModule();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  const units::square_meter_t kWheelSurface = units::square_meter_t((3 * 0.0254) * (3 * 0.0254) * std::numbers::pi);
  const units::kilogram_t kWheelMass = units::kilogram_t(0.079);
  const units::kilogram_square_meter_t kWheelDensity = kWheelSurface * kWheelMass;
  const double kTurningMotorGearBox = 12.0;

  double kP = SwerveConstants::kP;
  double kI = SwerveConstants::kI;
  double kD = SwerveConstants::kD;

  frc::DCMotor * mTurningGearBox;
  frc::DCMotor * mDriveGearBox;
  
  rev::spark::SparkMax * mTurningMotor;
  rev::spark::SparkMax * mDriveMotor;

  rev::spark::SparkMaxSim * mTurningMotorSim;
  rev::spark::SparkMaxSim * mDriveMotorSim;
  
  rev::spark::SparkMaxConfig * mDriveConfig;
  rev::spark::SparkMaxConfig * mTurningConfig;

  rev::spark::SparkRelativeEncoderSim * mDriveEncoderSim;
  rev::spark::SparkAbsoluteEncoderSim * mTurningAbsoluteEncoderSim;
  rev::spark::SparkClosedLoopController * mTurningClosedLoopController;
  
  frc::Rotation2d mTurningCurrentAngle;
  frc::SwerveModuleState mOptimizedState;

  frc::SwerveModuleState mModuleState;
  frc::SwerveModulePosition mModulePosition;
};
