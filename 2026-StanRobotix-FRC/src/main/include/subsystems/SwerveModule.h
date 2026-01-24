// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/SparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/RelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/velocity.h>
#include <units/angle.h>

#include "Constants.h"

class SwerveModule{
 public:
// Constructeur de la classe avec un motorID pour le Neo et un pour le Neo550
  SwerveModule(int iNeoMotorID, int iNeo550MotorID, bool iNeoInveryed = false, bool i90Deg = false);

// Méthode qui retourne le SwerveModulePosition du module
  frc::SwerveModulePosition getModulePosition();
// Méthode qui retourne le SwerveModuleState du module
  frc::SwerveModuleState getModuleState();

// Méthode qui fait rouler le module à partir du SwerveModuleState désiré
  void setDesiredState(frc::SwerveModuleState iDesiredState, double iSpeedModulation);

  void setPIDValues(double kP, double kI, double kD);

// Méthode qui met à jour le SwerveModulePosition et le SwerveModuleState du module
  void refreshModule();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax * mMotorNeo;
  rev::spark::SparkMax * mMotorNeo550;

  rev::spark::SparkMaxConfig * mNeoConfig;
  rev::spark::SparkMaxConfig * mNeo550Config;

  rev::spark::SparkClosedLoopController * mNeo550ClosedLoopController;

  rev::spark::SparkRelativeEncoder * mNeoEncoder;
  rev::spark::SparkAbsoluteEncoder * mNeo550AbsoluteEncoder;
  frc::PIDController * mNeo550PID;

  frc::Rotation2d mNeo550CurrentAngle;
  frc::SwerveModuleState mOptimizedState;

  frc::SwerveModuleState mModuleState;
  frc::SwerveModulePosition mModulePosition;
};