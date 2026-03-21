// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>

#include "commands/DriveCommands.h"

#include "subsystems/SubIntake.h"
#include "subsystems/SubPivotIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/subFeeder.h"
#include "subsystems/subIndexer.h"
#include "subsystems/SubDrivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  void ConfigureWhenConnectedToDS();

  frc2::Command* GetAutonomousCommand();
  frc2::CommandPtr GetShooterWhenInZone();

// Made from the example code at https://www.chiefdelphi.com/uploads/default/original/3X/b/a/ba7ccfd90bac0934e374dd4459d813cee2903942.pdf
  double Deadband(double iInput, double iThreshold, bool iSquared = false);

  bool isHubActive();
private:
  frc2::CommandXboxController* mCommandXboxController;

  SubShooter* mSubShooter = nullptr;
  SubFeeder* mSubFeeder = nullptr;
  // SubIndexer* mSubIndexer = nullptr;

  SubDrivetrain* mDrivetrain = nullptr;

  SubIntake* mSubIntake = nullptr;
  SubPivotIntake* mSubPivotIntake = nullptr;

  DriveCommands* mDriveCommands;

  void ConfigureBindings();
  void RegisterCommandsPathPlanner();
  void SetSubsystemDefaultCommands();

  frc::SendableChooser<frc2::Command*> mAutoChooser;
};
