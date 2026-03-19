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

#include "subsystems/ExampleSubsystem.h"
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
  
  enum Auto{
    BlueCenterBumpPath,
    BlueCenterBumpReversed,
    BlueCenterTrenchPath,
    BlueLeftBumpPath,
    BlueLeftTrenchPath,
    BlueRightBumpPath,
    BlueRightTrenchPath,
    EightPath,
    RedCenterBumpPath,
    RedCenterTrenchPath,
    RedLeftBumpPath,
    RedLeftTrenchPath,
    RedRightBumpPath,
    RedRightTrenchPath,
    TestAuto
  };

  frc2::Command* GetAutonomousCommand();


private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController* mCommandXboxController;

  // The robot's subsystems are defined here...
  ExampleSubsystem mSubsystem;

  SubShooter* mSubShooter = nullptr;
  SubFeeder* mSubFeeder = nullptr;
  // SubIndexer* mSubIndexer = nullptr;

  SubDrivetrain* mDrivetrain = nullptr;

  SubIntake* mSubIntake = nullptr;
  SubPivotIntake* mSubPivotIntake = nullptr;

  DriveCommands* mDriveCommands;

  void ConfigureBindings();

  Auto mAutonomousPhase = TestAuto;
  void RegisterCommandsPathPlanner();
  void SetSubsystemDefaultCommands();

  std::string GetActiveHubColor();

  frc::SendableChooser<frc2::Command*> autoChooser;
};
