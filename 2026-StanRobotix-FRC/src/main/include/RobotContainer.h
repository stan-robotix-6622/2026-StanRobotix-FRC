// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>

#include "commands/DriveCommands.h"

#include "subsystems/SubDrivetrain.h"
#include "subsystems/subFeeder.h"
#include "subsystems/subIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubPivotIntake.h"
#include "subsystems/SubShooter.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
	RobotContainer();

	void ConfigureWhenConnectedToDS();

	frc2::Command* GetAutonomousCommand();
	frc2::CommandPtr GetShooterWhenInZone();

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
