// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/CommandPtr.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <ShooterLookupTable.h>

#include <memory>

#include <units/time.h>

#include "commands/DriveCommands.h"

// #include "subsystems/SubClimb.h"
#include "subsystems/SubDrivetrain.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubPivotIntake.h"
#include "subsystems/SubShooter.h"

namespace Rebuilt
{
	enum MatchPeriod {
		Autonomous,
		TransitionShift,
		Shift1,
		Shift2,
		Shift3,
		Shift4,
		Endgame
	};

	struct MatchStatus
	{
		bool hubActive;
		MatchPeriod matchPeriod;
		std::string_view matchPeriodName;
		units::second_t timeLeftInPeriod;
		units::second_t timeLeftInMatch;
	};
}	 // namespace Rebuilt

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

	frc2::Command* GetAutonomousCommand();

	// Made from the example code at https://www.chiefdelphi.com/uploads/default/original/3X/b/a/ba7ccfd90bac0934e374dd4459d813cee2903942.pdf
	double Deadband(double iInput, double iThreshold, bool iSquared = false);

	Rebuilt::MatchStatus getMatchStatus();

 private:
	frc2::CommandXboxController* mCommandXboxController;
	frc2::CommandGenericHID* mCommandXboxControllerCopilot;

	// SubClimb* mSubClimb;

	SubShooter* mSubShooter = nullptr;
	SubFeeder* mSubFeeder = nullptr;

	SubDrivetrain* mDrivetrain = nullptr;

	SubIntake* mSubIntake = nullptr;
	SubPivotIntake* mSubPivotIntake = nullptr;

	DriveCommands* mDriveCommands;

	frc2::Trigger mIsInAllianceZoneTrigger;

	void ConfigureBindings();
	void ConfigureBindingsCopilot();
	void RegisterCommandsPathPlanner();
	void SetSubsystemDefaultCommands();

	frc::SendableChooser<frc2::Command*> mAutoChooser;

	nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> mNTShooterStatusTable = inst.GetTable("SmartDashboard/shooter");
	nt::StructArrayPublisher<LookupTable::ShooterStatus> mShooterStatusPublisher;
	nt::StructArraySubscriber<LookupTable::ShooterStatus> mShooterStatusSubscriber;
};
