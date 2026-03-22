// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <math.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Command.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>
#include <pathplanner/lib/events/PointTowardsZoneTrigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include "commands/DriveCommands.h"
#include "commands/PivotIntake.h"
#include "commands/FullIntake.h"
#include "commands/Shoot.h"

#include "Constants.h"

RobotContainer::RobotContainer()
{
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  frc::SmartDashboard::PutData("Xbox Controller", &mCommandXboxController->GetHID());

  frc::SmartDashboard::PutNumber("Shooter Setpoint", ShooterConstants::PIDConstants::setpoint.value());
  frc::SmartDashboard::PutNumber("Drivetrain Distance Setpoint", 3);
  // Initialize all of your commands and subsystems here
  mSubShooter = new SubShooter{};
  frc::SmartDashboard::PutData("shooter", mSubShooter);
  mSubFeeder = new SubFeeder{};
  // mSubIndexer = new SubIndexer{};
  mDrivetrain = new SubDrivetrain{};
  frc::SmartDashboard::PutData("swerve", mDrivetrain);
  mSubIntake = new SubIntake{};
  mSubPivotIntake = new SubPivotIntake{};
  frc::SmartDashboard::PutData("pivot", mSubPivotIntake);
  
  mDriveCommands = new DriveCommands{mDrivetrain};
  
  // Set the default commands for all subsystems
  SetSubsystemDefaultCommands();
  // Register all relevant commands to pathplanner
  RegisterCommandsPathPlanner();
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::SetSubsystemDefaultCommands()
{
  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
      [this]
      {
        mDrivetrain->driveFieldRelative(Deadband(-mCommandXboxController->GetLeftY(), 0.05),
                                        Deadband(-mCommandXboxController->GetLeftX(), 0.05),
                                        Deadband(-mCommandXboxController->GetRightX(), 0.05),
                                        (1 - mCommandXboxController->GetRightTriggerAxis()));
      },
      {mDrivetrain}));

  mSubPivotIntake->SetDefaultCommand(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kUp));
}

void RobotContainer::RegisterCommandsPathPlanner()
{
  pathplanner::NamedCommands::registerCommand("Pivot Up", PivotIntake(mSubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  pathplanner::NamedCommands::registerCommand("Pivot Down", PivotIntake(mSubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());
  pathplanner::NamedCommands::registerCommand("Full Intake", FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));
  pathplanner::NamedCommands::registerCommand("GoTo3mFromHub", mDrivetrain->Defer([this] {return mDrivetrain->getGoToDistanceFromHubCommand(3_m);}));
  pathplanner::NamedCommands::registerCommand("Intake", mSubIntake->getIntakeCommand());
  pathplanner::NamedCommands::registerCommand("Shoot", Shoot(mSubShooter).ToPtr());
  pathplanner::NamedCommands::registerCommand("Feed Shooter", mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));
  pathplanner::NamedCommands::registerCommand("Unstuck Feeder", mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage));
  // pathplanner::NamedCommands::registerCommand("Index Fuel", mSubIntake->getIntakeCommand());

  pathplanner::EventTrigger("Intake").WhileTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown)).OnTrue(frc2::cmd::Print("run Intake"));
  pathplanner::EventTrigger("Shoot").WhileTrue(Shoot(mSubShooter).ToPtr()).OnTrue(frc2::cmd::Print("run Shooter"));

  pathplanner::PointTowardsZoneTrigger("Hub").WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage)).OnTrue(frc2::cmd::Print("feed Shooter"));
}

void RobotContainer::ConfigureBindings()
{
  mCommandXboxController->Button(OperatorConstants::kPivotDownButton).ToggleOnTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));

  mCommandXboxController->Button(OperatorConstants::kShootButton).ToggleOnTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kFeedButton).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));
  mCommandXboxController->Button(OperatorConstants::kUnstuckFuelButton).WhileTrue(mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage));
  // mCommandXboxController->Button(OperatorConstants::kIndexButton).WhileTrue(mSubIntake->getIntakeCommand());

  mCommandXboxController->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this]
    {
      if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
      {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_rad));}
      else {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 180_rad));}
    }));

  mCommandXboxController->Button(OperatorConstants::kResetPoseButton).WhileTrue(frc2::cmd::RunOnce([this]
    { mDrivetrain->resetPose(frc::Pose2d(14_m, 4.021328_m, mDrivetrain->getPose().Rotation())); }));
}

void RobotContainer::ConfigureWhenConnectedToDS()
{
  mDrivetrain->ConfigurePathplanner();
  // Bindings that need the AutoBuilder to be configures
  // mCommandXboxController->Button(7).WhileTrue(mDrivetrain->getFollowPathCommand("EightPath").Repeatedly());
  mCommandXboxController->Button(8).WhileTrue(mDrivetrain->Defer([this] { return mDrivetrain->getGoToDistanceFromHubCommand(
  (units::meter_t)frc::SmartDashboard::GetNumber("Drivetrain Distance Setpoint", 3)); }));

  mAutoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &mAutoChooser);
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  return mAutoChooser.GetSelected();
}

double RobotContainer::Deadband(double iInput, double iThreshold, bool iSquared)
{
  if (abs(iInput) < iThreshold)
  {
    return 0.0;
  }
  if (!iSquared)
  {
    // ((iInput > 0) - (iInput < 0)) gives us the sign of iInput
    // Then we scale the value of iInput over the range [-1; iThreshold] or [iTheshold; 1]
    return (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold));
  }
  // Same as above but we square the value and keep the sign of the initial iInput
  return ((iInput > 0) - (iInput < 0)) *
    (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold)) * // Squared
    (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold));
}

bool RobotContainer::isHubActive()
{
  std::string gameData = frc::DriverStation::GetGameSpecificMessage();
  units::second_t matchTime = frc::DriverStation::GetMatchTime();
  auto mAlliance = frc::DriverStation::GetAlliance();

  if (matchTime <= 30_s || /* Autonomous and End Game */
      matchTime >= 130_s)  /* Transition Shift */ {
    return true;
  }
  if (mAlliance) {
    switch (gameData[0]) {
      case 'B': // Blue starts inactive
        if ((30_s <= matchTime && matchTime <= 55_s) || /* Shift 4 */
            (80_s <= matchTime && matchTime <= 105_s))  /* Shift 2 */ {
          return mAlliance.value() == frc::DriverStation::kBlue;
        }
        else {
          return mAlliance.value() == frc::DriverStation::kRed;
        }
      case 'R': // Red starts inactive
        if ((30_s <= matchTime && matchTime <= 55_s) || /* Shift 4 */
            (80_s <= matchTime && matchTime <= 105_s))  /* Shift 2 */ {
          return mAlliance.value() == frc::DriverStation::kRed;
        }
        else {
          return mAlliance.value() == frc::DriverStation::kBlue;
        }
      default: // If unexpected value
        return false;
    }
  }
  else { // If Alliance color not accessible
    return false;
  }
}