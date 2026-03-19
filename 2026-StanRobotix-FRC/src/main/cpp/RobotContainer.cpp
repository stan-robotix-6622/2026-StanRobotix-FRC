// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Command.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>
#include <pathplanner/lib/events/PointTowardsZoneTrigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/DriveCommands.h"
#include "commands/PivotIntake.h"
#include "commands/FullIntake.h"
#include "commands/Shoot.h"

#include "Constants.h"

RobotContainer::RobotContainer() {
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  
  // Initialize all of your commands and subsystems here
  mSubShooter = new SubShooter{};
  mSubFeeder = new SubFeeder{};
  // mSubIndexer = new SubIndexer{};
  mDrivetrain = new SubDrivetrain{};
  mSubIntake = new SubIntake{};
  mSubPivotIntake = new SubPivotIntake{};
  
  mDriveCommands = new DriveCommands{mDrivetrain};

  // Set the default commands for all subsystems
  SetSubsystemDefaultCommands();
  // Register all relevant commands to pathplanner
  RegisterCommandsPathPlanner();
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::SetSubsystemDefaultCommands() {
  frc::SmartDashboard::PutData("Xbox Controller", &mCommandXboxController->GetHID());

  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
    [this]
    {
      mDrivetrain->driveFieldRelative(-mCommandXboxController->GetLeftY(),
                                      -mCommandXboxController->GetLeftX(),
                                      -mCommandXboxController->GetRightX(),
                                      (1 - mCommandXboxController->GetRightTriggerAxis()));
    },
    {mDrivetrain})
  );

  // mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
  //   [this]
  //   {
  //     mDrivetrain->mesureSwerveFeedforward(
  //       mCommandXboxController->GetRightTriggerAxis() * ModuleConstants::kNominalVoltage,
  //       mCommandXboxController->GetLeftTriggerAxis() * ModuleConstants::kNominalVoltage
  //     );
  //   },
  //   {mDrivetrain}
  // ));

  mSubPivotIntake->SetDefaultCommand(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kUp));
}

void RobotContainer::RegisterCommandsPathPlanner() {
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

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  
  mCommandXboxController->Button(OperatorConstants::kPivotDownButton).ToggleOnTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));
  
  mCommandXboxController->Button(OperatorConstants::kShootButton).ToggleOnTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kFeedButton).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));
  mCommandXboxController->Button(OperatorConstants::kUnstuckFuelButton).WhileTrue(mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage));
  // mCommandXboxController->Button(OperatorConstants::kIndexButton).WhileTrue(mSubIntake->getIntakeCommand());
  
  mCommandXboxController->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this]
    {if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
      {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_rad));}
    else {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 180_rad));}}));

  mCommandXboxController->Button(OperatorConstants::kResetPoseButton).WhileTrue(frc2::cmd::RunOnce([this]
    {mDrivetrain->resetPose(frc::Pose2d(2_m, 7_m, mDrivetrain->getPose().Rotation()));}));

  // mCommandXboxController->Button(7).WhileTrue(mDriveCommands->getFeedforwardCharacterizationCommand());
  // mCommandXboxController->Button(8).WhileTrue(mDriveCommands->getWheelRadiusCharacterizationCommand());
}

void RobotContainer::ConfigureWhenConnectedToDS()
{
  mDrivetrain->ConfigurePathplanner();
  mCommandXboxController->Button(7).WhileTrue(mDrivetrain->getFollowPathCommand("EightPath").Repeatedly());
  mCommandXboxController->Button(8).WhileTrue(mDrivetrain->Defer([this] {return mDrivetrain->getGoToDistanceFromHubCommand(2.3_m);}));
  
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

std::string RobotContainer::GetActiveHubColor() {
  //return frc::DriverStation::GetGameSpecificMessage();
  std::string gameData = frc::DriverStation::GetGameSpecificMessage();
  double matchTime = (double)frc::DriverStation::GetMatchTime();

  if (0 <= matchTime <= 30 || 130 <= matchTime >= 160) {
    return "Both Hubs enabled";
  }

  if (gameData.length() > 0) {
    switch (gameData[0]) {
      case 'B':
        // Blue case code
        if ((30 <= matchTime && matchTime <= 55) || (80 <= matchTime && matchTime <= 105)) {
          return "Red Hub enabled";
        }
        else {
          return "Blue Hub enabled";
        }
        break;
      case 'R':
        // Red case code
        if ((30 <= matchTime && matchTime <= 55) || (80 <= matchTime && matchTime <= 105)) {
          return "Blue Hub enabled";
        }
        else {
          return "Red Hub enabled";
        }
        break;
      default:
        return "Corrupted data";
        // This is corrupt data
        break;
    }
  }
  
  else {
    //Code for no data received yet
    return "No data";
  }
}

/*frc2::CommandPtr RobotContainer::GetShooterWhenInZone()
  {
    if()
    {
      return S;
    }
    else
    {
      return 
    }
    

  }*/

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Runs the chosen command in autonomous
  return autoChooser.GetSelected();
}