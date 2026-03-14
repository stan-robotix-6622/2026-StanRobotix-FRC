// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>

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
  mIMU = new SubIMU{};
  mDrivetrain = new SubDrivetrain{mIMU};
  mSubIntake = new SubIntake{};
  mSubPivotIntake = new SubPivotIntake{};
  
  // Set the default commands for all subsystems
  SetSubsystemDefaultCommands();
  // Register all relevant commands to pathplanner
  RegisterCommandsPathPlanner();
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::SetSubsystemDefaultCommands() {
  frc::SmartDashboard::PutData("Xbox Controller", &mCommandXboxController->GetHID());
  frc::SmartDashboard::PutData("drivetrain/IMU", mIMU);

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

  pathplanner::EventTrigger("Intake").OnTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown)).OnTrue(frc2::cmd::Print("run Intake"));
  pathplanner::EventTrigger("Shoot").OnTrue(Shoot(mSubShooter).ToPtr()).OnTrue(frc2::cmd::Print("run Shooter"));
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  
  mCommandXboxController->Button(OperatorConstants::kPivotDownButton).ToggleOnTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));
  
  mCommandXboxController->Button(OperatorConstants::kShootButton).ToggleOnTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kFeedButton).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));
  mCommandXboxController->Button(OperatorConstants::kUnstuckFuelButton).WhileTrue(mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage));
  // mCommandXboxController->Button(OperatorConstants::kIndexButton).WhileTrue(mSubIntake->getIntakeCommand());
  
  mCommandXboxController->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this]
    {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_rad));}, {mIMU}));
    mCommandXboxController->Button(OperatorConstants::kResetPoseButton).WhileTrue(frc2::cmd::RunOnce([this]
      {mDrivetrain->resetPose(frc::Pose2d(2_m, 7_m, mDrivetrain->getPose().Rotation()));}, {mIMU}));

  // mCommandXboxController->Button(7).WhileTrue(DriveCommands::getFeedforwardCharacterizationCommand(mDrivetrain));
  // mCommandXboxController->Button(8).WhileTrue(DriveCommands::getWheelRadiusCharacterizationCommand(mDrivetrain));
}

void RobotContainer::ConfigureWhenConnectedToDS()
{
  mDrivetrain->ConfigurePathplanner();
  mCommandXboxController->Button(7).WhileTrue(mDrivetrain->getFollowPathCommand("EightPath").Repeatedly());
  mCommandXboxController->Button(8).WhileTrue(mDrivetrain->Defer([this] {return mDrivetrain->getGoToDistanceFromHubCommand(2.3_m);}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
 // return mDrivetrain->getFollowPathCommand("Blue Left - Left Blue Bump");

  switch (mAutonomousPhase){
    case EightPath:
      return mDrivetrain->getFollowPathCommand("EightPath");
      break;

    case BlueCenterBumpPath:
      return mDrivetrain->getFollowPathCommand("BlueCenterBumpPath");
      break;

    case BlueCenterBumpReversed:
      return mDrivetrain->getFollowPathCommand("BlueCenterBumpReversed");
      break;

    case BlueCenterTrenchPath:
      return mDrivetrain->getFollowPathCommand("BlueCenterTrenchPath");
      break;

    case BlueLeftBumpPath:
      return mDrivetrain->getFollowPathCommand("BlueLeftBumpPath");
      break;

    case BlueLeftTrenchPath:
      return mDrivetrain->getFollowPathCommand("BlueLeftTrenchPath");
      break;

   case BlueRightBumpPath:
      return mDrivetrain->getFollowPathCommand("BlueRightBumpPath");
      break;

   case BlueRightTrenchPath:
      return mDrivetrain->getFollowPathCommand("BlueRightTrenchPath");
      break;

    case RedCenterBumpPath:
      return mDrivetrain->getFollowPathCommand("RedCenterBumpPath");
      break;

    case RedCenterTrenchPath:
      return mDrivetrain->getFollowPathCommand("RedCenterTrenchPath");
      break;

    case RedLeftBumpPath:
      return mDrivetrain->getFollowPathCommand("RedLeftBumpPath");
      break;
    
    case RedLeftTrenchPath:
      return mDrivetrain->getFollowPathCommand("RedLeftTrenchPath");
      break;

    case RedRightBumpPath:
      return mDrivetrain->getFollowPathCommand("RedRightBumpPath");
      break;

    case RedRightTrenchPath:
      return mDrivetrain->getFollowPathCommand("RedRightTrenchPath");
      break;
      
    default:
      return frc2::cmd::Print("There is no case for this automonous command");
      break;
}
}

std::string RobotContainer::GetActiveHubColor() {
  return frc::DriverStation::GetGameSpecificMessage();
}