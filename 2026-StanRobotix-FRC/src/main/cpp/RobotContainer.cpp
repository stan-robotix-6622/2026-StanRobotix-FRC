// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include <iostream>

#include "RobotContainer.h"

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/PivotIntake.h"
#include "commands/FullIntake.h"
#include "commands/FeedShooter.h"
#include "commands/Shoot.h"
#include "commands/Index.h"
#include "commands/GoToDistanceFromHub.h"

RobotContainer::RobotContainer() {
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  frc::SmartDashboard::PutData("Xbox Controller", &mCommandXboxController->GetHID());

  // Initialize all of your commands and subsystems here
  mSubShooter = new SubShooter{};
  mSubFeeder = new SubFeeder{};
  // mSubIndexer = new SubIndexer{};
  mIMU = new SubIMU{};
  frc::DataLogManager::Log("Debut initialisation Drivetrain");
  mDrivetrain = new SubDrivetrain{mIMU};
  frc::DataLogManager::Log("Drivetrain initialise");
  m_SubIntake = new SubIntake{};
  m_SubPivotIntake = new SubPivotIntake{};

  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
      [this]
      {
        mDrivetrain->driveFieldRelative(-mCommandXboxController->GetLeftY(),
                                        -mCommandXboxController->GetLeftX(),
                                        -mCommandXboxController->GetRightX(),
                                        (1 - mCommandXboxController->GetRightTriggerAxis()) / 4);
      },
      {mDrivetrain}));

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

  mIMU->SetDefaultCommand(frc2::cmd::Run(
      [this]
      {
        frc::SmartDashboard::PutNumber("Drivetrain/Robot Rotation Degrees", mIMU->getRotation2d().Degrees().value());
      },
      {mIMU}));

  m_SubPivotIntake->SetDefaultCommand(frc2::cmd::Run([this]
  {
    m_SubPivotIntake->KeepPosition();
  }, {m_SubPivotIntake}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // mCommandXboxController->Button(OperatorConstants::kPivotUpButton).WhileTrue(PivotIntake(m_SubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  // mCommandXboxController->Button(OperatorConstants::kPivotDownButton).WhileTrue(PivotIntake(m_SubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kPivotUpButton).WhileTrue(FullIntake(m_SubIntake, m_SubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kPivotDownButton).WhileTrue(FullIntake(m_SubIntake, m_SubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());

  // mCommandXboxController->Button(7).WhileTrue(mDrivetrain->getFollowPathCommand("'8' Path").Repeatedly());
  
  mCommandXboxController->Button(OperatorConstants::kShootButton).WhileTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kFeedButton).WhileTrue(FeedShooter(mSubFeeder).ToPtr());
  // mCommandXboxController->Button(OperatorConstants::kIndexButton).WhileTrue(Index(mSubIndexer).ToPtr());
  
  // mCommandXboxController->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this] {mIMU->resetAngle();}, {mIMU}));
  mCommandXboxController->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this]
    {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_deg));}, {mIMU}));
};

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return mDrivetrain->getFollowPathCommand("'8' Path");
}
