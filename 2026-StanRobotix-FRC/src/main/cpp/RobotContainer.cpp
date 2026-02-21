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
  mDrivetrain = new SubDrivetrain{mIMU};
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

  mIMU->SetDefaultCommand(frc2::cmd::Run(
      [this]
      {
        frc::SmartDashboard::PutNumber("Drivetrain/Robot Rotation", mIMU->getRotation2d().Degrees().value());
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

  // mCommandXboxController->Y().WhileTrue(PivotIntake(m_SubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  // mCommandXboxController->B().WhileTrue(PivotIntake(m_SubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());
  m_driverController->Y().WhileTrue(FullIntake(m_SubIntake, m_SubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  m_driverController->B().WhileTrue(FullIntake(m_SubIntake, m_SubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());
  
  mCommandXboxController->X().WhileTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->RightBumper().WhileTrue(frc2::cmd::RunOnce([this] {mIMU->resetAngle();}, {mIMU}));
  // mCommandXboxController->RightBumper().WhileTrue(FeedShooter(mSubFeeder).ToPtr());
  // mCommandXboxController->LeftBumper().WhileTrue(Index(mSubIndexer).ToPtr());
};

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no AutonomousCommand");
}
