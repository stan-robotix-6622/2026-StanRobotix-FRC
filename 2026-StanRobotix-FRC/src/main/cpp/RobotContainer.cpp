// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include <iostream>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/FeedShooter.h"
#include "commands/Shoot.h"
#include "commands/Index.h"

RobotContainer::RobotContainer() {
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  frc::SmartDashboard::PutData("Xbox Controller", &mCommandXboxController->GetHID());

  // Initialize all of your commands and subsystems here
  mSubShooter = new SubShooter{};
  mSubFeeder = new SubFeeder{};
  // mSubIndexer = new SubIndexer{};
  mIMU = new SubIMU{};
  mDrivetrain = new SubDrivetrain{mIMU};

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

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  mCommandXboxController->Button(8).WhileTrue(mDrivetrain->Defer([this] {return mDrivetrain->getGoToDistanceFromHubCommand(111_in);}));
};

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no AutonomousCommand");
}
