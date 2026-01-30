// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDrivingrControllerPort};

  // Initialize all of your commands and subsystems here
  mIMU = new SubIMU{};
  mDrivetrain = new SubDrivetrain{mIMU};

  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
      [this] {
      mDrivetrain->driveFieldRelative(-mCommandXboxController->GetLeftY(),
                                        -mCommandXboxController->GetLeftX(),
                                        -mCommandXboxController->GetRightX(),
                                        0.3);
     },
     {mDrivetrain}));

  mIMU->SetDefaultCommand(frc2::cmd::Run(
    [this] {
      // std::cout << mIMU->getRotation2d().Degrees().value() << std::endl;
    }, {mIMU}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // mCommandXboxController->X().OnTrue(pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("Example Path")));
  mCommandXboxController->Y().WhileTrue(frc2::cmd::RunOnce([this] {mIMU->resetAngle();}, {mIMU}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no AutonomousCommand");
}
