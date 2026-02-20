// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include <iostream>

#include "commands/GoToDistanceFromHub.h"

RobotContainer::RobotContainer() {
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  frc::SmartDashboard::PutData("Xbox Controller", &mCommandXboxController->GetHID());

  // Initialize all of your commands and subsystems here
  mIMU = new SubIMU{};
  mDrivetrain = new SubDrivetrain{mIMU};

  // mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
  //     [this]
  //     {
  //       mDrivetrain->driveFieldRelative(-mCommandXboxController->GetLeftY(),
  //                                       -mCommandXboxController->GetLeftX(),
  //                                       -mCommandXboxController->GetRightX(),
  //                                       (1 - mCommandXboxController->GetRightTriggerAxis()) / 4);
  //     },
  //     {mDrivetrain}));

  // mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
  //     [this]
  //     {
  //       mDrivetrain->mesureSwerveFeedforward(
  //           units::volt_t(mCommandXboxController->GetRightTriggerAxis() * 12),
  //           units::volt_t(mCommandXboxController->GetLeftTriggerAxis() * 12));
  //     },
  //     {mDrivetrain}));

  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
    [this] {
      if (mCommandXboxController->GetHID().GetBButton() == true)
      {
        std::cout << "The Button B is held" << std::endl;
      }
    }, {mDrivetrain})
  );

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

  // mCommandXboxController->X().OnTrue(pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("Example Path")));
  mCommandXboxController->Y().WhileTrue(frc2::cmd::RunOnce([this] {mIMU->resetAngle();}, {mIMU}));
  mCommandXboxController->A().OnTrue(frc2::cmd::Print("Button A is pressed"));
  frc2::Trigger(
    [this] {return mCommandXboxController->GetHID().GetBButton();}
  ).WhileTrue(frc2::cmd::Print("Button B is held"));
  mCommandXboxController->X().OnTrue(frc2::cmd::Parallel(
    frc2::cmd::Race(
      GoToDistanceFromHub(mDrivetrain, 4_m).ToPtr(),
      frc2::cmd::Wait(2_s)),
    frc2::cmd::Print("Calculating...")));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no AutonomousCommand");
}
