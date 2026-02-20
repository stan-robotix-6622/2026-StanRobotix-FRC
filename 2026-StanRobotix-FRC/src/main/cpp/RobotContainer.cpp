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

  mCommandXboxController->Y().WhileTrue(frc2::cmd::RunEnd(
    [this] {std::cout << "Shoot" << std::endl;mSubShooter->setVelocity(mCommandXboxController->GetRightTriggerAxis() * ShooterConstants::kVitesseVoulue);},
    [this] {mSubShooter->setVelocity(0_tps);}, {mSubShooter}));

  // mCommandXboxController->Y().WhileTrue(frc2::cmd::RunEnd(
  //   [this] {std::cout << "Shoot" << std::endl;
  //     mSubShooter->setVoltage(mCommandXboxController->GetRightTriggerAxis() * 8_V);
  //     frc::SmartDashboard::PutNumber("shooter voltage", mCommandXboxController->GetRightTriggerAxis() * 8);},
  //   [this] {mSubShooter->setVelocity(0_tps);}, {mSubShooter}));
    
  mCommandXboxController->B().WhileTrue(frc2::cmd::RunEnd(
    [this] {std::cout << "Feed" << std::endl;mSubFeeder->setVoltage(mCommandXboxController->GetLeftTriggerAxis() * FeederConstants::kDesiredVoltage);},
    [this] {mSubFeeder->setVoltage(0_V);}, {mSubFeeder}));

    // mCommandXboxController->A().WhileTrue(frc2::cmd::RunEnd(
    // [this] {std::cout << "Index" << std::endl;mSubIndexer->setVoltage(mCommandXboxController->GetLeftTriggerAxis() * IndexerConstants::kDesiredVoltage);},
    // [this] {mSubIndexer->setVoltage(0_V);}, {mSubIndexer}));
      
  mCommandXboxController->X().WhileTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->Y().WhileTrue(frc2::cmd::RunOnce([this] {mIMU->resetAngle();}, {mIMU}));
  mCommandXboxController->RightBumper().WhileTrue(FeedShooter(mSubFeeder).ToPtr());
  //mCommandXboxController->LeftBumper().WhileTrue(Index(mSubIndexer).ToPtr());
};

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no AutonomousCommand");
}
