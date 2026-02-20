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
  m_subShooter = new subShooter{};
  m_subFeeder = new SubFeeder{};
  // m_subIndexer = new SubIndexer{};
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

  m_driverController.Y().WhileTrue(frc2::cmd::RunEnd(
    [this] {std::cout << "Shoot" << std::endl;m_subShooter->setVelocity(m_driverController.GetRightTriggerAxis() * ShooterConstants::kVitesseVoulue);},
    [this] {m_subShooter->setVelocity(0_tps);}, {m_subShooter}));

  // m_driverController.Y().WhileTrue(frc2::cmd::RunEnd(
  //   [this] {std::cout << "Shoot" << std::endl;
  //     m_subShooter->setVoltage(m_driverController.GetRightTriggerAxis() * 8_V);
  //     frc::SmartDashboard::PutNumber("shooter voltage", m_driverController.GetRightTriggerAxis() * 8);},
  //   [this] {m_subShooter->setVelocity(0_tps);}, {m_subShooter}));
    
  m_driverController.B().WhileTrue(frc2::cmd::RunEnd(
    [this] {std::cout << "Feed" << std::endl;m_subFeeder->setVoltage(m_driverController.GetLeftTriggerAxis() * FeederConstants::kDesiredVoltage);},
    [this] {m_subFeeder->setVoltage(0_V);}, {m_subFeeder}));

    // m_driverController.A().WhileTrue(frc2::cmd::RunEnd(
    // [this] {std::cout << "Index" << std::endl;m_subIndexer->setVoltage(m_driverController.GetLeftTriggerAxis() * IndexerConstants::kDesiredVoltage);},
    // [this] {m_subIndexer->setVoltage(0_V);}, {m_subIndexer}));
      
  m_driverController.X().WhileTrue(Shoot(m_subShooter).ToPtr());
  mCommandXboxController->Y().WhileTrue(frc2::cmd::RunOnce([this] {mIMU->resetAngle();}, {mIMU}));
  m_driverController.RightBumper().WhileTrue(FeedShooter(m_subFeeder).ToPtr());
  //m_driverController.LeftBumper().WhileTrue(Index(m_subIndexer).ToPtr());
};


  
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no AutonomousCommand");
}


