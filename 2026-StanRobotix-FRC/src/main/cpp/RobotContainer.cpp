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

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_subShooter = new subShooter{};
  m_subFeeder = new SubFeeder{};
 
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_driverController.Y().WhileTrue(frc2::cmd::RunEnd(
    [this] {std::cout << "Shoot" << std::endl;m_subShooter->setVelocity(m_driverController.GetRightTriggerAxis() * subShooterConstants::kVitesseVoulue);},
    [this] {m_subShooter->setVelocity(0_tps);}, {m_subShooter}));
    
  m_driverController.B().WhileTrue(frc2::cmd::RunEnd(
    [this] {std::cout << "Feed" << std::endl;m_subFeeder->setVoltage(m_driverController.GetLeftTriggerAxis() * SubFeederConstants::kVoltage);},
    [this] {m_subFeeder->setVoltage(0_V);}, {m_subFeeder}));
      
  m_driverController.X().WhileTrue(Shoot(m_subShooter).ToPtr());

  m_driverController.A().WhileTrue(FeedShooter(m_subFeeder).ToPtr());


 /* frc2::Trigger([this] {
    return m_XboxController.GetXButtonPressed();
  }).WhileTrue(Shoot(m_subShooter).ToPtr()); */
};


  
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}


