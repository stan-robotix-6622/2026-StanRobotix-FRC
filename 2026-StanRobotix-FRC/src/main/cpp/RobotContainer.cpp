// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>



#include "commands/Autos.h"
#include "commands/ExampleCommand.h"



RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_subShooter = new subShooter{};
  m_subFeeder = new subFeeder{};
 
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
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_driverController.X().ToggleOnTrue(frc2::cmd::RunEnd(
    [this] {m_subShooter->setVelocity(m_driverController.GetRightTriggerAxis() * 1_tps);},
    [this] {m_subShooter->setVelocity(0_tps);}, {m_subShooter}));

  m_driverController.Y().ToggleOnTrue(frc2::cmd::RunEnd(
    [this] {m_subFeeder->setVoltage(m_driverController.GetLeftTriggerAxis() * 1_V);},
    [this] {m_subFeeder->setVoltage(0_V);}, {m_subFeeder}));


  frc2::Trigger([this] {
    return m_XboxController.GetAButtonPressed();
  }).OnTrue(FeedShooter(m_subFeeder).ToPtr());


 /* frc2::Trigger([this] {
    return m_XboxController.GetXButtonPressed();
  }).WhileTrue(Shoot(m_subShooter).ToPtr()); */
};


  
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}


