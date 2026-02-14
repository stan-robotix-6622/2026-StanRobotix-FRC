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

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  m_driverController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  m_SubIntake = new SubIntake;
  m_SubPivotIntake = new SubPivotIntake;

  m_SubPivotIntake->SetDefaultCommand(frc2::cmd::Run([this]
  {
    m_SubPivotIntake->KeepPosition();
  }, {m_SubPivotIntake}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  m_driverController->Y().WhileTrue(PivotIntake(m_SubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  m_driverController->B().WhileTrue(PivotIntake(m_SubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());
  m_driverController->X().WhileTrue(frc2::cmd::Run([this] {m_SubIntake->SetVoltage(m_driverController->GetRightTriggerAxis() * 3.0);}, {m_SubIntake}));
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController->B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}