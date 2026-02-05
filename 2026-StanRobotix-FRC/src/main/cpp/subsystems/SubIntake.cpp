// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

SubIntake::SubIntake() {
     mIntakeMotor = new rev::spark::SparkMax(IntakeConstants::kMotorid, rev::spark::SparkLowLevel::MotorType::kBrushless);
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {}

void SubIntake::Stop() {
    mIntakeMotor->StopMotor();
}

void SubIntake::Keep()
{
    mIntakeMotor->Set(IntakeConstants::kSpeed);
}

void SubIntake::SetVoltage(double iVoltage)
{
    mIntakeMotor->SetVoltage(units::volt_t(iVoltage));
}