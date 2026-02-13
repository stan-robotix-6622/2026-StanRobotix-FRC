// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

SubIntake::SubIntake() {
    mIntakeMotor = new rev::spark::SparkMax(IntakeConstants::kMotorid, rev::spark::SparkLowLevel::MotorType::kBrushless);
 
    mIntakeMotorConfig = new rev::spark::SparkMaxConfig{};
    mIntakeMotorConfig->Inverted(IntakeConstants::kInverted);

    mIntakeMotor->Configure(*mIntakeMotorConfig, IntakeConstants::kReset, IntakeConstants::kPersist);
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {}

void SubIntake::Stop() {
    mIntakeMotor->StopMotor();
}

void SubIntake::SetVoltage(double iVoltage)
{
    mIntakeMotor->SetVoltage(units::volt_t(iVoltage));
}
