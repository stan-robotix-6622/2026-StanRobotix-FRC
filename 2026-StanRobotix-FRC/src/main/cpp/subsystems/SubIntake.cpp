// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

#include <frc2/command/Commands.h>

#include "Constants.h"

SubIntake::SubIntake() {
    mIntakeMotor = new rev::spark::SparkMax(CANid::kMotorIntakeID, rev::spark::SparkLowLevel::MotorType::kBrushless);

    mEncoder = new rev::spark::SparkRelativeEncoder{mIntakeMotor->GetEncoder()};
 
    mIntakeMotorConfig = new rev::spark::SparkMaxConfig{};
    mIntakeMotorConfig->Inverted(IntakeConstants::kInverted);
    mIntakeMotorConfig->SetIdleMode(IntakeConstants::kIdleMode);
    mIntakeMotorConfig->encoder.PositionConversionFactor(1 / IntakeConstants::kGearRatio);
    mIntakeMotorConfig->encoder.VelocityConversionFactor(1 / IntakeConstants::kGearRatio);
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

void SubIntake::SetSpeed(double iSpeed)
{
    mIntakeMotor->Set(iSpeed);
}

frc2::CommandPtr SubIntake::getIntakeCommand() {
    return frc2::cmd::RunEnd(
        [this]
        {
            SetSpeed(IntakeConstants::kSpeed);
        },

        [this]
        {
            Stop();
        },
        {}
    );
}

void SubIntake::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("intake");
    builder.AddDoubleProperty("velocity rpm", [this] {return mEncoder->GetVelocity();}, nullptr);
    builder.AddDoubleProperty("wheel perimeter speed", [this] {return mEncoder->GetVelocity() * IntakeConstants::kWheelRadius.value() * 2 * std::numbers::pi / 60;}, nullptr);
}